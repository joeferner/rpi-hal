/*
 * Headless GBM + EGL + GLES2 program that compiles and draws with the
 * exact vertex/fragment GLSL the bare-metal cube demo will use, so
 * Mesa's `vc4` driver's real shader compiler runs against it and its
 * debug-dump output (VC4_DEBUG=... below) can be captured.
 *
 * vc4 compiles lazily against the full draw state (attribs bound,
 * texture bound, etc.), not at glCompileShader/glLinkProgram time --
 * the glDrawArrays call below is what actually triggers codegen.
 *
 * Build (on the Pi 3, under Debian):
 *   sudo apt install build-essential pkg-config libgbm-dev libegl-dev \
 *       libgles2-mesa-dev libdrm-dev
 *   gcc -o shader_dump shader_dump.c \
 *       $(pkg-config --cflags --libs gbm egl glesv2)
 *
 * Run, capturing the driver's debug dump:
 *   VC4_DEBUG=qpu,shaderdb,cl ./shader_dump 2> dump.log
 *
 * VC4_DEBUG flag names above are recalled from Mesa's vc4_screen.c
 * debug-options table, not confirmed against the exact Mesa version
 * Debian ships -- if `dump.log` doesn't contain anything resembling
 * QPU disassembly, grep the installed driver's source
 * (`apt source mesa`, then `grep -n debug_options
 * src/gallium/drivers/vc4/vc4_screen.c`) for the real flag names.
 */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <gbm.h>

static const char *VERTEX_SHADER_SRC =
    "attribute vec4 aPosition;\n"
    "attribute vec2 aTexCoord;\n"
    "uniform mat4 uMvp;\n"
    "varying vec2 vTexCoord;\n"
    "void main() {\n"
    "    gl_Position = uMvp * aPosition;\n"
    "    vTexCoord = aTexCoord;\n"
    "}\n";

static const char *FRAGMENT_SHADER_SRC =
    "precision mediump float;\n"
    "varying vec2 vTexCoord;\n"
    "uniform sampler2D uTexture;\n"
    "void main() {\n"
    "    gl_FragColor = texture2D(uTexture, vTexCoord);\n"
    "}\n";

static GLuint compile(GLenum type, const char *src) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, NULL);
    glCompileShader(shader);
    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetShaderInfoLog(shader, sizeof(log), NULL, log);
        fprintf(stderr, "shader compile failed: %s\n", log);
        exit(1);
    }
    return shader;
}

int main(void) {
    int fd = open("/dev/dri/renderD128", O_RDWR);
    if (fd < 0) {
        perror("open /dev/dri/renderD128");
        return 1;
    }

    struct gbm_device *gbm = gbm_create_device(fd);
    if (!gbm) {
        fprintf(stderr, "gbm_create_device failed\n");
        return 1;
    }

    PFNEGLGETPLATFORMDISPLAYEXTPROC get_platform_display =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress(
            "eglGetPlatformDisplayEXT");
    if (!get_platform_display) {
        fprintf(stderr, "eglGetPlatformDisplayEXT not available\n");
        return 1;
    }

    EGLDisplay dpy = get_platform_display(EGL_PLATFORM_GBM_KHR, gbm, NULL);
    if (dpy == EGL_NO_DISPLAY) {
        fprintf(stderr, "eglGetPlatformDisplayEXT(GBM) failed\n");
        return 1;
    }

    if (!eglInitialize(dpy, NULL, NULL)) {
        fprintf(stderr, "eglInitialize failed\n");
        return 1;
    }
    eglBindAPI(EGL_OPENGL_ES_API);

    EGLint config_attribs[] = {
        /* Mesa's GBM EGL platform only advertises EGL_WINDOW_BIT --
         * a GBM-backed surface has to wrap a real gbm_surface (buffer
         * object), so EGL_PBUFFER_BIT configs don't exist here even
         * though we never display the result anywhere. */
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE,        8,
        EGL_GREEN_SIZE,      8,
        EGL_BLUE_SIZE,       8,
        EGL_ALPHA_SIZE,      8,
        EGL_NONE,
    };
    EGLConfig config;
    EGLint num_configs = 0;
    if (!eglChooseConfig(dpy, config_attribs, &config, 1, &num_configs) ||
        num_configs == 0) {
        fprintf(stderr, "eglChooseConfig found no matching config\n");
        return 1;
    }

    struct gbm_surface *gbm_surface = gbm_surface_create(
        gbm, 4, 4, GBM_FORMAT_ARGB8888, GBM_BO_USE_RENDERING);
    if (!gbm_surface) {
        fprintf(stderr, "gbm_surface_create failed\n");
        return 1;
    }

    EGLSurface surface = eglCreateWindowSurface(
        dpy, config, (EGLNativeWindowType)gbm_surface, NULL);
    if (surface == EGL_NO_SURFACE) {
        fprintf(stderr, "eglCreateWindowSurface failed\n");
        return 1;
    }

    EGLint context_attribs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
    EGLContext ctx =
        eglCreateContext(dpy, config, EGL_NO_CONTEXT, context_attribs);
    if (ctx == EGL_NO_CONTEXT) {
        fprintf(stderr, "eglCreateContext failed\n");
        return 1;
    }
    if (!eglMakeCurrent(dpy, surface, surface, ctx)) {
        fprintf(stderr, "eglMakeCurrent failed\n");
        return 1;
    }

    fprintf(stderr, "GL_RENDERER: %s\n", (const char *)glGetString(GL_RENDERER));
    fprintf(stderr, "GL_VERSION:  %s\n", (const char *)glGetString(GL_VERSION));

    GLuint vs = compile(GL_VERTEX_SHADER, VERTEX_SHADER_SRC);
    GLuint fs = compile(GL_FRAGMENT_SHADER, FRAGMENT_SHADER_SRC);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    /* Pin attribute locations to the layout the bare-metal vertex
     * buffer will use, so the VPM read config the compiler picks here
     * matches what gets replicated in the CL builder later. */
    glBindAttribLocation(prog, 0, "aPosition");
    glBindAttribLocation(prog, 1, "aTexCoord");
    glLinkProgram(prog);
    GLint linked = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &linked);
    if (!linked) {
        char log[4096];
        glGetProgramInfoLog(prog, sizeof(log), NULL, log);
        fprintf(stderr, "link failed: %s\n", log);
        return 1;
    }
    glUseProgram(prog);

    GLint mvp_loc = glGetUniformLocation(prog, "uMvp");
    GLint tex_loc = glGetUniformLocation(prog, "uTexture");
    fprintf(stderr, "uMvp uniform location=%d\n", mvp_loc);
    fprintf(stderr, "uTexture uniform location=%d\n", tex_loc);

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    unsigned char pixel[4] = {255, 0, 0, 255};
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 pixel);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glUniform1i(tex_loc, 0);

    static const float identity[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    };
    glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, identity);

    static const float verts[] = {
        /* x,  y,  z, w,    u,   v */
        -1, -1, 0, 1,      0,   0,
         1, -1, 0, 1,      1,   0,
         0,  1, 0, 1,    0.5,   1,
    };
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), verts);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                           verts + 4);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    /* The real cube demo draws from an index buffer (shared vertices
     * between triangles), not glDrawArrays -- use glDrawElements here
     * too so the captured BCL reflects the CL packet the bare-metal
     * renderer will actually need to emit. */
    static const unsigned short indices[] = {0, 1, 2};

    glViewport(0, 0, 4, 4);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_SHORT, indices);
    glFinish();

    /* glFinish() alone never triggered an "RCL:" dump alongside the
     * "BCL:" one in earlier runs -- on a window-system-backed EGL
     * surface, actual tile rendering may be deferred until present
     * time rather than happening as part of the draw call itself.
     * eglSwapBuffers is the real "present this frame" signal; testing
     * whether that's what was missing, rather than assuming the debug
     * flag just doesn't cover the render pass. */
    eglSwapBuffers(dpy, surface);

    fprintf(stderr, "draw complete\n");
    return 0;
}
