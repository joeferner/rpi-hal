/*
 * Reference implementation of the bare-metal rotating textured cube,
 * drawn by Mesa's real `vc4` driver on a Pi 3 under Debian.
 *
 * Everything here is deliberately identical to `examples/gpu_cube.rs`:
 * the same 24 vertices and 36 indices, the same 4x4 red/white
 * checkerboard texture, the same perspective/view/model matrices, the
 * same 512x512 render target, and depth testing with GL_LESS. That
 * makes it useful two different ways:
 *
 *  1. As a *picture*. It writes each frame out as a .ppm. If Mesa
 *     renders this cube correctly and the bare-metal version does not,
 *     the geometry, texture coordinates, matrix math and shaders are
 *     all exonerated and the fault is in the hand-built control lists.
 *     If Mesa gets it wrong too, the fault is in the scene itself.
 *
 *  2. As a *capture*. The other program here (shader_dump.c) draws a
 *     single flat triangle with no depth testing, at 4x4, which is why
 *     every question about depth, multi-tile binning, or how a real
 *     cube's state differs had to be answered by reading kernel source
 *     and extrapolating from it. Running this under VC4_DEBUG gives the
 *     annotated control lists for the *actual* scene instead,
 *     including the depth configuration and an 8x8 tile layout.
 *
 * Build (on the Pi 3, under Debian):
 *   sudo apt install build-essential pkg-config libgbm-dev libegl-dev \
 *       libgles2-mesa-dev libdrm-dev
 *   gcc -O2 -o cube_reference cube_reference.c -lm \
 *       $(pkg-config --cflags --libs gbm egl glesv2)
 *
 * Run, just to look at the result:
 *   ./cube_reference
 *   # writes cube_000.ppm, cube_015.ppm, ... one per angle
 *
 * Run, capturing the driver's shader and control-list dumps. Use a
 * single frame, or the dump is enormous and every draw looks alike:
 *   VC4_DEBUG=qpu,qir,shaderdb,cl ./cube_reference 1 2> cube_dump.log
 *
 * The angles are chosen to include the case the bare-metal version
 * visibly gets wrong: face-on (0 degrees) renders correctly there,
 * while partway through a quarter turn the left side is cut off and
 * triangles drop out of the face.
 */
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <gbm.h>

#define WIDTH 512
#define HEIGHT 512

/* ---- Shaders: the same GLSL examples/gpu_cube.rs's bytes came from ---- */

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

/* ---- Geometry: identical to gpu_cube.rs's VERTICES/INDICES ----
 *
 * 24 vertices, 4 per face rather than 8 shared corners, so each face
 * carries its own texture coordinates. Each vertex is x, y, z, w, u, v.
 */
static const float VERTICES[24 * 6] = {
    /* +X */
     1, -1, -1, 1,  0, 0,
     1,  1, -1, 1,  1, 0,
     1,  1,  1, 1,  1, 1,
     1, -1,  1, 1,  0, 1,
    /* -X */
    -1, -1,  1, 1,  0, 0,
    -1,  1,  1, 1,  1, 0,
    -1,  1, -1, 1,  1, 1,
    -1, -1, -1, 1,  0, 1,
    /* +Y */
    -1,  1, -1, 1,  0, 0,
    -1,  1,  1, 1,  1, 0,
     1,  1,  1, 1,  1, 1,
     1,  1, -1, 1,  0, 1,
    /* -Y */
    -1, -1,  1, 1,  0, 0,
    -1, -1, -1, 1,  1, 0,
     1, -1, -1, 1,  1, 1,
     1, -1,  1, 1,  0, 1,
    /* +Z */
    -1, -1,  1, 1,  0, 0,
     1, -1,  1, 1,  1, 0,
     1,  1,  1, 1,  1, 1,
    -1,  1,  1, 1,  0, 1,
    /* -Z */
     1, -1, -1, 1,  0, 0,
    -1, -1, -1, 1,  1, 0,
    -1,  1, -1, 1,  1, 1,
     1,  1, -1, 1,  0, 1,
};

static const unsigned short INDICES[36] = {
     0,  1,  2,   0,  2,  3,
     4,  5,  6,   4,  6,  7,
     8,  9, 10,   8, 10, 11,
    12, 13, 14,  12, 14, 15,
    16, 17, 18,  16, 18, 19,
    20, 21, 22,  20, 22, 23,
};

/* ---- Matrix math: column-major, matching gpu_cube.rs's `math` ---- */

static void mat_multiply(const float *a, const float *b, float *out) {
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += a[k * 4 + row] * b[col * 4 + k];
            }
            out[col * 4 + row] = sum;
        }
    }
}

/* Rotation about Y only, matching SINGLE_AXIS_ROTATION in the Rust. */
static void mat_rotation_y(float angle, float *out) {
    float s = sinf(angle), c = cosf(angle);
    float m[16] = {
          c, 0, -s, 0,
          0, 1,  0, 0,
          s, 0,  c, 0,
          0, 0,  0, 1,
    };
    memcpy(out, m, sizeof(m));
}

static void mat_translation(float x, float y, float z, float *out) {
    float m[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        x, y, z, 1,
    };
    memcpy(out, m, sizeof(m));
}

static void mat_perspective(float fovy, float aspect, float near, float far,
                            float *out) {
    float f = 1.0f / tanf(fovy / 2.0f);
    float m[16] = {
        f / aspect, 0, 0, 0,
        0, f, 0, 0,
        0, 0, (far + near) / (near - far), -1,
        0, 0, (2.0f * far * near) / (near - far), 0,
    };
    memcpy(out, m, sizeof(m));
}

static void print_matrix(const char *label, const float *m) {
    fprintf(stderr, "%s = [", label);
    for (int i = 0; i < 16; i++) {
        fprintf(stderr, "%g%s", m[i], i == 15 ? "" : ", ");
    }
    fprintf(stderr, "]\n");
}

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

/* Writes the framebuffer as a binary PPM, flipping vertically since
 * glReadPixels returns bottom-up and PPM is top-down. */
static void write_ppm(const char *path, const unsigned char *rgba) {
    FILE *f = fopen(path, "wb");
    if (!f) {
        perror(path);
        return;
    }
    fprintf(f, "P6\n%d %d\n255\n", WIDTH, HEIGHT);
    for (int y = HEIGHT - 1; y >= 0; y--) {
        for (int x = 0; x < WIDTH; x++) {
            fwrite(&rgba[(y * WIDTH + x) * 4], 1, 3, f);
        }
    }
    fclose(f);
    fprintf(stderr, "wrote %s\n", path);
}

int main(int argc, char **argv) {
    /* Default to a sweep through a quarter turn; pass a count to limit
     * it (use 1 when capturing VC4_DEBUG output). */
    int frames = 7;
    if (argc > 1) {
        frames = atoi(argv[1]);
        if (frames < 1) frames = 1;
    }

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
        /* Mesa's GBM EGL platform only advertises EGL_WINDOW_BIT. */
        EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE,        8,
        EGL_GREEN_SIZE,      8,
        EGL_BLUE_SIZE,       8,
        EGL_ALPHA_SIZE,      8,
        /* Unlike shader_dump.c, this one needs a real depth buffer --
         * occlusion is half of what is being verified. */
        EGL_DEPTH_SIZE,      24,
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
        gbm, WIDTH, HEIGHT, GBM_FORMAT_ARGB8888, GBM_BO_USE_RENDERING);
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
    GLint depth_bits = 0;
    glGetIntegerv(GL_DEPTH_BITS, &depth_bits);
    fprintf(stderr, "GL_DEPTH_BITS: %d\n", depth_bits);

    GLuint vs = compile(GL_VERTEX_SHADER, VERTEX_SHADER_SRC);
    GLuint fs = compile(GL_FRAGMENT_SHADER, FRAGMENT_SHADER_SRC);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    /* Same pinned locations as shader_dump.c, so the VPM layout the
     * compiler picks matches what the bare-metal shader record says. */
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

    /* 4x4 red/white checkerboard, authored R,G,B,A -- GL takes RGBA
     * here regardless of what byte order the hardware ends up wanting
     * internally, which is precisely the difference worth checking
     * against the bare-metal version's B,G,R,A texels. */
    unsigned char texture[4 * 4 * 4];
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            int i = (y * 4 + x) * 4;
            int red = ((x + y) % 2) == 0;
            texture[i + 0] = 255;
            texture[i + 1] = red ? 0 : 255;
            texture[i + 2] = red ? 0 : 255;
            texture[i + 3] = 255;
        }
    }
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 4, 4, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glUniform1i(tex_loc, 0);

    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          VERTICES);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          VERTICES + 4);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    /* Depth testing on, LESS, exactly as the bare-metal configuration
     * bits ask for. Culling stays off, matching the Rust's deliberate
     * choice to let depth alone provide occlusion. */
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDisable(GL_CULL_FACE);

    glViewport(0, 0, WIDTH, HEIGHT);
    /* Same clear as the bare-metal demo: R=0x20, G=0x40, B=0x80. */
    glClearColor(0x20 / 255.0f, 0x40 / 255.0f, 0x80 / 255.0f, 1.0f);

    float projection[16];
    mat_perspective(1.0f, (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f,
                    projection);

    unsigned char *pixels = malloc((size_t)WIDTH * HEIGHT * 4);
    if (!pixels) {
        fprintf(stderr, "out of memory\n");
        return 1;
    }

    for (int frame = 0; frame < frames; frame++) {
        /* 15 degrees per step: 0 is face-on (which the bare-metal
         * version renders correctly) and the later steps cover the
         * partway-round positions where it goes wrong. */
        float degrees = frame * 15.0f;
        float angle = degrees * (float)M_PI / 180.0f;

        float model[16], view[16], view_model[16], mvp[16];
        mat_rotation_y(angle, model);
        mat_translation(0.0f, 0.0f, -4.0f, view);
        mat_multiply(view, model, view_model);
        mat_multiply(projection, view_model, mvp);

        if (frame == 0) {
            print_matrix("mvp", mvp);
        }
        fprintf(stderr, "frame %d: %g degrees\n", frame, degrees);

        glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, mvp);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, INDICES);
        glFinish();

        glReadPixels(0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

        /* Report the centre pixel the same way the bare-metal demo
         * does, so the two are directly comparable. */
        unsigned int centre;
        memcpy(&centre,
               &pixels[((HEIGHT / 2) * WIDTH + (WIDTH / 2)) * 4],
               sizeof(centre));
        fprintf(stderr, "  centre=0x%08x\n", centre);

        char path[64];
        snprintf(path, sizeof(path), "cube_%03d.ppm", (int)degrees);
        write_ppm(path, pixels);

        eglSwapBuffers(dpy, surface);
    }

    free(pixels);
    fprintf(stderr, "done\n");
    return 0;
}
