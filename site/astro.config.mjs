// @ts-check
import starlight from "@astrojs/starlight";
import { defineConfig } from "astro/config";
import starlightBlog from "starlight-blog";

const repo = "https://github.com/joeferner/rpi-hal";

export default defineConfig({
  // A GitHub Pages project site: served under the repository's name.
  site: "https://joeferner.github.io",
  base: "/rpi-hal",
  integrations: [
    starlight({
      title: "rpi-hal",
      description:
        "Bare-metal Rust for the Raspberry Pi: an embedded-hal HAL, Embassy support, application bring-up, and a UART loader.",
      social: [{ icon: "github", label: "GitHub", href: repo }],
      editLink: { baseUrl: `${repo}/edit/main/site/` },
      lastUpdated: true,
      plugins: [
        starlightBlog({
          title: "Blog",
          navigation: "header-end",
        }),
      ],
      sidebar: [
        {
          label: "Start here",
          items: ["getting-started", "start"],
        },
        {
          label: "Crates",
          items: ["crates/hal", "crates/embassy", "crates/kickstart", "crates/loader"],
        },
        {
          label: "Reference",
          items: ["boards"],
        },
      ],
    }),
  ],
});
