import { docsLoader } from "@astrojs/starlight/loaders";
import { docsSchema } from "@astrojs/starlight/schema";
import { defineCollection } from "astro:content";
import { blogSchema } from "starlight-blog/schema";

export const collections = {
  docs: defineCollection({
    loader: docsLoader(),
    // Blog posts live in the docs collection under blog/, with the extra
    // frontmatter (date, authors, tags, ...) starlight-blog adds.
    schema: docsSchema({ extend: (context) => blogSchema(context) }),
  }),
};
