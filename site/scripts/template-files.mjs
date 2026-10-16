// The files in template/ that cargo-generate copies into a project: all of
// them except what boards.json's `ignore` lists (the board table and its
// plumbing) and cargo-generate.toml itself. Shared by build-assets.mjs and
// scaffold-cli.mjs, so the browser and the CI check render the same set.

import { readdir, readFile } from "node:fs/promises";
import { join, relative, sep } from "node:path";

/** template path ("src/main.rs") -> contents, for every file a project gets. */
export async function templateFiles(templateDir, boards) {
  const skip = new Set([...boards.ignore, "cargo-generate.toml"]);
  const files = {};
  async function walk(dir) {
    for (const entry of await readdir(dir, { withFileTypes: true })) {
      const full = join(dir, entry.name);
      const rel = relative(templateDir, full).split(sep).join("/");
      if (skip.has(rel.split("/")[0])) {
        continue;
      }
      if (entry.isDirectory()) {
        await walk(full);
      } else {
        files[rel] = await readFile(full, "utf8");
      }
    }
  }
  await walk(templateDir);
  return Object.fromEntries(Object.entries(files).sort(([a], [b]) => a.localeCompare(b)));
}
