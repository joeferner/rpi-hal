// Renders a project the way the site's project selector does, into a
// directory, so CI can diff it against `cargo generate`'s output.
//
//   node site/scripts/scaffold-cli.mjs --board pi3 [--arch 64] --name demo --out DIR
//
// Writes DIR/<name>/. Only the project files: the selector's zip also
// carries the card's firmware and loader image, which cargo-generate
// leaves to `make sd-card`.

import { mkdir, readFile, writeFile } from "node:fs/promises";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import { parseArgs } from "node:util";

import { findVariant, renderProject } from "../src/lib/scaffold.mjs";
import { templateFiles } from "./template-files.mjs";

const { values } = parseArgs({
  options: {
    board: { type: "string" },
    arch: { type: "string" },
    name: { type: "string" },
    out: { type: "string" },
  },
});
if (!values.board || !values.name || !values.out) {
  console.error("usage: scaffold-cli.mjs --board <id> [--arch <arch>] --name <name> --out <dir>");
  process.exit(2);
}

const templateDir = join(dirname(fileURLToPath(import.meta.url)), "..", "..", "template");
const boards = JSON.parse(await readFile(join(templateDir, "boards.json"), "utf8"));
const variant = findVariant(boards, values.board, values.arch);
const project = await renderProject(await templateFiles(templateDir, boards), variant, values.name);

for (const [path, text] of Object.entries(project)) {
  const dest = join(values.out, values.name, path);
  await mkdir(dirname(dest), { recursive: true });
  await writeFile(dest, text);
}
