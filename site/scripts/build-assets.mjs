// Writes what the project selector downloads, into public/ before every
// build (`npm run build` runs it first):
//
//   public/scaffold/boards.json    template/boards.json, as is
//   public/scaffold/template.json  every template file a project gets
//   public/card/<sha256>           each firmware file and loader image,
//                                  named by its checksum
//
// The browser can't fetch the firmware or the loader images itself: GitHub
// serves them without the headers that let a page on another origin read
// them. So they're downloaded here, checked against boards.json, and
// served from this site. Naming them by checksum means one copy each,
// however many boards share it.
//
// Downloads are kept in .cache/ between local builds.

import { createHash } from "node:crypto";
import { copyFile, mkdir, readFile, rm, writeFile } from "node:fs/promises";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

import { templateFiles } from "./template-files.mjs";

const site = join(dirname(fileURLToPath(import.meta.url)), "..");
const templateDir = join(site, "..", "template");
const cacheDir = join(site, ".cache", "card");
const publicScaffold = join(site, "public", "scaffold");
const publicCard = join(site, "public", "card");

const sha256 = (bytes) => createHash("sha256").update(bytes).digest("hex");

async function cached(sha) {
  try {
    const bytes = await readFile(join(cacheDir, sha));
    return sha256(bytes) === sha;
  } catch {
    return false;
  }
}

async function fetchVerified({ url, sha256: expected, name }) {
  if (await cached(expected)) {
    return;
  }
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`${name}: ${url} answered ${response.status}`);
  }
  const bytes = Buffer.from(await response.arrayBuffer());
  const actual = sha256(bytes);
  if (actual !== expected) {
    throw new Error(`${name}: checksum mismatch for ${url}\n  expected ${expected}\n  got      ${actual}`);
  }
  await writeFile(join(cacheDir, expected), bytes);
  console.log(`fetched ${name} (${bytes.length} bytes) from ${url}`);
}

const boardsText = await readFile(join(templateDir, "boards.json"), "utf8");
const boards = JSON.parse(boardsText);

await rm(publicScaffold, { recursive: true, force: true });
await rm(publicCard, { recursive: true, force: true });
await mkdir(publicScaffold, { recursive: true });
await mkdir(publicCard, { recursive: true });
await mkdir(cacheDir, { recursive: true });

await writeFile(join(publicScaffold, "boards.json"), boardsText);
const files = await templateFiles(templateDir, boards);
await writeFile(join(publicScaffold, "template.json"), JSON.stringify(files));

// Every card file of every variant, once per checksum.
const unique = new Map();
for (const board of boards.boards) {
  for (const variant of Object.values(board.variants)) {
    for (const file of variant.card) {
      unique.set(file.sha256, file);
    }
  }
}
for (const file of unique.values()) {
  await fetchVerified(file);
  await copyFile(join(cacheDir, file.sha256), join(publicCard, file.sha256));
}

console.log(`assets: ${Object.keys(files).length} template files, ${unique.size} card files`);
