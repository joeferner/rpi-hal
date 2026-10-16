// Renders the project template the way `cargo generate` does. One module,
// run in two places: the browser (the project selector) and Node
// (scripts/scaffold-cli.mjs, which CI diffs against cargo-generate's own
// output for every board, so the zip and `cargo generate` can't drift).
//
// The variables come from template/boards.json, which scripts/template/gen.py
// writes from the same table as the cargo-generate hook. So this module
// derives nothing about a board; it only renders.

import { Liquid } from "liquidjs";

// cargo-generate turns a project name into kebab-case. Accepting only
// names that are already kebab-case means there's no conversion here to
// keep in step with it.
export const NAME_PATTERN = /^[a-z][a-z0-9]*(-[a-z0-9]+)*$/;

/** The board's variant for `arch`, or its only one when `arch` is omitted. */
export function findVariant(boards, boardId, arch) {
  const board = boards.boards.find((b) => b.id === boardId);
  if (!board) {
    throw new Error(`unknown board ${boardId}`);
  }
  const chosen = arch ?? board.arches[0];
  const variant = board.variants[chosen];
  if (!variant) {
    throw new Error(`no ${chosen} build for ${boardId}`);
  }
  return variant;
}

/**
 * Renders every template file for one board variant.
 *
 * @param {Record<string, string>} files template path -> contents
 * @param {{vars: Record<string, string | boolean>}} variant from findVariant
 * @param {string} name the project name
 * @returns {Promise<Record<string, string>>} project path -> contents
 */
export async function renderProject(files, variant, name) {
  if (!NAME_PATTERN.test(name)) {
    throw new Error(`"${name}" isn't a valid project name: lowercase letters, digits and single hyphens, starting with a letter`);
  }
  // Undefined variables are errors, as they are in cargo-generate: a typo
  // in a template has to fail rather than render as an empty string.
  const engine = new Liquid({ strictVariables: true, strictFilters: true });
  const vars = {
    "project-name": name,
    crate_name: name.replaceAll("-", "_"),
    ...variant.vars,
  };
  const out = {};
  for (const [path, text] of Object.entries(files)) {
    out[path] = await engine.parseAndRender(text, vars);
  }
  return out;
}
