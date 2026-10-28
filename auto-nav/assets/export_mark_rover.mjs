// Export the exact cached Isaac Mark Rover through Isaac's own CAD pipeline.
// Usage: node auto-nav/assets/export_mark_rover.mjs [isaac-repository]
import { execFileSync } from "node:child_process";
import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";
import { createHash } from "node:crypto";

const isaac = resolve(process.argv[2] || "/Users/tonystark/Desktop/isaacprotocol");
const output = dirname(fileURLToPath(import.meta.url));
const designId = "94612c06b9f5a5d045b4a1b1d3e31ec867bf1db483d3a065802e13713bffa8d9";
const source = join(isaac, "simulation/robot_packages/local-designs", `${designId}.json`);
const sourceBytes = readFileSync(source);
if (createHash("sha256").update(sourceBytes).digest("hex") !== designId)
  throw new Error("Saved Mark Rover CAD source failed its SHA-256 integrity check");
const document = JSON.parse(sourceBytes);
const { exportCadMJCF, kernelParts } = await import(pathToFileURL(join(isaac, "lib/cad/exports.js")));
const { densityFor } = await import(pathToFileURL(join(isaac, "lib/cad/model.js")));
const python = join(isaac, "simulation/venv/bin/python");
const geometries = JSON.parse(execFileSync(python, ["-c", `
import json,sys
from simulation.cad.kernel import mesh_for
parts=json.load(sys.stdin)
print(json.dumps({p['id']:mesh_for(p) for p in parts}))
`], {cwd: isaac,
      input: JSON.stringify(kernelParts(document, false).map(p => ({...p, density: densityFor(p)}))),
      encoding: "utf8", timeout: 180_000, maxBuffer: 128 * 1024 * 1024}));
const xml = exportCadMJCF(document, geometries);
writeFileSync(join(output, "mark_rover.xml"), xml);
writeFileSync(join(output, "mark_rover.isaac-cad.json"), sourceBytes);
writeFileSync(join(output, "mark_rover.provenance.json"), JSON.stringify({
  name: document.name,
  designId,
  source: "Isaac Protocol's saved local Mark Rover design; copied unchanged",
  sourcePath: source,
  exporter: "lib/cad/exports.js exportCadMJCF + simulation/cad/kernel.py mesh_for",
  parts: document.parts.length,
  cameras: document.cameras.map(c => c.id),
  drives: document.parts.filter(p => p.actuator).map(p => ({name:p.name, mode:p.actuator.mode})),
  xmlSha256: createHash("sha256").update(xml).digest("hex"),
  hardwareValidated: false,
}, null, 2) + "\n");
console.log(JSON.stringify({parts: document.parts.length, xmlBytes: Buffer.byteLength(xml), output}, null, 2));
