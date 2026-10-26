// Build every printable part as a binary STL (print orientation) + a mesh report.
//   node build_stl.js [params.json] [--jscad <path to @jscad/modeling>]
const fs = require('fs');
const path = require('path');

const args = process.argv.slice(2);
const ji = args.indexOf('--jscad');
const jscadPath = ji >= 0 ? args[ji + 1] : (process.env.JSCAD_MODELING || '@jscad/modeling');
const jscad = require(jscadPath);
const params = args[0] && !args[0].startsWith('--') ? JSON.parse(fs.readFileSync(args[0], 'utf8')) : {};
const EVP = require('./evp_parts.js')(jscad);
const M = require('./mesh_utils.js');
const { measureBoundingBox, measureVolume } = jscad.measurements;

const outDir = path.join(__dirname, 'stl');
fs.mkdirSync(outDir, { recursive: true });
const report = [];
for (const name of Object.keys(EVP.PARTS)) {
  const t0 = Date.now();
  const { geom, tris } = M.printMesh(jscad, EVP, name, params);
  const bb = measureBoundingBox(geom), size = bb[1].map((v, i) => +(v - bb[0][i]).toFixed(1));
  const vol = measureVolume(geom) / 1000;
  fs.writeFileSync(path.join(outDir, name + '.stl'), M.stlBytes(tris, name));
  const r = { part: name, fa: EVP.PARTS[name].fa, material: EVP.PARTS[name].mat, size_mm: size, volume_cm3: +vol.toFixed(1),
              grams_solid_PETG: +(vol * 1.27).toFixed(0), triangles: tris.length, open_edges: M.openEdges(tris), sec: (Date.now() - t0) / 1000 };
  report.push(r);
  console.log(JSON.stringify(r));
}
fs.writeFileSync(path.join(outDir, 'report.json'), JSON.stringify({ params: Object.assign(EVP.defaults(), params), parts: report }, null, 1));
