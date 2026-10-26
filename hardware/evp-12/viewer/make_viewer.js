// Inlines three.js, OrbitControls, JSCAD modeling, evp_parts.js and mesh_utils.js into one offline HTML file.
const fs = require('fs'); const path = require('path');
const libs = {
  THREE: process.env.THREE_JS || '/home/claude/three/build/three.min.js',
  ORBIT: process.env.ORBIT_JS || '/home/claude/three/examples/js/controls/OrbitControls.js',
  JSCAD: process.env.JSCAD_JS || '/home/claude/jscad/packages/modeling/dist/jscad-modeling.min.js',
  PARTS: path.join(__dirname, '../cad/evp_parts.js'),
  MESH: path.join(__dirname, '../cad/mesh_utils.js'),
  INFO: path.join(__dirname, 'part_info.js'),
  SIM: path.join(__dirname, 'sim.js'),
};
let html = fs.readFileSync(path.join(__dirname, 'viewer_src.html'), 'utf8');
for (const [k, f] of Object.entries(libs)) {
  const src = fs.readFileSync(f, 'utf8').replace(/<\/script/gi, '<\\/script');
  html = html.replace('/*@@' + k + '@@*/', () => src);
}
fs.writeFileSync(path.join(__dirname, 'EVP-12-3D.html'), html);
console.log('EVP-12-3D.html', (html.length / 1024).toFixed(0), 'KB');
