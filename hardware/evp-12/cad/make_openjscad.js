// Bundles evp_parts.js into one file for https://openjscad.xyz (drag & drop; sliders appear).
const fs = require('fs');
const factory = fs.readFileSync(__dirname + '/evp_parts.js', 'utf8');
const out = `// EVP-12 parametric models — JSCAD v2 single-file version (drop on https://openjscad.xyz)
const jscad = require('@jscad/modeling')
const module_ = { exports: null }
;(function (module) {
${factory.replace(/^\(function \(root, factory\) \{[\s\S]*?\}\)\(typeof self[^,]*, /m, '(function (root, factory) { module.exports = factory })(null, ')}
})(module_)
const EVP = module_.exports(jscad)
const groups = { VTU: 'فرستنده خودرو', DH: 'هد گیرنده', CU: 'کنترلر کابینت' }
const getParameterDefinitions = () => [
  { name: 'show', type: 'choice', caption: 'نمایش', values: ['VTU', 'DH', 'CU'], captions: ['فرستنده خودرو', 'هد گیرنده', 'کنترلر کابینت'], initial: 'DH' },
  { name: 'explode', type: 'slider', caption: 'انفجار', min: 0, max: 1, step: 0.05, initial: 0 },
  ...EVP.PARAMS.map(([name, initial, min, max, step, , caption]) => ({ name, type: 'number', caption, initial, min, max, step }))
]
const main = (p) => {
  const pl = EVP.placements(Object.assign(EVP.defaults(), p))
  const out = []
  for (const [n, info] of Object.entries(EVP.PARTS)) {
    if (info.group !== p.show) continue
    let g = pl[n].at(EVP.build(n, p))
    g = jscad.transforms.translate(pl[n].ex.map(x => x * p.explode), g)
    out.push(jscad.colors.colorize(info.color, g))
  }
  return out
}
module.exports = { main, getParameterDefinitions }
`;
fs.writeFileSync(__dirname + '/evp_openjscad.js', out);
