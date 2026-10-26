/* EVP-12 mesh utilities (Node + browser): triangulate, repair, binary STL, open-edge check. */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.EVPMesh = factory();
})(typeof self !== 'undefined' ? self : this, function () {
function triangles(geom3, g) {
  const tris = [];
  for (const poly of geom3.toPolygons(g)) {
    const v = poly.vertices;
    for (let i = 1; i < v.length - 1; i++) tris.push([v[0], v[i], v[i + 1]]);
  }
  return tris;
}


// Mesh repair: weld near-coincident vertices, then split triangles at T-junctions
// (vertices lying on another triangle's edge), drop degenerate triangles.
function repair(tris, tol = 1e-2, etol = 2e-3) {
  // 1. weld
  const cell = tol * 4, grid = new Map(), verts = [];
  const vid = (p) => {
    const cx = Math.floor(p[0] / cell), cy = Math.floor(p[1] / cell), cz = Math.floor(p[2] / cell);
    for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) for (let dz = -1; dz <= 1; dz++) {
      const l = grid.get((cx + dx) + ',' + (cy + dy) + ',' + (cz + dz));
      if (l) for (const i of l) { const q = verts[i]; if (Math.abs(q[0] - p[0]) <= tol && Math.abs(q[1] - p[1]) <= tol && Math.abs(q[2] - p[2]) <= tol) return i; }
    }
    const i = verts.length; verts.push([p[0], p[1], p[2]]);
    const k = cx + ',' + cy + ',' + cz; if (!grid.has(k)) grid.set(k, []); grid.get(k).push(i);
    return i;
  };
  const area2 = (a, b, c) => { const A = verts[a], B = verts[b], C = verts[c];
    const u = [B[0] - A[0], B[1] - A[1], B[2] - A[2]], v = [C[0] - A[0], C[1] - A[1], C[2] - A[2]];
    return Math.hypot(u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]); };
  let T = [];
  for (const t of tris) { const a = vid(t[0]), b = vid(t[1]), c = vid(t[2]); if (a !== b && b !== c && a !== c && area2(a, b, c) > 1e-6) T.push([a, b, c]); }
  // 2. T-junctions. Coarse grid over vertices for edge queries.
  const G = 12, vg = new Map();
  verts.forEach((p, i) => { const k = Math.floor(p[0] / G) + ',' + Math.floor(p[1] / G) + ',' + Math.floor(p[2] / G); if (!vg.has(k)) vg.set(k, []); vg.get(k).push(i); });
  const onEdge = (a, b) => {
    const A = verts[a], B = verts[b];
    const lo = [0, 1, 2].map(i => Math.floor((Math.min(A[i], B[i]) - tol) / G)), hi = [0, 1, 2].map(i => Math.floor((Math.max(A[i], B[i]) + tol) / G));
    const d = [B[0] - A[0], B[1] - A[1], B[2] - A[2]], L2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    let best = -1, bt = 2;
    for (let x = lo[0]; x <= hi[0]; x++) for (let y = lo[1]; y <= hi[1]; y++) for (let z = lo[2]; z <= hi[2]; z++) {
      const l = vg.get(x + ',' + y + ',' + z); if (!l) continue;
      for (const i of l) {
        if (i === a || i === b) continue;
        const P = verts[i], w = [P[0] - A[0], P[1] - A[1], P[2] - A[2]];
        const t = (w[0] * d[0] + w[1] * d[1] + w[2] * d[2]) / L2;
        if (t <= 1e-6 || t >= 1 - 1e-6) continue;
        const e = [w[0] - t * d[0], w[1] - t * d[1], w[2] - t * d[2]];
        if (e[0] * e[0] + e[1] * e[1] + e[2] * e[2] <= etol * etol && t < bt) { bt = t; best = i; }
      }
    }
    return best;
  };
  const out = []; const stack = T.slice(); let guard = 0;
  while (stack.length && guard++ < 5e6) {
    const [a, b, c] = stack.pop(); let split = false;
    for (const [p, q, r] of [[a, b, c], [b, c, a], [c, a, b]]) {
      const m = onEdge(p, q);
      if (m >= 0 && area2(p, m, r) > 1e-10 && area2(m, q, r) > 1e-10) { stack.push([p, m, r], [m, q, r]); split = true; break; }
    }
    if (!split) out.push([a, b, c]);
  }
  if (stack.length) { console.error('repair: split guard hit, keeping', stack.length); for (const t of stack) out.push(t); }
  // 3. close the tiny sliver holes CSG leaves behind (boundary loops -> triangle fans)
  const he = new Map();                                   // directed edge "a,b" -> count
  for (const [a, b, c] of out) for (const [p, q] of [[a, b], [b, c], [c, a]]) { const k = p + ',' + q; he.set(k, (he.get(k) || 0) + 1); }
  const nextOf = new Map();
  for (const [k, n] of he) { const [p, q] = k.split(',').map(Number); const back = he.get(q + ',' + p) || 0;
    for (let i = 0; i < n - back; i++) { if (!nextOf.has(p)) nextOf.set(p, []); nextOf.get(p).push(q); } }
  let filled = 0;
  for (const start of [...nextOf.keys()]) {
    while (nextOf.get(start) && nextOf.get(start).length) {
      const loop = [start]; let cur = nextOf.get(start).pop(), ok = false;
      for (let g = 0; g < 2000; g++) {
        if (cur === start) { ok = true; break; }
        loop.push(cur); const l = nextOf.get(cur); if (!l || !l.length) break; cur = l.pop();
      }
      if (!ok || loop.length < 3) continue;
      let per = 0; for (let i = 0; i < loop.length; i++) { const A = verts[loop[i]], B = verts[loop[(i + 1) % loop.length]]; per += Math.hypot(A[0] - B[0], A[1] - B[1], A[2] - B[2]); }
      for (let i = 1; i < loop.length - 1; i++) out.push([loop[0], loop[i + 1], loop[i]]);
      filled++;
    }
  }
  if (filled) console.error('repair: closed', filled, 'sliver loops');
  // 4. drop zero-area slivers
  return out.map(t => t.map(i => verts[i])).filter(([A, B, C]) => {
    const u = [B[0] - A[0], B[1] - A[1], B[2] - A[2]], v = [C[0] - A[0], C[1] - A[1], C[2] - A[2]];
    return Math.hypot(u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]) > 1e-9;
  });
}

function stlBytes(tris, name) {
  const buf = new ArrayBuffer(84 + tris.length * 50), dv = new DataView(buf), u8 = new Uint8Array(buf);
  const head = ('EVP-12 ' + name).padEnd(80, ' ').slice(0, 80);
  for (let i = 0; i < 80; i++) u8[i] = head.charCodeAt(i) & 0x7f;
  dv.setUint32(80, tris.length, true);
  let o = 84;
  for (const [a, b, c] of tris) {
    const ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2], vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx, l = Math.hypot(nx, ny, nz) || 1;
    for (const x of [nx / l, ny / l, nz / l, ...a, ...b, ...c]) { dv.setFloat32(o, x, true); o += 4; }
    dv.setUint16(o, 0, true); o += 2;
  }
  return u8;
}

// open edges: an edge used once (or by same-direction faces) = hole in the surface
function openEdges(tris) {
  const key = (p) => p.map(x => Math.round(x * 1e4) + 0).join(',');
  const m = new Map();
  for (const t of tris) for (let i = 0; i < 3; i++) {
    const a = key(t[i]), b = key(t[(i + 1) % 3]);
    const k = a < b ? a + '|' + b : b + '|' + a, s = a < b ? 1 : -1;
    m.set(k, (m.get(k) || 0) + s);
  }
  let bad = 0; for (const v of m.values()) if (v !== 0) bad++;
  return bad;
}


// Print-ready mesh of one part: build, orient, centre on the bed, triangulate, repair.
function printMesh(jscad, EVP, name, params) {
  const { translate } = jscad.transforms;
  let g = EVP.build(name, params);
  const orient = EVP.PRINT_ORIENT[name];
  if (orient) g = orient(g);
  const bb = jscad.measurements.measureBoundingBox(g);
  g = translate([-(bb[0][0] + bb[1][0]) / 2, -(bb[0][1] + bb[1][1]) / 2, -bb[0][2]], g);
  g = jscad.modifiers.generalize({ triangulate: true }, g);
  return { geom: g, tris: repair(triangles(jscad.geometries.geom3, g)) };
}
return { triangles, repair, stlBytes, openEdges, printMesh };
});
