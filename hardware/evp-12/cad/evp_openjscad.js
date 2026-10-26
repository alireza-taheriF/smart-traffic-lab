// EVP-12 parametric models — JSCAD v2 single-file version (drop on https://openjscad.xyz)
const jscad = require('@jscad/modeling')
const module_ = { exports: null }
;(function (module) {
/*
 * EVP-12 — parametric 3D models (JSCAD).  Units: millimetres.  Z is up.
 *
 * One source for everything:
 *   - Node:     node build_stl.js [params.json]   -> stl/*.stl
 *   - Browser:  EVP-12-3D.html (sliders, exploded view, STL export)
 *   - JSCAD:    drop evp_openjscad.js on https://openjscad.xyz for the built-in parameter UI
 *
 * Printable parts:
 *   VTU (vehicle transmitter, roof):  vtu_base, vtu_lid, vtu_bezel, vtu_led_plate
 *   DH  (detector head, one per approach): dh_body, dh_back, dh_gasket, dh_yoke, dh_saddle
 *   CU  (cabinet controller):          cu_base, cu_lid, cu_din
 */
(function (root, factory) { module.exports = factory })(null, function (jscad) {
  const { primitives, booleans, transforms, extrusions, hulls, colors } = jscad;
  const { cuboid, cylinder, roundedRectangle, rectangle, circle } = primitives;
  // polygon() wants counter-clockwise points: fix the winding whatever order they are written in
  const polygon = ({ points }) => {
    let a = 0; for (let i = 0; i < points.length; i++) { const p = points[i], q = points[(i + 1) % points.length]; a += p[0] * q[1] - q[0] * p[1]; }
    return primitives.polygon({ points: a < 0 ? points.slice().reverse() : points });
  };
  const { union, subtract } = booleans;
  const { translate, rotateX, rotateY, rotateZ } = transforms;
  const { extrudeLinear } = extrusions;
  const { hull } = hulls;
  const D = Math.PI / 180;

  // ------------------------------------------------------------------ parameters
  // [key, default, min, max, step, group, Persian label]
  const PARAMS = [
    ['seg', 48, 16, 96, 8, 'عمومی', 'تعداد قطعات دایره (کیفیت)'],
    ['clr', 0.3, 0.1, 0.6, 0.05, 'عمومی', 'لقی جفت‌شدن قطعات (mm)'],
    ['insert_m3', 4.0, 3.6, 4.4, 0.1, 'عمومی', 'سوراخ اینسرت حرارتی M3 (mm)'],

    ['vtu_L', 170, 150, 230, 1, 'فرستنده خودرو', 'طول بدنه'],
    ['vtu_W', 120, 100, 170, 1, 'فرستنده خودرو', 'عرض بدنه'],
    ['vtu_H', 56, 50, 80, 1, 'فرستنده خودرو', 'ارتفاع کفی'],
    ['vtu_lid_H', 16, 10, 30, 1, 'فرستنده خودرو', 'ارتفاع درب'],
    ['vtu_wall', 3, 2.4, 4, 0.2, 'فرستنده خودرو', 'ضخامت دیواره'],
    ['vtu_r', 8, 3, 15, 1, 'فرستنده خودرو', 'شعاع گوشه'],
    ['led_cols', 6, 4, 8, 1, 'فرستنده خودرو', 'تعداد ستون LED'],
    ['led_rows', 4, 2, 6, 1, 'فرستنده خودرو', 'تعداد ردیف LED'],
    ['led_pitch', 8.5, 7, 12, 0.5, 'فرستنده خودرو', 'فاصله LEDها'],
    ['led_hole', 5.2, 4.9, 5.6, 0.05, 'فرستنده خودرو', 'قطر سوراخ LED ۵ میلی‌متری'],
    ['mag_hole', 6.5, 5, 9, 0.5, 'فرستنده خودرو', 'سوراخ پیچ آهنربا (M6)'],
    ['gland_d', 12.7, 10, 20.5, 0.1, 'فرستنده خودرو', 'سوراخ گلند کابل (PG7=12.7)'],
    ['sma_d', 6.5, 6, 7, 0.1, 'فرستنده خودرو', 'سوراخ SMA آنتن LoRa'],
    ['visor_L', 22, 0, 40, 1, 'فرستنده خودرو', 'طول سایه‌بان پنجره'],

    ['dh_W', 64, 56, 90, 1, 'هد گیرنده', 'عرض بدنه هد'],
    ['dh_H', 50, 44, 80, 1, 'هد گیرنده', 'ارتفاع بدنه هد'],
    ['dh_L', 56, 45, 90, 1, 'هد گیرنده', 'طول بدنه هد'],
    ['hood_W', 50, 30, 80, 1, 'هد گیرنده', 'عرض داخلی کاور (زاویه افقی)'],
    ['hood_H', 34, 20, 60, 1, 'هد گیرنده', 'ارتفاع داخلی کاور (زاویه عمودی)'],
    ['hood_L', 120, 60, 200, 5, 'هد گیرنده', 'طول کاور'],
    ['hood_visor', 30, 0, 60, 1, 'هد گیرنده', 'سایه‌بان بالای کاور'],
    ['tsop_h', 6.5, 4, 9, 0.5, 'هد گیرنده', 'ارتفاع مرکز لنز TSOP از PCB'],
    ['pole_D', 114, 60, 220, 1, 'هد گیرنده', 'قطر تیر چراغ'],
    ['band_W', 13, 9, 20, 0.5, 'هد گیرنده', 'عرض تسمه‌ی بست (کلمپ)'],
    ['tilt_max', 20, 10, 30, 1, 'هد گیرنده', 'حداکثر تیلت ±'],
    ['pan_max', 45, 20, 60, 1, 'هد گیرنده', 'حداکثر چرخش افقی ±'],

    ['cu_L', 200, 160, 220, 1, 'کنترلر کابینت', 'طول جعبه'],
    ['cu_W', 130, 100, 180, 1, 'کنترلر کابینت', 'عرض جعبه'],
    ['cu_H', 50, 35, 80, 1, 'کنترلر کابینت', 'ارتفاع جعبه'],
    ['pcb_ax', 94, 40, 140, 1, 'کنترلر کابینت', 'فاصله سوراخ برد اصلی (X)'],
    ['pcb_ay', 64, 30, 110, 1, 'کنترلر کابینت', 'فاصله سوراخ برد اصلی (Y)'],
    ['rly_x', 69, 30, 140, 1, 'کنترلر کابینت', 'فاصله سوراخ برد رله (X)'],
    ['rly_y', 49, 20, 110, 1, 'کنترلر کابینت', 'فاصله سوراخ برد رله (Y)'],
  ];
  const defaults = () => Object.fromEntries(PARAMS.map(p => [p[0], p[1]]));

  // ------------------------------------------------------------------ helpers
  let SEG = 48;
  const box = (x0, y0, z0, x1, y1, z1) =>
    cuboid({ size: [x1 - x0, y1 - y0, z1 - z0], center: [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2] });
  // cylinder along an axis between two coordinates
  const cylZ = (r, x, y, z0, z1, s = SEG) => cylinder({ radius: r, height: z1 - z0, center: [x, y, (z0 + z1) / 2], segments: s });
  const cylX = (r, y, z, x0, x1, s = SEG) => rotateY(90 * D, cylinder({ radius: r, height: x1 - x0, center: [-z, y, (x0 + x1) / 2], segments: s }));
  const cylY = (r, x, z, y0, y1, s = SEG) => rotateX(-90 * D, cylinder({ radius: r, height: y1 - y0, center: [x, -z, (y0 + y1) / 2], segments: s }));
  // rounded rectangle prism (vertical edges rounded), centred in XY
  const rprism = (w, d, r, z0, z1, cx = 0, cy = 0) => {
    const rr = Math.max(0.01, Math.min(r, w / 2 - 0.01, d / 2 - 0.01));
    return translate([cx, cy, z0], extrudeLinear({ height: z1 - z0 }, roundedRectangle({ size: [w, d], roundRadius: rr, segments: SEG })));
  };
  // a 2D shape drawn in (x, z) extruded along y from y0 to y1
  const xzExtrude = (shape, y0, y1) => translate([0, y1, 0], rotateX(90 * D, extrudeLinear({ height: y1 - y0 }, shape)));
  // a 2D shape drawn in (y, z) extruded along x from x0 to x1
  const yzExtrude = (shape, x0, x1) => translate([x0, 0, 0], rotateZ(90 * D, rotateX(90 * D, extrudeLinear({ height: x1 - x0 }, shape))));
  // arc slot (2D) of width w at radius R between angles a0..a1 (deg), centred at (cx, cy)
  const arcSlot2 = (cx, cy, R, a0, a1, w) => {
    const parts = []; const n = Math.max(2, Math.ceil(Math.abs(a1 - a0) / 5));
    for (let i = 0; i < n; i++) {
      const t0 = (a0 + (a1 - a0) * i / n) * D, t1 = (a0 + (a1 - a0) * (i + 1) / n) * D;
      parts.push(hull(circle({ radius: w / 2, center: [cx + R * Math.cos(t0), cy + R * Math.sin(t0)], segments: 24 }),
                      circle({ radius: w / 2, center: [cx + R * Math.cos(t1), cy + R * Math.sin(t1)], segments: 24 })));
    }
    return union(...parts);
  };
  // rectangular ring prism (frame) centred at origin in XY
  const frame = (ow, od, iw, id, z0, z1) => subtract(box(-ow / 2, -od / 2, z0, ow / 2, od / 2, z1), box(-iw / 2, -id / 2, z0 - 1, iw / 2, id / 2, z1 + 1));

  // ================================================================== VTU — vehicle transmitter
  function vtuDims(P) {
    const L = P.vtu_L, W = P.vtu_W, H = P.vtu_H, t = P.vtu_wall, f = 3;
    const ledW = (P.led_cols - 1) * P.led_pitch + 5.5, ledH = (P.led_rows - 1) * P.led_pitch + 5.5;
    const winW = ledW + 8, winH = ledH + 3, winZ = f + (H - f) / 2 + 1;
    const pad = 7;                        // thick front wall around the window
    const pocket = 5;                     // acrylic border beyond the opening
    const plateX = L / 2 - pad - 6;       // front face of the LED plate
    return { L, W, H, t, f, ledW, ledH, winW, winH, winZ, pad, pocket, plateX };
  }

  function vtu_base(P) {
    const d = vtuDims(P); const { L, W, H, t, f, winW, winH, winZ, pad, pocket } = d;
    let g = subtract(rprism(L, W, P.vtu_r, 0, H), rprism(L - 2 * t, W - 2 * t, P.vtu_r - t, f, H + 1));
    const adds = [];
    // thick front pad around window
    const padY = winW / 2 + pocket + 9, padZ0 = f, padZ1 = Math.min(H - 3, winZ + winH / 2 + pocket + 3);
    adds.push(box(L / 2 - pad, -padY, padZ0, L / 2 - t + 0.5, padY, padZ1));
    // lid screw columns at inner corners
    const cx = L / 2 - t - 4.5, cy = W / 2 - t - 4.5;
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) adds.push(cylZ(5.5, sx * cx, sy * cy, f - 0.5, H));
    // LED plate bosses
    const bx = d.plateX - 3 - 8;
    for (const sy of [-1, 1]) adds.push(cylZ(4.5, bx, sy * 18, f - 0.5, f + 4));
    // main board bosses (perfboard 90x70, holes 84x64)
    const mx = -L / 2 + t + 8 + 45, pcx = 84 / 2, pcy = 64 / 2;
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) adds.push(cylZ(3.8, mx + sx * pcx, sy * pcy, f - 0.5, f + 6));
    // magnet stud bosses
    const gx = L / 2 - 26, gy = W / 2 - 24;
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) adds.push(cylZ(8, sx * gx, sy * gy, f - 0.5, f + 3));
    g = union(g, ...adds);

    const cuts = [];
    // window opening + acrylic pocket (3 mm sheet + silicone bead)
    cuts.push(box(L / 2 - pad - 1, -winW / 2, winZ - winH / 2, L / 2 + 1, winW / 2, winZ + winH / 2));
    cuts.push(box(L / 2 - 3.5, -winW / 2 - pocket, winZ - winH / 2 - pocket, L / 2 + 1, winW / 2 + pocket, winZ + winH / 2 + pocket));
    // bezel screw inserts (left/right of the window)
    for (const sy of [-1, 1]) for (const sz of [-1, 1])
      cuts.push(cylX(P.insert_m3 / 2, sy * (winW / 2 + pocket + 4.5), winZ + sz * 8, L / 2 - 8, L / 2 + 1));
    // lid inserts
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) cuts.push(cylZ(P.insert_m3 / 2, sx * cx, sy * cy, H - 9, H + 1));
    // LED plate bosses
    for (const sy of [-1, 1]) cuts.push(cylZ(P.insert_m3 / 2, bx, sy * 18, f, f + 5));
    // board bosses
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) cuts.push(cylZ(P.insert_m3 / 2, mx + sx * pcx, sy * pcy, f, f + 7));
    // magnet studs through the floor
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) cuts.push(cylZ(P.mag_hole / 2, sx * gx, sy * gy, -1, f + 4));
    // rear wall: cable gland, SMA bulkhead, pressure vent (M12)
    cuts.push(cylX(P.gland_d / 2, -W / 4, 26, -L / 2 - 1, -L / 2 + t + 1));
    cuts.push(cylX(P.sma_d / 2, W / 8, 30, -L / 2 - 1, -L / 2 + t + 1));
    cuts.push(cylX(6.1, W / 4 + 10, 30, -L / 2 - 1, -L / 2 + t + 1));
    // gasket groove on the rim (silicone cord 1.5 mm)
    const gl = L - t, gw = W - t, gr = P.vtu_r - t / 2;
    cuts.push(subtract(rprism(gl + 1.7, gw + 1.7, gr + 0.85, H - 1.6, H + 1), rprism(gl - 1.7, gw - 1.7, gr - 0.85, H - 2, H + 2)));
    return subtract(g, ...cuts);
  }

  function vtu_lid(P) {
    const { L, W, t } = vtuDims(P); const Hl = P.vtu_lid_H, top = 2.5;
    // modelled sitting on z = 0 (the base rim)
    let g = subtract(rprism(L, W, P.vtu_r, 0, Hl), rprism(L - 2 * t, W - 2 * t, P.vtu_r - t, -1, Hl - top));
    const cx = L / 2 - t - 4.5, cy = W / 2 - t - 4.5;
    const cols = [];
    // square corner pillars merged into the walls (screw passes through their centre)
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
      const x0 = sx * cx - 5.5, x1 = sx * cx + 5.5, y0 = sy * cy - 5.5, y1 = sy * cy + 5.5;
      cols.push(box(sx > 0 ? x0 : -L / 2 + t - 0.7, sy > 0 ? y0 : -W / 2 + t - 0.7, 2, sx > 0 ? L / 2 - t + 0.7 : x1, sy > 0 ? W / 2 - t + 0.7 : y1, Hl - 1));
    }
    // tongue into the base groove (compresses the silicone cord)
    const gl = L - t, gw = W - t, gr = P.vtu_r - t / 2;
    cols.push(subtract(rprism(gl + 1.1, gw + 1.1, gr + 0.55, -1.0, 0.2), rprism(gl - 1.1, gw - 1.1, gr - 0.55, -2, 1)));
    g = union(g, ...cols);
    const cuts = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
      cuts.push(cylZ(1.7, sx * cx, sy * cy, -2, Hl + 1));
      cuts.push(cylZ(3.3, sx * cx, sy * cy, Hl - 4, Hl + 1));         // counterbore M3 head
    }
    // GPS antenna mark (thin top over the patch antenna)
    cuts.push(cylZ(22, -L / 2 + 50, 0, Hl - 0.6, Hl + 1));
    return subtract(g, ...cuts);
  }

  function vtu_bezel(P) {
    // clamps the IR window; modelled in its assembled place on the base front face (x = L/2)
    const { L, winW, winH, winZ, pocket } = vtuDims(P);
    const ow = winW + 2 * pocket + 18, oh = winH + 2 * pocket + 4, T = 3;
    let g = box(L / 2, -ow / 2, winZ - oh / 2, L / 2 + T, ow / 2, winZ + oh / 2);
    if (P.visor_L > 0) {
      g = union(g, box(L / 2, -ow / 2, winZ + oh / 2 - T, L / 2 + T + P.visor_L, ow / 2, winZ + oh / 2));
      for (const sy of [-1, 1]) g = union(g, xzExtrude(polygon({ points: [[L / 2 + T, winZ + oh / 2 - T], [L / 2 + T + P.visor_L, winZ + oh / 2 - T], [L / 2 + T, winZ + oh / 2 - T - 10]] }),
                                                        sy * ow / 2 - (sy > 0 ? 2.5 : 0), sy * ow / 2 + (sy < 0 ? 2.5 : 0)));
    }
    const cuts = [box(L / 2 - 1, -winW / 2, winZ - winH / 2, L / 2 + T + 1, winW / 2, winZ + winH / 2)];
    for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
      const y = sy * (winW / 2 + pocket + 4.5), z = winZ + sz * 8;
      cuts.push(cylX(1.7, y, z, L / 2 - 1, L / 2 + T + 1));
      cuts.push(translate([0, 0, 0], cylX(3.0, y, z, L / 2 + T - 1.6, L / 2 + T + 0.1)));   // countersink-ish
    }
    return subtract(g, ...cuts);
  }

  function vtu_led_plate(P) {
    // L-shaped holder for the IR LED array; modelled in place
    const d = vtuDims(P); const { f, winZ, plateX } = d;
    const T = 3, pw = Math.max(60, d.ledW + 14), z0 = f + 4, z1 = winZ + d.ledH / 2 + 8;
    let g = union(box(plateX - T, -pw / 2, z0, plateX, pw / 2, z1),
                  box(plateX - T - 16, -26, z0, plateX, 26, z0 + T));
    const cuts = [];
    for (let i = 0; i < P.led_cols; i++) for (let j = 0; j < P.led_rows; j++) {
      const y = (i - (P.led_cols - 1) / 2) * P.led_pitch, z = winZ + (j - (P.led_rows - 1) / 2) * P.led_pitch;
      cuts.push(cylX(P.led_hole / 2, y, z, plateX - T - 1, plateX + 1, 24));
    }
    // adjustment slots onto the floor bosses (move the array 0..6 mm toward the window)
    const bx = plateX - 3 - 8;
    for (const sy of [-1, 1]) cuts.push(translate([0, 0, z0 - 1], extrudeLinear({ height: T + 2 }, hull(circle({ radius: 1.7, center: [bx - 3, sy * 18], segments: 20 }), circle({ radius: 1.7, center: [bx + 3, sy * 18], segments: 20 })))));
    // wire pass + zip-tie holes
    cuts.push(box(plateX - T - 1, -6, z0 + T, plateX + 1, 6, z0 + T + 5));
    return subtract(g, ...cuts);
  }

  // ================================================================== DH — detector head
  function dhDims(P) {
    const W = P.dh_W, H = P.dh_H, L = P.dh_L, t = 4, ht = 2.4;
    const winW = 34, winH = 18;            // bulkhead window (2 x TSOP side by side)
    const px = L * 0.6, pz = 6;            // tilt pivot
    return { W, H, L, t, ht, winW, winH, px, pz };
  }

  function dh_body(P) {
    const d = dhDims(P); const { W, H, L, t, ht, winW, winH, px, pz } = d;
    const hw = P.hood_W, hh = P.hood_H, hl = P.hood_L;
    // body box, open at the back (x = 0), bulkhead wall at the front
    let g = subtract(box(0, -W / 2, -H / 2, L, W / 2, H / 2), box(-1, -W / 2 + t, -H / 2 + t, L - 4, W / 2 - t, H / 2 - t));
    // hood tube + sun visor
    const ow = hw + 2 * ht, oh = hh + 2 * ht;
    let hood = subtract(box(L - 1, -ow / 2, -oh / 2, L + hl, ow / 2, oh / 2), box(L, -hw / 2, -hh / 2, L + hl + 1, hw / 2, hh / 2));
    if (P.hood_visor > 0) hood = union(hood, box(L + hl - 1, -ow / 2, oh / 2 - ht, L + hl + P.hood_visor, ow / 2, oh / 2));
    // baffles: knife rings that stop off-axis light without clipping the field of view
    const xd = L - 8, xm = L + hl, baffles = [];
    for (let k = 1; k <= 4; k++) {
      const x = L + hl * k / 5, s = (x - xd) / (xm - xd);
      const aw = Math.max(winW, hw * s) / 2 + 1.0, ah = Math.max(winH, hh * s) / 2 + 1.0;
      if (hw / 2 - aw < 1.2 && hh / 2 - ah < 1.2) continue;
      baffles.push(subtract(box(x - 0.6, -hw / 2 - 0.1, -hh / 2 - 0.1, x + 0.6, hw / 2 + 0.1, hh / 2 + 0.1),
                            box(x - 1, -Math.min(aw, hw / 2), -Math.min(ah, hh / 2), x + 1, Math.min(aw, hw / 2), Math.min(ah, hh / 2))));
    }
    // trunnions (M6 heat insert) + tilt-lock boss (M4 insert), both sides
    const adds = [hood, ...baffles];
    for (const sy of [-1, 1]) {
      adds.push(cylY(10, px, pz, sy > 0 ? W / 2 - 0.5 : -W / 2 - 6, sy > 0 ? W / 2 + 6 : -W / 2 + 0.5));
      adds.push(cylY(6, px - 25, pz, sy > 0 ? W / 2 - 0.5 : -W / 2 - 6, sy > 0 ? W / 2 + 6 : -W / 2 + 0.5));
    }
    // back cover screw columns
    const cy = W / 2 - t - 3.2, cz = H / 2 - t - 3.2;
    for (const sy of [-1, 1]) for (const sz of [-1, 1]) adds.push(cylX(4.2, sy * cy, sz * cz, 0, 14));
    g = union(g, ...adds);

    const cuts = [];
    // bulkhead window + inner pocket for a 2 mm IR-pass filter
    cuts.push(box(L - 5, -winW / 2, -winH / 2, L + 1, winW / 2, winH / 2));
    cuts.push(box(L - 4.5, -winW / 2 - 3, -winH / 2 - 3, L - 1.8, winW / 2 + 3, winH / 2 + 3));
    // PCB card guides (1.6 mm board, TSOP lens centre on the optical axis z = 0)
    const zp = -P.tsop_h;
    for (const sy of [-1, 1]) cuts.push(box(-1, sy > 0 ? W / 2 - t - 0.1 : -W / 2 + t - 1.6, zp - 1.6 - 0.3, L - 6, sy > 0 ? W / 2 - t + 1.6 : -W / 2 + t + 0.1, zp + 0.3));
    // inserts
    for (const sy of [-1, 1]) {
      cuts.push(cylY(4.0, px, pz, sy > 0 ? W / 2 - 4 : -W / 2 - 7, sy > 0 ? W / 2 + 7 : -W / 2 + 4));   // M6 insert Ø8
      cuts.push(cylY(2.8, px - 25, pz, sy > 0 ? W / 2 - 2 : -W / 2 - 7, sy > 0 ? W / 2 + 7 : -W / 2 + 2)); // M4 insert Ø5.6
    }
    for (const sy of [-1, 1]) for (const sz of [-1, 1]) cuts.push(cylX(P.insert_m3 / 2, sy * cy, sz * cz, -1, 9));
    // drain hole in the hood floor behind the first baffle
    cuts.push(cylZ(2, L + 6, 0, -oh / 2 - 1, -hh / 2 + 1, 20));
    return subtract(g, ...cuts);
  }

  function dh_back(P) {
    const { W, H, t } = dhDims(P); const T = 4, G = 1.5;      // plate, gasket thickness
    const cy = W / 2 - t - 3.2, cz = H / 2 - t - 3.2;
    // plate behind the gasket (x < -G) + locating lip that enters the body opening
    let g = union(box(-T - G, -W / 2, -H / 2, -G, W / 2, H / 2),
                  subtract(box(-G - 1, -W / 2 + t + P.clr, -H / 2 + t + P.clr, 2.5, W / 2 - t - P.clr, H / 2 - t - P.clr),
                           box(-G - 1, -W / 2 + t + 2, -H / 2 + t + 2, 3, W / 2 - t - 2, H / 2 - t - 2)));
    const cuts = [];
    for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
      cuts.push(cylX(1.7, sy * cy, sz * cz, -T - G - 1, 3));
      cuts.push(cylX(3.2, sy * cy, sz * cz, -T - G - 1, -T - G + 2));
      cuts.push(cylX(4.8, sy * cy, sz * cz, -G - 0.2, 3));       // clear the screw columns in the lip
    }
    cuts.push(cylX(P.gland_d / 2, 0, 8, -T - G - 1, 3));         // PG7 gland, above the PCB
    cuts.push(cylX(1.55, -W / 2 + t + 7, 8, -T - G - 1, 3));    // 3 mm alignment LED (blinks on every frame)
    return subtract(g, ...cuts);
  }

  function dh_gasket(P) {
    // flat TPU frame between the body's back face and the cover
    const { W, H, t } = dhDims(P);
    return subtract(box(-1.5, -W / 2, -H / 2, 0, W / 2, H / 2), box(-2, -W / 2 + t - 0.3, -H / 2 + t - 0.3, 1, W / 2 - t + 0.3, H / 2 - t + 0.3));
  }

  function yokeDims(P) {
    const d = dhDims(P);
    const gap = d.W + 2 * 6 + 2 * 1.0;       // between the arms: body + trunnions + washers
    const armT = 6, hp = 45, baseT = 6;
    return { ...d, gap, armT, hp, baseT };
  }

  function dh_yoke(P) {
    // pivot axis along y at (0, 0, hp); pan axis = z through origin
    const { gap, armT, hp, baseT } = yokeDims(P);
    const bw = gap + 2 * armT, R = 25;
    let base = box(-34, -bw / 2, 0, 34, bw / 2, baseT);
    const arm2 = hull(circle({ radius: 15, center: [0, hp], segments: SEG }), circle({ radius: 9, center: [-R, hp], segments: SEG }),
                      rectangle({ size: [50, 2], center: [-9, baseT + 1] }));
    const arms = [xzExtrude(arm2, gap / 2, gap / 2 + armT), xzExtrude(arm2, -gap / 2 - armT, -gap / 2)];
    // gussets
    const gus = polygon({ points: [[-30, baseT], [10, baseT], [-10, baseT + 18]] });
    arms.push(xzExtrude(gus, gap / 2 + armT - 0.5, gap / 2 + armT + 3), xzExtrude(gus, -gap / 2 - armT - 3, -gap / 2 - armT + 0.5));
    let g = union(base, ...arms);
    const cuts = [cylY(3.25, 0, hp, -bw, bw)];
    const slot = arcSlot2(0, hp, R, 180 - P.tilt_max, 180 + P.tilt_max, 4.5);
    cuts.push(xzExtrude(slot, -bw, bw));
    // pan: centre M8 + two arc slots for M5
    cuts.push(cylZ(4.25, 0, 0, -1, baseT + 1));
    for (const a of [0, 180]) cuts.push(translate([0, 0, -1], extrudeLinear({ height: baseT + 2 }, arcSlot2(0, 0, R, a - P.pan_max, a + P.pan_max, 5.5))));
    return subtract(g, ...cuts);
  }

  const SADDLE_PAN_X = 60;      // pan axis = SADDLE_PAN_X + 9 from the pole contact
  function dh_saddle(P) {
    // the pole is vertical; its surface touches the saddle at x ≈ 0. Shelf top at z = 0.
    const R = P.pole_D / 2, bw = P.band_W + 1, halfY = 35, panX = SADDLE_PAN_X, shelfX = panX + 22;
    let g = union(box(0, -halfY, -88, 20, halfY, 0), box(0, -halfY, -8, shelfX + 22, halfY, 0));
    const gus = polygon({ points: [[19, -8], [shelfX + 18, -8], [19, -62]] });
    for (const sy of [-1, 1]) g = union(g, xzExtrude(gus, sy * (halfY - 3) - 3, sy * (halfY - 3) + 3));
    const cuts = [];
    cuts.push(cylZ(R, -R + 4, 0, -100, 10, 96));          // concave seat (self-centres on a range of poles)
    for (const zc of [-16, -72]) cuts.push(box(6, -halfY - 1, zc - bw / 2, 9.8, halfY + 1, zc + bw / 2));  // band tunnels
    cuts.push(cylZ(5.0, panX + 9, 0, -9, 1));            // M8 insert Ø10
    for (const sx of [-1, 1]) cuts.push(cylZ(3.2, panX + 9 + sx * 25, 0, -9, 1));   // M5 insert Ø6.4
    return subtract(g, ...cuts);
  }

  // ================================================================== CU — cabinet controller
  function cuDims(P) { return { L: P.cu_L, W: P.cu_W, H: P.cu_H, t: 2.5, f: 3, r: 6 }; }

  function cu_base(P) {
    const { L, W, H, t, f, r } = cuDims(P);
    let g = subtract(rprism(L, W, r, 0, H), rprism(L - 2 * t, W - 2 * t, r - t, f, H + 1));
    const ax = -L / 2 + t + 6 + P.pcb_ax / 2 + 3, ay = 0;
    const rx = L / 2 - t - 6 - P.rly_x / 2 - 3, ry = 0;
    const cx = L / 2 - t - 4, cy = W / 2 - t - 4;
    const adds = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
      adds.push(cylZ(3.8, ax + sx * P.pcb_ax / 2, ay + sy * P.pcb_ay / 2, f - 0.5, f + 6));
      adds.push(cylZ(3.8, rx + sx * P.rly_x / 2, ry + sy * P.rly_y / 2, f - 0.5, f + 6));
      adds.push(cylZ(4.5, sx * cx, sy * cy, f - 0.5, H));
    }
    for (const sx of [-1, 1]) adds.push(cylZ(5, sx * 20, 0, f - 0.5, f + 3));      // DIN plate inserts
    g = union(g, ...adds);
    const cuts = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
      cuts.push(cylZ(P.insert_m3 / 2, ax + sx * P.pcb_ax / 2, ay + sy * P.pcb_ay / 2, f, f + 7));
      cuts.push(cylZ(P.insert_m3 / 2, rx + sx * P.rly_x / 2, ry + sy * P.rly_y / 2, f, f + 7));
      cuts.push(cylZ(P.insert_m3 / 2, sx * cx, sy * cy, H - 8, H + 1));
    }
    for (const sx of [-1, 1]) cuts.push(cylZ(P.insert_m3 / 2, sx * 20, 0, -1, f + 4));
    // field wiring slot (pluggable 5.08 mm terminals) on the front wall, cable-tie bars kept
    const sl = L - 2 * r - 20, sz0 = f + 9, sz1 = Math.min(H - 6, sz0 + 18);
    for (let i = 0; i < 3; i++) {
      const w = sl / 3 - 4, x0 = -sl / 2 + i * (sl / 3) + 2;
      cuts.push(box(x0, -W / 2 - 1, sz0, x0 + w, -W / 2 + t + 1, sz1));
    }
    // rear wall: LoRa SMA, USB service opening, RS-485 gland
    cuts.push(cylY(P.sma_d / 2, L / 2 - 30, H - 14, W / 2 - t - 1, W / 2 + 1));
    cuts.push(box(ax - 8, W / 2 - t - 1, f + 8, ax + 8, W / 2 + 1, f + 16));
    cuts.push(cylY(P.gland_d / 2, -L / 2 + 30, H / 2, W / 2 - t - 1, W / 2 + 1));
    // vent slots on both short walls
    for (const sx of [-1, 1]) for (let k = 0; k < 5; k++)
      cuts.push(box(sx > 0 ? L / 2 - t - 1 : -L / 2 - 1, -24 + k * 12 - 2, H - 22, sx > 0 ? L / 2 + 1 : -L / 2 + t + 1, -24 + k * 12 + 2, H - 8));
    return subtract(g, ...cuts);
  }

  function cu_lid(P) {
    const { L, W, t, r } = cuDims(P); const T = 2.5;
    let g = union(rprism(L, W, r, 0, T),
                  subtract(rprism(L - 2 * t - 2 * P.clr, W - 2 * t - 2 * P.clr, r - t, -3, 0.1), rprism(L - 2 * t - 4, W - 2 * t - 4, r - t - 2, -4, 1)));
    const cx = L / 2 - t - 4, cy = W / 2 - t - 4;
    const cuts = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
      cuts.push(cylZ(1.7, sx * cx, sy * cy, -5, T + 1));
      cuts.push(cylZ(3.2, sx * cx, sy * cy, T - 1.5, T + 1));
      cuts.push(cylZ(5.2, sx * cx, sy * cy, -5, 0));          // clear the corner columns
    }
    // status light pipes: PWR, LoRa, N, E, S, W, HEALTHY
    for (let i = 0; i < 7; i++) cuts.push(cylZ(1.6, -L / 2 + 30 + i * 12, W / 2 - 20, -1, T + 1, 20));
    // label recess
    cuts.push(box(-35, -W / 2 + 18, T - 0.6, 55, -W / 2 + 48, T + 1));
    return subtract(g, ...cuts);
  }

  function cu_din(P) {
    // DIN-rail (35 mm top-hat) clip plate, screwed under cu_base. Rail axis = x. Plate top at z = 0.
    const T = 4, len = 60;
    const prof = union(
      rectangle({ size: [56, T], center: [0, -T / 2] }),
      // fixed hook (y < 0)
      polygon({ points: [[-21.5, -T], [-18, -T], [-18, -T - 1.3], [-16.2, -T - 1.3], [-16.2, -T - 2.8], [-21.5, -T - 2.8]] }),
      // spring latch (y > 0) with lead-in, on a thin flexible lip
      polygon({ points: [[18, -T], [20, -T], [20, -T - 4.2], [16.0, -T - 4.2], [16.0, -T - 2.9], [18, -T - 1.3]] }),
    );
    let g = yzExtrude(prof, -len / 2, len / 2);
    const cuts = [];
    // relief slot so the latch side can flex
    cuts.push(box(-20, 13, -T - 1, 20, 14.6, 1));
    for (const sx of [-1, 1]) {
      cuts.push(cylZ(1.7, sx * 20, 0, -T - 1, 1));
      cuts.push(cylZ(3.2, sx * 20, 0, -T - 1, -T + 1.8));
    }
    // screwdriver release tab hole
    cuts.push(box(-4, 22, -T - 1, 4, 26, 1));
    return subtract(g, ...cuts);
  }

  // ================================================================== registry + assembly
  const PARTS = {
    vtu_base:      { fn: vtu_base, group: 'VTU', fa: 'بدنه فرستنده (کفی)', mat: 'ASA / PETG', color: [0.20, 0.22, 0.25] },
    vtu_lid:       { fn: vtu_lid, group: 'VTU', fa: 'درب فرستنده', mat: 'ASA / PETG', color: [0.85, 0.20, 0.18] },
    vtu_bezel:     { fn: vtu_bezel, group: 'VTU', fa: 'قاب پنجره IR + سایه‌بان', mat: 'ASA / PETG', color: [0.15, 0.15, 0.17] },
    vtu_led_plate: { fn: vtu_led_plate, group: 'VTU', fa: 'نگهدارنده آرایه LED', mat: 'PETG', color: [0.95, 0.75, 0.20] },
    dh_body:       { fn: dh_body, group: 'DH', fa: 'بدنه هد گیرنده + کاور', mat: 'ASA (مشکی مات)', color: [0.16, 0.17, 0.19] },
    dh_back:       { fn: dh_back, group: 'DH', fa: 'درب پشت هد', mat: 'ASA', color: [0.30, 0.32, 0.36] },
    dh_gasket:     { fn: dh_gasket, group: 'DH', fa: 'واشر درب پشت', mat: 'TPU 95A', color: [0.10, 0.45, 0.80] },
    dh_yoke:       { fn: dh_yoke, group: 'DH', fa: 'یوک تیلت/پن', mat: 'PETG / ASA', color: [0.95, 0.65, 0.10] },
    dh_saddle:     { fn: dh_saddle, group: 'DH', fa: 'زین بست تیر', mat: 'ASA / PETG', color: [0.55, 0.57, 0.60] },
    cu_base:       { fn: cu_base, group: 'CU', fa: 'جعبه کنترلر', mat: 'PETG', color: [0.85, 0.86, 0.88] },
    cu_lid:        { fn: cu_lid, group: 'CU', fa: 'درب جعبه کنترلر', mat: 'PETG', color: [0.20, 0.45, 0.75] },
    cu_din:        { fn: cu_din, group: 'CU', fa: 'گیره ریل DIN', mat: 'PETG', color: [0.25, 0.25, 0.28] },
  };

  // Placement of each part in the assembly (plus an explode vector).
  function placements(P) {
    const v = vtuDims(P), y = yokeDims(P), c = cuDims(P);
    const panX = SADDLE_PAN_X + 9;   // saddle pan centre
    const pivotZ = y.hp + 0;         // yoke pivot height above saddle shelf
    const dhOffset = (g) => translate([panX - y.px, 0, pivotZ - y.pz], g);
    return {
      vtu_base:      { at: g => g, ex: [0, 0, 0] },
      vtu_lid:       { at: g => translate([0, 0, v.H], g), ex: [0, 0, 45] },
      vtu_bezel:     { at: g => g, ex: [40, 0, 0] },
      vtu_led_plate: { at: g => g, ex: [0, 0, 70] },
      dh_saddle:     { at: g => g, ex: [-25, 0, 0] },
      dh_yoke:       { at: g => translate([panX, 0, 0], g), ex: [0, 0, 25] },
      dh_body:       { at: dhOffset, ex: [0, 0, 60] },
      dh_back:       { at: dhOffset, ex: [-40, 0, 60] },
      dh_gasket:     { at: dhOffset, ex: [-20, 0, 60] },
      cu_base:       { at: g => g, ex: [0, 0, 0] },
      cu_lid:        { at: g => translate([0, 0, c.H], g), ex: [0, 0, 45] },
      cu_din:        { at: g => g, ex: [0, 0, -30] },
    };
  }

  // Non-printed reference geometry for the viewer (LEDs, PCBs, pole, acrylic, magnets, TSOPs).
  function references(P) {
    const v = vtuDims(P), d = dhDims(P), y = yokeDims(P), c = cuDims(P);
    const panX = SADDLE_PAN_X + 9, dx = panX - y.px, dz = y.hp - y.pz;
    const refs = [];
    // VTU: LEDs, acrylic window, board, magnets
    const leds = [];
    for (let i = 0; i < P.led_cols; i++) for (let j = 0; j < P.led_rows; j++) {
      const yy = (i - (P.led_cols - 1) / 2) * P.led_pitch, zz = v.winZ + (j - (P.led_rows - 1) / 2) * P.led_pitch;
      leds.push(cylX(2.5, yy, zz, v.plateX - 1, v.plateX + 7, 12));
    }
    refs.push({ group: 'VTU', fa: 'LED مادون‌قرمز ×' + P.led_cols * P.led_rows, color: [0.55, 0.05, 0.08], g: union(...leds), ex: [0, 0, 70] });
    refs.push({ group: 'VTU', fa: 'پنجره اکریلیک IR-pass ۳ میلی‌متر', color: [0.25, 0.02, 0.05, 0.6],
                g: box(v.L / 2 - 3.3, -v.winW / 2 - v.pocket + 0.3, v.winZ - v.winH / 2 - v.pocket + 0.3, v.L / 2 - 0.3, v.winW / 2 + v.pocket - 0.3, v.winZ + v.winH / 2 + v.pocket - 0.3), ex: [22, 0, 0] });
    refs.push({ group: 'VTU', fa: 'برد اصلی (ESP32 + LoRa + درایور)', color: [0.10, 0.50, 0.25], g: box(-v.L / 2 + v.t + 8, -35, v.f + 6, -v.L / 2 + v.t + 98, 35, v.f + 7.6), ex: [0, 0, 70] });
    refs.push({ group: 'VTU', fa: 'ماژول GPS', color: [0.75, 0.70, 0.55], g: box(-v.L / 2 + 38, -12, v.H + P.vtu_lid_H - 9, -v.L / 2 + 62, 12, v.H + P.vtu_lid_H - 3), ex: [0, 0, 45] });
    const mags = [];
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) mags.push(cylZ(16, sx * (v.L / 2 - 26), sy * (v.W / 2 - 24), -8, 0, 32));
    refs.push({ group: 'VTU', fa: 'آهنربای گلدانی Ø32 با رزوه M6', color: [0.45, 0.45, 0.48], g: union(...mags), ex: [0, 0, -30] });
    // DH: pole, PCB, TSOPs, filter
    refs.push({ group: 'DH', fa: 'تیر چراغ (مرجع)', color: [0.62, 0.64, 0.66], g: cylZ(P.pole_D / 2, -P.pole_D / 2 + 4, 0, -200, 160, 64), ex: [-25, 0, 0] });
    const zp = -P.tsop_h;
    const pcb = translate([dx, 0, dz], box(4, -d.W / 2 + d.t - 1.3, zp - 1.6, d.L - 7, d.W / 2 - d.t + 1.3, zp));
    const tsops = [];
    for (const sy of [-1, 1]) tsops.push(translate([dx, 0, dz], box(d.L - 12, sy * 7 - 3, zp, d.L - 7.5, sy * 7 + 3, zp + 11)));
    refs.push({ group: 'DH', fa: 'برد گیرنده + ۲ عدد TSOP38238', color: [0.10, 0.50, 0.25], g: union(pcb, ...tsops), ex: [0, 0, 60] });
    refs.push({ group: 'DH', fa: 'فیلتر IR-pass ۲ میلی‌متر', color: [0.25, 0.02, 0.05, 0.6], g: translate([dx, 0, dz], box(d.L - 4.3, -d.winW / 2 - 2.7, -d.winH / 2 - 2.7, d.L - 2.3, d.winW / 2 + 2.7, d.winH / 2 + 2.7)), ex: [0, 0, 60] });
    // CU: boards
    const ax = -c.L / 2 + c.t + 6 + P.pcb_ax / 2 + 3, rx = c.L / 2 - c.t - 6 - P.rly_x / 2 - 3;
    refs.push({ group: 'CU', fa: 'برد اصلی کنترلر', color: [0.10, 0.50, 0.25], g: box(ax - P.pcb_ax / 2 - 3, -P.pcb_ay / 2 - 3, c.f + 6, ax + P.pcb_ax / 2 + 3, P.pcb_ay / 2 + 3, c.f + 7.6), ex: [0, 0, 22] });
    refs.push({ group: 'CU', fa: 'ماژول رله ۴ کانال', color: [0.15, 0.30, 0.65], g: union(box(rx - P.rly_x / 2 - 3, -P.rly_y / 2 - 3, c.f + 6, rx + P.rly_x / 2 + 3, P.rly_y / 2 + 3, c.f + 7.6), box(rx - P.rly_x / 2 + 2, -P.rly_y / 2 + 2, c.f + 7.6, rx + P.rly_x / 2 - 2, -P.rly_y / 2 + 17, c.f + 23)), ex: [0, 0, 22] });
    refs.push({ group: 'CU', fa: 'ریل DIN ۳۵ میلی‌متر (مرجع)', color: [0.70, 0.72, 0.75], g: union(box(-110, -17.5, -5, 110, 17.5, -4), box(-110, -13.5, -11.5, 110, 13.5, -5)), ex: [0, 0, -30] });
    return refs;
  }

  function build(name, params) {
    const P = Object.assign(defaults(), params || {});
    SEG = Math.round(P.seg);
    return PARTS[name].fn(P);
  }

  // Print orientation (rotation applied before STL export so the part lies flat, best side down)
  const PRINT_ORIENT = {
    vtu_lid: g => rotateX(180 * D, g),
    vtu_bezel: g => rotateY(-90 * D, g),
    vtu_led_plate: g => rotateY(90 * D, g),   // LED face down, foot up
    dh_body: g => rotateY(-90 * D, g),        // stand on the open back, hood up
    dh_back: g => rotateY(-90 * D, g),        // plate on the bed, lip up
    dh_gasket: g => rotateY(90 * D, g),
    dh_yoke: g => g,
    dh_saddle: g => rotateX(180 * D, g),      // shelf top on the bed: walls and gussets vertical, tunnels bridge 4 mm
    cu_lid: g => rotateX(180 * D, g),
    cu_din: g => rotateX(180 * D, g),
  };

  return { PARAMS, PARTS, defaults, build, placements, references, PRINT_ORIENT, dims: { vtuDims, dhDims, yokeDims, cuDims } };
});

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
