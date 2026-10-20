#!/usr/bin/env node
/* Captures README screenshots + a demo video with Playwright.  Usage: NODE_PATH=<playwright> node scripts/capture.js */
const path = require('path'), fs = require('fs');
const { chromium } = require('playwright');
const root = path.join(__dirname, '..'), A = path.join(root, 'docs', 'assets'), url = 'file://' + path.join(root, 'docs', 'index.html');
const click = (p, sel) => p.evaluate(s => document.querySelector(s).click(), sel);
const scrollTo = (p, y) => p.evaluate(y => window.scrollTo({ top: y, behavior: 'instant' }), y);
const top = (p, sel) => p.evaluate(s => document.querySelector(s).getBoundingClientRect().top + window.scrollY, sel);
(async () => {
  const exe = fs.existsSync('/opt/pw-browsers/chromium') ? '/opt/pw-browsers/chromium' : undefined;
  const b = await chromium.launch({ executablePath: exe });
  // ---------- screenshots ----------
  const p = await b.newPage({ viewport: { width: 1360, height: 900 }, deviceScaleFactor: 1.5 });
  await p.goto(url); await p.waitForTimeout(800);
  await p.evaluate(() => { document.querySelector('#speed button:nth-child(3)').click(); });   // x4
  await p.waitForTimeout(24000);
  await p.screenshot({ path: path.join(A, 'controls.png'), clip: { x: 0, y: 0, width: 1360, height: 700 } });
  await click(p, '[data-ev=amb]'); await click(p, '[data-ev=fire]');
  await p.waitForFunction(() => window.__lab.world.smart.net.nodes[0].pre, null, { timeout: 30000 }).catch(() => {});
  await p.waitForTimeout(1500);
  await scrollTo(p, (await top(p, '#banner')) - 10);
  await p.screenshot({ path: path.join(A, 'hero.png') });
  await (await p.$('#panels')).screenshot({ path: path.join(A, 'side-by-side.png') });
  await p.waitForTimeout(12000);
  await scrollTo(p, (await top(p, '#mt')) - 90); await p.waitForTimeout(400);
  await p.screenshot({ path: path.join(A, 'metrics.png') });
  await scrollTo(p, (await top(p, '#mode')) - 70); await p.waitForTimeout(400);
  await p.screenshot({ path: path.join(A, 'decisions.png') });
  // other layouts
  for (const k of ['tee', 'corridor', 'grid']) {
    await p.selectOption('#scn', k); await p.waitForTimeout(6000);
    await scrollTo(p, (await top(p, '#panels')) - 10);
    await (await p.$('#panels')).screenshot({ path: path.join(A, 'layout-' + k + '.png') });
  }
  await p.close();
  const m = await b.newPage({ viewport: { width: 400, height: 860 }, deviceScaleFactor: 2 });
  await m.goto(url); await m.waitForTimeout(6000);
  await scrollTo(m, (await top(m, '#banner')) - 8); await m.screenshot({ path: path.join(A, 'mobile.png') });
  await m.close();
  // ---------- video ----------
  const ctx = await b.newContext({ viewport: { width: 1280, height: 720 }, recordVideo: { dir: path.join(root, 'results', 'video'), size: { width: 1280, height: 720 } } });
  const v = await ctx.newPage(); await v.goto(url); await v.waitForTimeout(2500);
  await v.evaluate(() => { document.querySelector('#speed button:nth-child(3)').click(); });
  await scrollTo(v, (await top(v, '#banner')) + 40); await v.waitForTimeout(11000);
  await click(v, '[data-ev=amb]'); await v.waitForTimeout(9000);
  await click(v, '[data-ev=fire]'); await click(v, '[data-ev=police]'); await v.waitForTimeout(9000);
  await click(v, '#surge'); await v.waitForTimeout(8000);
  await scrollTo(v, (await top(v, '#mt')) - 70); await v.waitForTimeout(5000);
  await scrollTo(v, (await top(v, '#mode')) - 60); await v.waitForTimeout(5000);
  await v.selectOption('#scn', 'corridor'); await scrollTo(v, (await top(v, '#panels')) - 10); await v.waitForTimeout(9000);
  await ctx.close(); await b.close();
  console.log('done');
})();
