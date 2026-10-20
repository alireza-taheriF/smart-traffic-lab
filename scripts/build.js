#!/usr/bin/env node
/* Inlines src/traffic-core.js into web/lab.template.html -> docs/index.html (single self-contained page, served by GitHub Pages). */
const fs = require('fs'), path = require('path');
const root = path.join(__dirname, '..');
const core = fs.readFileSync(path.join(root, 'src', 'traffic-core.js'), 'utf8');
if (core.includes('</script')) throw new Error('core must not contain </script');
const tpl = fs.readFileSync(path.join(root, 'web', 'lab.template.html'), 'utf8');
const html = '<!doctype html>\n<html lang="fa" dir="rtl"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">' +
  '<style>:root{color-scheme:light}html{padding-top:env(safe-area-inset-top,0px)}body{margin:0}[hidden]{display:none!important}</style></head><body>\n' +
  tpl.replace('/*CORE*/', core) + '\n</body></html>\n';
fs.writeFileSync(path.join(root, 'docs', 'index.html'), html);
console.log('docs/index.html', (html.length / 1024).toFixed(0) + ' KB');
