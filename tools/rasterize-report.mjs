// Optional report export using the already-bundled image runtime, no download.
import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
const directory = path.resolve(process.argv[2] || 'results/static_20261104_2010/figures');
const workspace = path.resolve(path.dirname(new URL(import.meta.url).pathname.replace(/^\/([A-Za-z]:)/, '$1')), '..');
const cache = path.join(workspace, 'artifacts', 'font-cache');
fs.mkdirSync(cache, { recursive: true });
const fontConfig = path.join(cache, 'fonts.conf');
const escape = s => s.replace(/&/g, '&amp;').replace(/</g, '&lt;');
fs.writeFileSync(fontConfig, `<?xml version="1.0"?><!DOCTYPE fontconfig SYSTEM "urn:fontconfig:fonts.dtd"><fontconfig><dir>${escape(path.join(process.env.WINDIR || 'C:/Windows', 'Fonts').replaceAll('\\','/'))}</dir><cachedir>${escape(cache.replaceAll('\\','/'))}</cachedir></fontconfig>`);
process.env.FONTCONFIG_FILE = fontConfig;
let sharp;
try { sharp = require('sharp'); }
catch { sharp = require(path.join(os.homedir(), '.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/sharp')); }
for (const name of fs.readdirSync(directory).filter(n => n.endsWith('.svg'))) {
  const output = path.join(directory, name.replace(/\.svg$/, '.png'));
  await sharp(path.join(directory, name), { density: 144 }).png().toFile(output);
  console.log(output);
}
