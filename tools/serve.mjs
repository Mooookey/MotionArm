import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const port = Number(process.argv[2] || 8000);
const types = { '.html':'text/html; charset=utf-8', '.js':'text/javascript; charset=utf-8', '.css':'text/css; charset=utf-8', '.json':'application/json; charset=utf-8', '.svg':'image/svg+xml', '.png':'image/png', '.csv':'text/csv; charset=utf-8', '.md':'text/plain; charset=utf-8', '.bin':'application/octet-stream' };
const server = http.createServer((req, res) => {
  if (!['GET','HEAD'].includes(req.method)) { res.writeHead(405); res.end(); return; }
  try {
    const pathname = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
    const results = pathname.startsWith('/results/');
    const base = path.join(root, results ? 'results' : 'web');
    const relative = results ? pathname.slice('/results'.length) : pathname;
    let target = path.resolve(base, '.' + relative);
    if (target !== base && !target.startsWith(base + path.sep)) { res.writeHead(403); res.end(); return; }
    if (fs.statSync(target).isDirectory()) target = path.join(target, 'index.html');
    const stat = fs.statSync(target);
    res.writeHead(200, { 'Content-Type': types[path.extname(target)] || 'application/octet-stream', 'Content-Length':stat.size, 'Cache-Control':'no-store' });
    if (req.method === 'HEAD') res.end();
    else fs.createReadStream(target).on('error', () => res.destroy()).pipe(res);
  } catch { res.writeHead(404, {'Content-Type':'text/plain; charset=utf-8'}); res.end('Not found'); }
});
server.on('error', error => { console.error(error.message); process.exitCode=1; });
server.listen(port, '127.0.0.1', () => console.log(`Ready: http://127.0.0.1:${port}`));
process.on('SIGINT', () => server.close(() => process.exit(0)));
