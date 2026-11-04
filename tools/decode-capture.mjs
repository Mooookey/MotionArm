import fs from 'node:fs';
import path from 'node:path';
import { FrameParser } from '../web/protocol.js';
import { analyzeSession } from '../web/metrics.js';
import { framesToCsv } from '../web/recording.js';

const directory = path.resolve(process.argv[2] ?? 'results/latest');
const parser = new FrameParser();
const bytes = fs.readFileSync(path.join(directory, 'raw.bin'));
const sessions = new Map();
let currentMeta = {};
const errors = [];
const get = id => {
  if (!sessions.has(id)) sessions.set(id, { meta: { ...currentMeta }, frames: [], end: null, status: null });
  return sessions.get(id);
};
for (let offset = 0; offset < bytes.length; offset += 32768) {
  for (const packet of parser.push(bytes.subarray(offset, offset + 32768))) {
    const d = packet.data;
    if (packet.kind === 'META') {
      currentMeta = d;
      const s = get(d.session_id); s.meta = d;
    } else if (packet.kind === 'DATA') get(d.session_id).frames.push(d);
    else if (packet.kind === 'END') get(d.session_id).end = d;
    else if (packet.kind === 'STATUS') get(d.session_id).status = d;
    else if (packet.kind === 'ERROR') errors.push(d);
  }
}
const candidates = [...sessions.values()].filter(s => s.frames.length);
if (!candidates.length) {
  console.error(JSON.stringify({ error: 'No DATA received', errors, meta: currentMeta, last_status: [...sessions.values()].at(-1)?.status, diagnostics: parser.diagnostics }, null, 2));
  process.exitCode = 1;
} else {
  const session = candidates.at(-1);
  const capturePath = path.join(directory, 'capture.json');
  const capture = fs.existsSync(capturePath) ? JSON.parse(fs.readFileSync(capturePath, 'utf8').replace(/^\uFEFF/, '')) : {};
  const diagnostics = { ...parser.diagnostics };
  const recording = { ...session, diagnostics, source: 'hardware', captured_at: capture.captured_at, capture, errors };
  const metrics = analyzeSession(session.frames, session.meta, session.end, diagnostics);
  fs.writeFileSync(path.join(directory, 'recording.json'), JSON.stringify(recording));
  fs.writeFileSync(path.join(directory, 'metrics.json'), JSON.stringify(metrics, null, 2));
  fs.writeFileSync(path.join(directory, 'metadata.json'), JSON.stringify({ ...recording, frames: undefined, metrics }, null, 2));
  fs.writeFileSync(path.join(directory, 'data.csv'), framesToCsv(session.frames));
  console.log(JSON.stringify({ directory, rows: session.frames.length, meta: session.meta, end: session.end, diagnostics, errors, metrics }, null, 2));
}
