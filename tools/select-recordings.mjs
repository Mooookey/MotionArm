// Display selections preserve every retained sample's original sequence/time/value.
// Full sessions remain authoritative for transport counts and all benchmark tests.
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { framesToCsv, validateRecording } from '../web/recording.js';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const output = path.join(root, 'results/selections');
fs.mkdirSync(output, { recursive: true });
const sources = {
  dynamic: path.join(root, 'results/dynamic_20261104/recording.json'),
  displacement: path.join(root, 'results/displacement_20261104/recording.json'),
};
const records = {}, hashes = {};
for (const [key, filename] of Object.entries(sources)) {
  const bytes = fs.readFileSync(filename);
  records[key] = validateRecording(JSON.parse(bytes.toString('utf8')));
  hashes[key] = crypto.createHash('sha256').update(bytes).digest('hex');
}
const selections = [
  ['axis_x','dynamic',4600,6000,'Second-round X-dominant raise-and-return; retains cross-axis human motion.'],
  ['axis_y','dynamic',6200,8000,'Second-round Y-dominant raise-and-return; about 59.5% Y gyro energy, visibly mixed-axis.'],
  ['axis_z','dynamic',8200,9400,'Second-round Z-dominant turn-and-return; interpret with operator observation/video.'],
  ['continuous_pose','dynamic',4600,13350,'43.75-second multi-axis follow display; includes pauses and axis tests, not 43.75 seconds of uninterrupted nonzero rotation.'],
  ['short_rest','dynamic',13931,14763,'Only 4.16 seconds of final rest; cannot replace the prescribed 60/300-second static tests.'],
  ['displacement_attempt','displacement',1265,2014,'Rest-to-rest translation attempt with relatively stable attitude; about 0.58m estimated change, not evidence of accurate 15cm displacement.'],
];
const manifest = {
  created_at: new Date().toISOString(),
  rule: 'Selected by action order, main gyro axis, stable boundaries and return behavior; never by closeness to a desired result.',
  original_files_unchanged: true,
  excluded_from_axis_examples: 'Dynamic source 0–20s already starts in motion and contains earlier mixed operations; retained in the full original.',
  transport_rule: 'Only full-session END may be used for packet-loss denominator. Selected files have end=null; source_end is audit context only.',
  video: 'Operator confirms video is held on another device; attach the original video separately for final submission.',
  selections: [],
};
for (const [name, source, firstSeq, lastSeq, reason] of selections) {
  const record = records[source];
  const frames = record.frames.filter(f => f.seq >= firstSeq && f.seq <= lastSeq);
  if (!frames.length || frames[0].seq !== firstSeq || frames.at(-1).seq !== lastSeq) throw new Error(`Missing boundaries for ${name}`);
  const firstTime = record.frames[0].t_us;
  const selection = {
    name, source_file: path.relative(root,sources[source]).replaceAll('\\','/'),
    source_sha256: hashes[source], source_session_id: record.meta.session_id,
    first_seq: firstSeq, last_seq: lastSeq, first_t_us: frames[0].t_us, last_t_us: frames.at(-1).t_us,
    source_elapsed_start_s: (frames[0].t_us-firstTime)/1e6,
    source_elapsed_end_s: (frames.at(-1).t_us-firstTime)/1e6,
    duration_s: (frames.at(-1).t_us-frames[0].t_us)/1e6,
    sample_count: frames.length, reason,
  };
  const kind = source === 'displacement' ? 'displacement' : 'dynamic';
  const selected = {
    format:'imu-lab-recording-v1', source:'hardware', recording_kind:kind,
    captured_at:record.captured_at, meta:{...record.meta,recording_kind:kind},
    end:null, source_end:record.end, diagnostics:{source:'hardware'}, selection, frames,
  };
  fs.writeFileSync(path.join(output,name+'.json'),JSON.stringify(selected));
  fs.writeFileSync(path.join(output,name+'.csv'),framesToCsv(frames));
  manifest.selections.push(selection);
}
fs.writeFileSync(path.join(output,'manifest.json'),JSON.stringify(manifest,null,2));
console.log(JSON.stringify(manifest,null,2));
