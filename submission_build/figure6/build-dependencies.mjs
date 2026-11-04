import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const dataRoot = 'C:/Users/21211/AppData/Local/Arduino15';
const librariesRoot = path.join(dataRoot, 'libraries');
const coreRoot = path.join(dataRoot, 'packages/esp32/hardware/esp32/3.3.12');
const buildRoot = path.join(root, 'artifacts/ImuExam');
const sha = file => createHash('sha256').update(fs.readFileSync(file)).digest('hex');
const relative = file => path.relative(root, file).replaceAll('\\', '/');
const properties = text => Object.fromEntries(text.split(/\r?\n/).filter(line => line && !line.startsWith('#') && line.includes('=')).map(line => { const p = line.indexOf('='); return [line.slice(0, p).trim(), line.slice(p + 1).trim()]; }));
const libraryConfigs = [
  ['Adafruit_MPU6050', 'license.txt', 'BSD-3-Clause', 'https://github.com/adafruit/Adafruit_MPU6050'],
  ['Adafruit_BusIO', 'LICENSE', 'MIT', 'https://github.com/adafruit/Adafruit_BusIO'],
  ['Adafruit_Unified_Sensor', 'LICENSE.txt', 'Apache-2.0', 'https://github.com/adafruit/Adafruit_Sensor'],
];
const usedLibraries = JSON.parse(fs.readFileSync(path.join(buildRoot, 'libraries.cache'), 'utf8'));
const libraries = libraryConfigs.map(([directory, licenseFilename, license, url]) => {
  const directoryPath = path.join(librariesRoot, directory);
  const propsPath = path.join(directoryPath, 'library.properties');
  const props = properties(fs.readFileSync(propsPath, 'utf8'));
  const licensePath = path.join(directoryPath, licenseFilename);
  return {
    name: props.name, directory, version: props.version, upstream_url: props.url || url,
    version_source: propsPath.replaceAll('\\', '/'), library_properties_sha256: sha(propsPath),
    installed_directory: directoryPath.replaceAll('\\', '/'), used_in_compiled_build: usedLibraries.some(p => path.resolve(p) === path.resolve(directoryPath)),
    license, license_filename: licenseFilename, license_source: licensePath.replaceAll('\\', '/'), license_sha256: sha(licensePath),
    license_upstream_url: `${url}/blob/master/${licenseFilename}`,
    declared_dependencies: props.depends ? props.depends.split(',').map(s => s.trim()) : [],
  };
});
const corePackagePath = path.join(coreRoot, 'package.json');
const corePackage = JSON.parse(fs.readFileSync(corePackagePath, 'utf8'));
const installed = JSON.parse(fs.readFileSync(path.join(coreRoot, 'installed.json'), 'utf8'));
const platform = installed.packages[0].platforms.find(p => p.version === '3.3.12');
const options = JSON.parse(fs.readFileSync(path.join(buildRoot, 'build.options.json'), 'utf8'));
if (corePackage.version !== '3.3.12' || options.fqbn !== 'esp32:esp32:esp32') throw new Error('Unexpected core/board in existing successful build');
const definitions = {
  'ImuExam.ino.bin': { role: 'application', flash_offset: '0x10000' },
  'ImuExam.ino.bootloader.bin': { role: 'bootloader', flash_offset: '0x1000' },
  'ImuExam.ino.partitions.bin': { role: 'partition_table', flash_offset: '0x8000' },
  'boot_app0.bin': { role: 'boot_app0', flash_offset: '0xe000' },
  'ImuExam.ino.merged.bin': { role: 'merged_4MB_flash_image', flash_offset: '0x0' },
};
const files = fs.readdirSync(buildRoot).filter(name => name.endsWith('.bin')).sort().map(name => {
  const full = path.join(buildRoot, name), stat = fs.statSync(full), canonical = name.replace('_flashed.bin', '.bin');
  const definition = definitions[name];
  return { filename: name, source_path: relative(full), size_bytes: stat.size, sha256: sha(full), modified_at: stat.mtime.toISOString(),
    recommended_for_delivery: !!definition, ...(definition ?? { role: 'duplicate_build_or_flash_copy', canonical_filename: canonical, canonical_sha256_matches: fs.existsSync(path.join(buildRoot, canonical)) && sha(full) === sha(path.join(buildRoot, canonical)) }) };
});
const cli = 'D:/ESP32/app/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe';
let cliVersion;
try { cliVersion = execFileSync(cli, ['version'], { encoding: 'utf8', windowsHide: true }).trim(); } catch { cliVersion = 'Not queried; see AGENTS.md'; }
const compileLog = fs.readFileSync(path.join(buildRoot, 'compile.log'), 'utf8');
const uploadLog = fs.readFileSync(path.join(buildRoot, 'upload.log'), 'utf8');
const sharpPath = 'C:/Users/21211/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/sharp/package.json';
const dependencies = {
  format: 'imu-exam-dependencies-v1', created_at: new Date().toISOString(), workspace: root,
  evidence: 'Read from installed library.properties/license files, core package metadata, successful build output and local binary contents; no packages installed or updated.',
  firmware: {
    fqbn: options.fqbn, board: 'ESP32 Dev Module', flash_size_bytes: 4194304,
    arduino_core: { version: corePackage.version, package: 'esp32:esp32', installed_directory: coreRoot, license: corePackage.license,
      metadata_source: corePackagePath, metadata_sha256: sha(corePackagePath), upstream_url: corePackage.repository.url,
      archive_url: platform.url, archive_checksum_from_installed_metadata: platform.checksum,
      license_evidence: ['package.json: license field', 'cores/esp32/esp32-hal.h: LGPL 2.1 or later header'],
      scope_note: 'Core metadata license; bundled SDK components retain their own notices.' },
    arduino_cli: { path: cli, version_output: cliVersion },
    libraries,
    built_in_libraries: ['Wire', 'SPI'],
    dependency_note: 'MPU6050 library.properties also declares GFX/SSD1306, but libraries.cache for this successful build lists only MPU6050, BusIO, Unified Sensor plus Wire/SPI. No OLED is used.',
    runtime_serial: { baud: 460800, data_bits: 8, parity: 'none', stop_bits: 1 }, upload_baud: 115200,
  },
  host: { node: { version: process.version, executable: process.execPath, platform: process.platform, architecture: process.arch },
    web_and_analysis: 'Browser Web Serial/Canvas and Node built-in modules; no project npm dependencies or CDN required.',
    optional_figure_rasterizer: fs.existsSync(sharpPath) ? { package: 'sharp', version: JSON.parse(fs.readFileSync(sharpPath, 'utf8')).version, source: sharpPath, usage: 'tools/rasterize-report.mjs, full uncropped SVG-to-PNG output' } : null },
  existing_successful_build: {
    directory: relative(buildRoot), fqbn: options.fqbn, sketch: 'firmware/ImuExam',
    compile_log: relative(path.join(buildRoot, 'compile.log')), compile_log_sha256: sha(path.join(buildRoot, 'compile.log')),
    compile_memory_summary: compileLog.trim().split(/\r?\n/),
    upload_log: relative(path.join(buildRoot, 'upload.log')), upload_log_sha256: sha(path.join(buildRoot, 'upload.log')),
    upload_log_contains_hash_verified: uploadLog.includes('Hash of data verified.'),
    flash_arguments_file: relative(path.join(buildRoot, 'flash_args')),
    recommended_bin_files: Object.keys(definitions), binaries: files,
    delivery_note: 'Use either the merged image at 0x0 or the four separate images at their recorded offsets. _flashed copies have identical hashes to their canonical counterparts and need not be duplicated in the submission.',
  },
};
fs.writeFileSync(path.join(root, 'submission_build/dependencies.json'), JSON.stringify(dependencies, null, 2) + '\n');
console.log(JSON.stringify({ output: 'submission_build/dependencies.json', core: corePackage.version, fqbn: options.fqbn, node: process.version, libraries: libraries.map(l => ({ name: l.name, version: l.version, license: l.license })), recommended_binaries: files.filter(f => f.recommended_for_delivery).map(f => ({ filename: f.filename, size_bytes: f.size_bytes, sha256: f.sha256 })) }, null, 2));
