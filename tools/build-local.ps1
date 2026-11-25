param(
    [ValidateSet('Check', 'Build', 'Upload')][string]$Action = 'Check',
    [ValidateSet('wearable', 'robot')][string]$Sketch = 'robot',
    [ValidateSet('no_power', 'powered')][string]$RobotMode = 'no_power',
    [ValidatePattern('^COM[1-9][0-9]*$')][string]$Port,
    [ValidateRange(9600, 2000000)][int]$UploadBaud = 115200,
    [string]$Cli,
    [string]$ArduinoData,
    [string]$BuildRoot = (Join-Path $env:TEMP 'MotionArm-build'),
    [string]$ReuseBuildDirectory
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
function Get-MotionArmSourceHash([string]$Source, [string]$SketchName, [string]$Mode) {
    $files = @(Get-ChildItem -LiteralPath $Source -File | Where-Object { $_.Extension -in '.ino','.h','.cpp' })
    $files += @(Get-ChildItem -LiteralPath (Join-Path $repo 'libraries/MotionArmLink') -Recurse -File)
    if ($SketchName -eq 'robot' -and $Mode -eq 'powered') {
        $files += Get-Item -LiteralPath (Join-Path $repo 'config/robot_trial.json')
        $files += Get-Item -LiteralPath (Join-Path $Source 'generate_profile.py')
    }
    $canonical = [Text.StringBuilder]::new()
    foreach ($file in ($files | Sort-Object FullName)) {
        $relative = $file.FullName.Substring($repo.Length).Replace('\','/')
        [void]$canonical.Append($relative).Append('=').Append((Get-FileHash -LiteralPath $file.FullName).Hash).Append("`n")
    }
    $bytes = [Text.Encoding]::UTF8.GetBytes($canonical.ToString())
    $sha = [Security.Cryptography.SHA256]::Create()
    try { return ([BitConverter]::ToString($sha.ComputeHash($bytes))).Replace('-','').ToLowerInvariant() }
    finally { $sha.Dispose() }
}
if (!$Cli) {
    $arduinoCommand = Get-Command arduino-cli.exe -ErrorAction SilentlyContinue
    $locations = @(
        $arduinoCommand.Source,
        (Join-Path $env:LOCALAPPDATA 'Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe'),
        'D:/ESP32/app/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe'
    )
    $Cli = $locations | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
}
if (!$Cli) { throw 'Arduino CLI not found. Pass -Cli.' }
if ($ArduinoData) { $env:ARDUINO_DIRECTORIES_DATA = $ArduinoData }

if ($Action -eq 'Check') {
    & $Cli core list
    if ($LASTEXITCODE) { throw 'Could not read installed board cores.' }
    [System.IO.Ports.SerialPort]::GetPortNames()
    return
}
if ($Action -eq 'Upload' -and $Port -notin [System.IO.Ports.SerialPort]::GetPortNames()) {
    throw 'Specify an available -Port for upload.'
}
$source = Join-Path $repo "firmware/$Sketch"
$sourceHash = Get-MotionArmSourceHash $source $Sketch $RobotMode
if ($Sketch -ne 'robot' -and $RobotMode -ne 'no_power') { throw 'Powered mode is only valid for robot.' }
if (!(Test-Path -LiteralPath (Join-Path $source "$Sketch.ino"))) { throw "Sketch missing: $source" }
$BuildRoot = [IO.Path]::GetFullPath($BuildRoot)
if ($BuildRoot -match '[^\x00-\x7F]') { throw 'Pass an ASCII-only -BuildRoot.' }
if ($ReuseBuildDirectory) {
    if ($Action -ne 'Upload') { throw 'ReuseBuildDirectory is only valid with Upload.' }
    $stage = [IO.Path]::GetFullPath($ReuseBuildDirectory)
    $manifest = Get-Content -LiteralPath (Join-Path $stage 'build-mode.json') -Raw | ConvertFrom-Json
    if ($manifest.sketch -ne $Sketch -or $manifest.robotMode -ne $RobotMode) { throw 'Build mode/role mismatch.' }
    if ($manifest.source_sha256 -ne $sourceHash) { throw 'Build sources changed; rebuild before upload.' }
    $stageSketch = Join-Path $stage $Sketch
    $output = Join-Path $stage 'build'
    $binary = Join-Path $output "$Sketch.ino.bin"
    if (!(Test-Path -LiteralPath $binary) -or $manifest.binary_sha256 -ne (Get-FileHash -LiteralPath $binary).Hash) {
        throw 'Compiled binary missing or does not match build manifest.'
    }
    & $Cli upload --fqbn 'esp32:esp32:esp32' --port $Port --upload-property "upload.speed=$UploadBaud" --input-dir $output $stageSketch
    if ($LASTEXITCODE) { throw "Upload failed: $Port" }
    "Reused build directory: $stage"
    return
}
$stage = Join-Path $BuildRoot ([Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $stage | Out-Null
$stageSketch = Join-Path $stage $Sketch
$stageLibrary = Join-Path $stage MotionArmLink
Copy-Item -LiteralPath $source -Destination $stageSketch -Recurse
Copy-Item -LiteralPath (Join-Path $repo 'libraries/MotionArmLink') -Destination $stageLibrary -Recurse
if ($Sketch -eq 'robot' -and $RobotMode -eq 'powered') {
    # Regenerate only in the English stage. A stale tracked profile cannot be
    # silently paired with a different JSON configuration in a powered image.
    $profileHeader = Join-Path $stageSketch 'PoweredTrialProfile.h'
    $profilePython = Join-Path $repo '.venv/Scripts/python.exe'
    if (!(Test-Path -LiteralPath $profilePython)) {
        $profilePython = (Get-Command python -ErrorAction Stop).Source
    }
    & $profilePython -B (Join-Path $source 'generate_profile.py') (Join-Path $repo 'config/robot_trial.json') $profileHeader --trial
    if ($LASTEXITCODE) { throw 'Powered trial profile validation failed.' }
    if ((Get-FileHash -LiteralPath $profileHeader).Hash -ne
        (Get-FileHash -LiteralPath (Join-Path $source 'PoweredTrialProfile.h')).Hash) {
        throw 'Tracked PoweredTrialProfile.h is stale; regenerate and review it before building.'
    }
}
$output = Join-Path $stage 'build'
$fqbn = 'esp32:esp32:esp32'
$compileArgs = @('compile','--fqbn',$fqbn,'--library',$stageLibrary,'--build-path',$output)
if ($Sketch -eq 'robot' -and $RobotMode -eq 'powered') {
    $compileArgs += @('--build-property','compiler.cpp.extra_flags=-DMOTIONARM_POWERED_TRIAL=1')
}
& $Cli @compileArgs $stageSketch
if ($LASTEXITCODE) { throw "Compile failed: $stage" }
@{sketch=$Sketch;robotMode=$RobotMode;source_sha256=$sourceHash;
  binary_sha256=(Get-FileHash -LiteralPath (Join-Path $output "$Sketch.ino.bin")).Hash} |
  ConvertTo-Json | Set-Content -LiteralPath (Join-Path $stage 'build-mode.json') -Encoding ascii
if ($Action -eq 'Upload') {
    & $Cli upload --fqbn $fqbn --port $Port --upload-property "upload.speed=$UploadBaud" --input-dir $output $stageSketch
    if ($LASTEXITCODE) { throw "Upload failed: $Port" }
}
"Build directory: $stage"
