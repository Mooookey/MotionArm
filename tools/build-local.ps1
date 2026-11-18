param(
    [ValidateSet('Check', 'Build', 'Upload')][string]$Action = 'Check',
    [ValidatePattern('^[a-zA-Z][a-zA-Z0-9_]*$')][string]$Sketch = 'robot',
    [ValidatePattern('^COM[1-9][0-9]*$')][string]$Port,
    [ValidateRange(9600, 2000000)][int]$UploadBaud = 115200,
    [string]$Cli,
    [string]$ArduinoData,
    [string]$BuildRoot = (Join-Path $env:TEMP 'MotionArm-build')
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
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
if (!(Test-Path -LiteralPath (Join-Path $source "$Sketch.ino"))) { throw "Sketch missing: $source" }
$BuildRoot = [IO.Path]::GetFullPath($BuildRoot)
if ($BuildRoot -match '[^\x00-\x7F]') { throw 'Pass an ASCII-only -BuildRoot.' }
$stage = Join-Path $BuildRoot ([Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $stage | Out-Null
$stageSketch = Join-Path $stage $Sketch
$stageLibrary = Join-Path $stage MotionArmLink
Copy-Item -LiteralPath $source -Destination $stageSketch -Recurse
Copy-Item -LiteralPath (Join-Path $repo 'libraries/MotionArmLink') -Destination $stageLibrary -Recurse
$output = Join-Path $stage 'build'
$fqbn = 'esp32:esp32:esp32'
& $Cli compile --fqbn $fqbn --library $stageLibrary --build-path $output $stageSketch
if ($LASTEXITCODE) { throw "Compile failed: $stage" }
if ($Action -eq 'Upload') {
    & $Cli upload --fqbn $fqbn --port $Port --upload-property "upload.speed=$UploadBaud" --input-dir $output $stageSketch
    if ($LASTEXITCODE) { throw "Upload failed: $Port" }
}
"Build directory: $stage"
