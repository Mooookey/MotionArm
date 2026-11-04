param(
 [ValidateSet('Check','Build','Upload','Monitor')][string]$Action='Check',
 [string]$Sketch='', [string]$Port='', [int]$Baud=115200, [int]$Seconds=15,
 [string]$Fqbn='esp32:esp32:esp32', [string]$ArduinoData=''
)
$ErrorActionPreference='Stop'
$taskRoot=Split-Path $PSScriptRoot -Parent
$taskCli='D:\ESP32\app\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
$taskProfile=[Environment]::GetFolderPath('UserProfile')
if(!$ArduinoData){
 foreach($taskCandidate in @($env:ARDUINO_DIRECTORIES_DATA,(Join-Path $env:LOCALAPPDATA 'Arduino15'),(Join-Path $taskProfile 'AppData\Local\Arduino15'))){
  if($taskCandidate -and (Test-Path -LiteralPath (Join-Path $taskCandidate 'packages\esp32'))){$ArduinoData=$taskCandidate;break}
 }
}
if(!$ArduinoData){throw 'ESP32 Arduino data directory missing; supply -ArduinoData'}
# Process-local overrides also work when the execution sandbox redirects LOCALAPPDATA.
$env:ARDUINO_DIRECTORIES_DATA=$ArduinoData
$env:ARDUINO_DIRECTORIES_DOWNLOADS=Join-Path $ArduinoData 'staging'
$env:ARDUINO_DIRECTORIES_USER=Join-Path $taskProfile 'Documents\Arduino'
$taskLibs=Join-Path $ArduinoData 'libraries'
$taskConfig=Join-Path $PSScriptRoot 'offline.yaml'
if(!(Test-Path -LiteralPath $taskCli)){throw "CLI missing: $taskCli"}
if($Action -eq 'Check'){
 & $taskCli version
 & $taskCli --config-file $taskConfig core list
 if($LASTEXITCODE -ne 0){throw 'Core discovery failed'}
 'Serial ports:'; [System.IO.Ports.SerialPort]::GetPortNames()
 'Sensor libraries:'
 foreach($taskName in @('Adafruit_MPU6050','Adafruit_SSD1306','Adafruit_GFX_Library','Adafruit_BusIO','Adafruit_Unified_Sensor')){
  [pscustomobject]@{Library=$taskName;Present=(Test-Path -LiteralPath (Join-Path $taskLibs "$taskName/library.properties"))}
 }
 return
}
if($Action -eq 'Monitor'){
 if($Port -notin [System.IO.Ports.SerialPort]::GetPortNames()){throw 'Specify an available COM port'}
 $taskSerial=[System.IO.Ports.SerialPort]::new($Port,$Baud)
 $taskSerial.ReadTimeout=500
 New-Item -ItemType Directory -Force (Join-Path $taskRoot 'artifacts') | Out-Null
 $taskLog=Join-Path $taskRoot 'artifacts/serial.log'
 try{
  $taskSerial.Open(); $taskClock=[Diagnostics.Stopwatch]::StartNew()
  while($taskClock.Elapsed.TotalSeconds -lt $Seconds){
   try{$taskLine=$taskSerial.ReadLine(); $taskLine; Add-Content -LiteralPath $taskLog -Value $taskLine}catch [TimeoutException]{}
  }
 }finally{if($taskSerial.IsOpen){$taskSerial.Close()};$taskSerial.Dispose()}
 return
}
if(!$Sketch){throw 'Specify -Sketch with the actual sketch directory'}
$Sketch=(Resolve-Path -LiteralPath $Sketch).Path
if($Sketch -match '[^\x00-\x7F]'){throw 'Use an ASCII sketch path'}
$taskName=Split-Path $Sketch -Leaf
$taskBuild=Join-Path $taskRoot "artifacts/$taskName"
New-Item -ItemType Directory -Force $taskBuild | Out-Null
& $taskCli --config-file $taskConfig compile --fqbn $Fqbn --libraries $taskLibs --build-path $taskBuild $Sketch 2>&1 | Tee-Object -FilePath (Join-Path $taskBuild 'compile.log')
if($LASTEXITCODE -ne 0){throw 'Compilation failed; see compile.log'}
if($Action -eq 'Upload'){
 if($Port -notin [System.IO.Ports.SerialPort]::GetPortNames()){throw 'Specify an available COM port; close other serial monitors first'}
 & $taskCli --config-file $taskConfig upload --fqbn $Fqbn --port $Port --upload-property upload.speed=115200 --input-dir $taskBuild $Sketch 2>&1 | Tee-Object -FilePath (Join-Path $taskBuild 'upload.log')
 if($LASTEXITCODE -ne 0){throw 'Upload failed; see upload.log'}
}


