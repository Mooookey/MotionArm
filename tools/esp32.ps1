# Compatibility entry point. Current firmware builds live in English TEMP only.
param(
 [ValidateSet('Check','Build','Upload','Monitor')][string]$Action='Check',
 [string]$Sketch='robot', [string]$Port='',
 [ValidateRange(9600,2000000)][int]$Baud=115200,
 [ValidateRange(1,60)][int]$Seconds=15,
 [string]$Fqbn='esp32:esp32:esp32', [string]$ArduinoData='',
 [ValidateSet('no_power','powered')][string]$RobotMode='no_power',
 [string]$ReuseBuildDirectory=''
)
$ErrorActionPreference='Stop'
$taskRoot=Split-Path $PSScriptRoot -Parent
if($Fqbn -ne 'esp32:esp32:esp32'){throw 'Current hardware requires esp32:esp32:esp32.'}
if($Action -eq 'Monitor'){
 if($Port -notin [System.IO.Ports.SerialPort]::GetPortNames()){throw 'Specify an available COM port; close other serial owners.'}
 $taskSerial=[System.IO.Ports.SerialPort]::new($Port,$Baud)
 $taskSerial.DtrEnable=$false
 $taskSerial.RtsEnable=$false
 $taskSerial.ReadTimeout=100
 try{
  $taskSerial.Open()
  $taskClock=[Diagnostics.Stopwatch]::StartNew()
  while($taskClock.Elapsed.TotalSeconds -lt $Seconds){
   try{$taskSerial.ReadLine()}catch [TimeoutException]{}
  }
 }finally{if($taskSerial.IsOpen){$taskSerial.Close()};$taskSerial.Dispose()}
 return
}
if($Sketch -notin @('robot','wearable')){
 $taskResolved=(Resolve-Path -LiteralPath $Sketch).Path.TrimEnd([IO.Path]::DirectorySeparatorChar)
 $taskKnown=@{}
 foreach($taskName in @('robot','wearable')){
  $taskKnown[(Join-Path $taskRoot "firmware/$taskName")]= $taskName
 }
 if(!$taskKnown.ContainsKey($taskResolved)){throw 'Only current firmware/wearable and firmware/robot are build entries. Archived demos are excluded.'}
 $Sketch=$taskKnown[$taskResolved]
}
$taskArgs=@{Action=$Action;Sketch=$Sketch;RobotMode=$RobotMode;UploadBaud=$Baud}
if($Port){$taskArgs.Port=$Port}
if($ArduinoData){$taskArgs.ArduinoData=$ArduinoData}
if($ReuseBuildDirectory){$taskArgs.ReuseBuildDirectory=$ReuseBuildDirectory}
& (Join-Path $PSScriptRoot 'build-local.ps1') @taskArgs
