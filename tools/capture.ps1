param(
 [Parameter(Mandatory=$true)][string]$Port,
 [int]$Seconds=300,
 [string]$OutDir='',
 [int]$Baud=460800,
 [int]$ReadySeconds=6,
 [switch]$Recalibrate
)
$ErrorActionPreference='Stop'
$taskRoot=Split-Path $PSScriptRoot -Parent
if($Port -notin [System.IO.Ports.SerialPort]::GetPortNames()){throw "Port unavailable: $Port"}
if(!$OutDir){$OutDir=Join-Path $taskRoot ('results\capture_'+(Get-Date -Format 'yyyyMMdd_HHmmss'))}
New-Item -ItemType Directory -Path $OutDir -Force | Out-Null
$taskBin=Join-Path $OutDir 'raw.bin'
if(Test-Path -LiteralPath $taskBin){throw "Refusing to overwrite existing capture: $taskBin"}
$taskSerial=[System.IO.Ports.SerialPort]::new($Port,$Baud,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$taskSerial.ReadTimeout=100
$taskSerial.WriteTimeout=2000
$taskSerial.ReadBufferSize=1048576
$taskSerial.DtrEnable=$false
$taskSerial.RtsEnable=$false
$taskFile=[System.IO.File]::Open($taskBin,[System.IO.FileMode]::CreateNew)
$taskBuffer=[byte[]]::new(32768)
$taskBytes=0L
$taskCapturedAt=[DateTimeOffset]::Now.ToString('o')
function Receive-CaptureBytes {
 try{
  $taskRead=$taskSerial.Read($taskBuffer,0,$taskBuffer.Length)
  if($taskRead -gt 0){$taskFile.Write($taskBuffer,0,$taskRead);$script:taskBytes+=$taskRead}
 }catch [TimeoutException]{}
}
try{
 $taskSerial.Open()
 $taskSerial.Write("STOP`nINFO`n")
 if($Recalibrate){$taskSerial.Write("CALIBRATE`n")}
 Write-Host "Port $Port @ $Baud; waiting $ReadySeconds seconds for stable calibration."
 $taskClock=[Diagnostics.Stopwatch]::StartNew()
 while($taskClock.Elapsed.TotalSeconds -lt $ReadySeconds){Receive-CaptureBytes}
 $taskSerial.Write("START`n")
 Write-Host "Capturing $Seconds seconds to $taskBin"
 $taskClock.Restart()
 $taskNextProgress=15
 while($taskClock.Elapsed.TotalSeconds -lt $Seconds){
  Receive-CaptureBytes
  if($taskClock.Elapsed.TotalSeconds -ge $taskNextProgress){
   Write-Host ("{0:N0}s / {1}s; {2:N0} bytes" -f $taskClock.Elapsed.TotalSeconds,$Seconds,$taskBytes)
   $taskNextProgress+=15
  }
 }
 $taskSerial.Write("STOP`n")
 $taskClock.Restart()
 $taskRetried=$false
 while($taskClock.Elapsed.TotalSeconds -lt 3){
  Receive-CaptureBytes
  if(!$taskRetried -and $taskClock.Elapsed.TotalSeconds -gt 1.5){$taskSerial.Write("STOP`nINFO`n");$taskRetried=$true}
 }
}finally{
 if($taskSerial.IsOpen){try{$taskSerial.Write("STOP`n")}catch{};$taskSerial.Close()}
 $taskSerial.Dispose();$taskFile.Dispose()
}
@{port=$Port;baud=$Baud;requested_seconds=$Seconds;captured_at=$taskCapturedAt;bytes=$taskBytes;source='hardware'} |
 ConvertTo-Json | Set-Content -LiteralPath (Join-Path $OutDir 'capture.json') -Encoding utf8
Write-Host "Saved $taskBytes bytes. Decode with: node tools/decode-capture.mjs <capture-directory>"
