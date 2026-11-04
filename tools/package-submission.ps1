param([string]$Output='E:\Code\emb-exam\submission\imu_exam_20261104.zip')
$ErrorActionPreference='Stop'
$taskRoot=Split-Path $PSScriptRoot -Parent
$taskOutputDir=Split-Path $Output -Parent
New-Item -ItemType Directory -Force -Path $taskOutputDir | Out-Null
if(Test-Path -LiteralPath $Output){throw "Archive already exists; choose a new -Output: $Output"}
$taskInputs=@('README.md','PROTOCOL.md','firmware','web','tools','tests','docs','results') |
 ForEach-Object {Join-Path $taskRoot $_}
Compress-Archive -LiteralPath $taskInputs -DestinationPath $Output -CompressionLevel Optimal
Get-Item -LiteralPath $Output | Select-Object FullName,Length
Get-FileHash -LiteralPath $Output -Algorithm SHA256 | Select-Object Algorithm,Hash
