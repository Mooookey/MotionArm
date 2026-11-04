param([int]$Port=8000)
$ErrorActionPreference='Stop'
$taskRoot=Split-Path $PSScriptRoot -Parent
Write-Host "IMU dashboard: http://localhost:$Port (use Edge or Chrome; Ctrl+C stops server)"
& node (Join-Path $PSScriptRoot 'serve.mjs') $Port
