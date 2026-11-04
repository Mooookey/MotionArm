$ErrorActionPreference='Stop'
$taskRoot=Split-Path $PSScriptRoot -Parent
Push-Location -LiteralPath $taskRoot
try{
 & node --test web/app.test.js web/protocol.test.js tests/metrics.test.mjs
 if($LASTEXITCODE -ne 0){throw 'JavaScript tests failed'}
 $taskCompiler=Get-Command g++ -ErrorAction SilentlyContinue
 $taskCompilerPath=if($taskCompiler){$taskCompiler.Source}else{'D:\MSYS2\files\ucrt64\bin\g++.exe'}
 if(!(Test-Path -LiteralPath $taskCompilerPath)){throw 'g++ is required for the firmware math tests'}
 $taskSavedPath=$env:PATH
 try{
  $env:PATH=(Split-Path $taskCompilerPath -Parent)+';'+$env:PATH
  $taskTestDir=Join-Path $taskRoot 'artifacts\tests'
  New-Item -ItemType Directory -Path $taskTestDir -Force | Out-Null
  $taskTestExe=Join-Path $taskTestDir 'imu_math_test.exe'
  & $taskCompilerPath -std=c++17 -O2 -Wall -Wextra -Werror firmware/ImuExam/tests/math_test.cpp -o $taskTestExe
  if($LASTEXITCODE -ne 0){throw 'C++ math test compilation failed'}
  & $taskTestExe
  if($LASTEXITCODE -ne 0){throw 'Firmware math tests failed'}
 }finally{$env:PATH=$taskSavedPath}
}finally{Pop-Location}
