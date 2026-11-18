$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$sourceDir = Join-Path $projectRoot 'Seekfree_TC264_Opensource_Library/code'
$testBuildDir = Join-Path ([IO.Path]::GetTempPath()) ('tc264-motor-test-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testBuildDir | Out-Null
# Compile copies of the actual application sources against mocked hardware.
foreach ($name in @('Motor.c', 'Motor.h', 'init.h', 'pid.c', 'pid.h', 'Function.c', 'Function.h', 'uart.c', 'uart.h')) {
    Copy-Item -LiteralPath (Join-Path $sourceDir $name) -Destination $testBuildDir
}
foreach ($name in @('image.h', 'zf_common_typedef.h', 'zf_common_headfile.h', 'test_motor_control.c')) {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination $testBuildDir
}
Push-Location $testBuildDir
try {
    & gcc -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Wno-missing-field-initializers -O2 Motor.c pid.c Function.c uart.c test_motor_control.c -o test_motor_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
    & (Join-Path $testBuildDir 'test_motor_control.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Motor control regression test failed' }
} finally {
    Pop-Location
}
