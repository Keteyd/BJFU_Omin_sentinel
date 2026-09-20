$ErrorActionPreference = 'Stop'
$root = (Resolve-Path "$PSScriptRoot/../..").Path
$out = Join-Path $root 'NoMachineTemp/motor-offline-test'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$env:TEMP = $out
$env:TMP = $out
$source = [IO.File]::ReadAllText((Join-Path $PSScriptRoot '../AGVSentinel_gimbal/Src/Periphal/periph_motor.c'))
$start = $source.IndexOf('uint8_t Motor_IsMotorOffline(')
$fresh = $source.IndexOf('uint8_t Motor_IsMotorFeedbackFresh(', $start)
$end = $source.IndexOf("`n}", $fresh) + 2
if ($start -lt 0 -or $end -le $start) { throw 'Production function not found' }
[IO.File]::WriteAllText((Join-Path $out 'motor_offline_under_test.inc'), $source.Substring($start, $end - $start))
& 'D:/minGW/bin/gcc.exe' -std=c99 -Wall -Wextra -Werror "-I$out" "$PSScriptRoot/test_motor_offline_atomic.c" -o "$out/test.exe"
if ($LASTEXITCODE -ne 0) { throw 'Compile failed' }
& "$out/test.exe"
if ($LASTEXITCODE -ne 0) { throw 'Test failed' }
