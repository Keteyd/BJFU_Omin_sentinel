param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$workspace = Split-Path -Parent $repo
$output = Join-Path $workspace 'NoMachineTemp/yaw-slow-native-tests'
$gimbal = Join-Path $repo 'AGVSentinel_gimbal'
$oldTmp = $env:TMP
$oldTemp = $env:TEMP
New-Item -ItemType Directory -Path $output -Force | Out-Null
try {
    $env:TMP = $output
    $env:TEMP = $output
    $cases = @(
        @{ Name = 'test_yaw_identification'; Includes = @('Inc/Modules'); Stubs = $null },
        @{ Name = 'test_yaw_identification_slow'; Includes = @('Inc/Modules'); Stubs = $null },
        @{ Name = 'test_yaw_mpc'; Includes = @('Inc/Modules'); Stubs = $null },
        @{ Name = 'test_chassis_frame'; Includes = @('Inc/Application'); Stubs = $null },
        @{ Name = 'test_yaw_backend'; Includes = @('Inc/Application'); Stubs = $null },
        @{ Name = 'test_yaw_follow'; Includes = @('Inc/Modules'); Stubs = $null },
        @{ Name = 'test_big_yaw_pid'; Includes = @('Inc/Modules', 'Inc/Algorithm'); Stubs = 'host';
           Sources = @('Src/Algorithm/alg_pid.c', 'Src/Algorithm/alg_math.c', 'Src/Algorithm/alg_filter.c');
           Flags = @('-Wno-error=parentheses') },
        @{ Name = 'test_big_yaw_manual'; Includes = @('Inc/Modules'); Stubs = $null },
        @{ Name = 'test_yaw_identification_app'; Includes = @('Inc/Modules', 'Inc/Application'); Stubs = 'ident_stubs' },
        @{ Name = 'test_can_trace'; Includes = @('Inc/Modules', 'Inc/Application'); Stubs = $null },
        @{ Name = 'test_can_trace_hal'; Includes = @('Inc/Modules'); Stubs = 'can_stubs' },
        @{ Name = 'test_pc_batch'; Includes = @('Inc/Modules', 'Inc/Application', 'Inc/Periphal'); Stubs = 'pc_stubs' }
    )
    foreach ($case in $cases) {
        $arguments = @('-std=c11', '-Wall', '-Wextra', '-Werror')
        if ($case.Flags) { $arguments += $case.Flags }
        if ($case.Stubs) { $arguments += @('-I', (Join-Path $PSScriptRoot $case.Stubs)) }
        foreach ($include in $case.Includes) { $arguments += @('-I', (Join-Path $gimbal $include)) }
        $exe = Join-Path $output ($case.Name + '.exe')
        $arguments += (Join-Path $PSScriptRoot ($case.Name + '.c'))
        if ($case.Sources) {
            foreach ($source in $case.Sources) { $arguments += (Join-Path $gimbal $source) }
        }
        $arguments += @('-lm', '-o', $exe)
        & $Compiler @arguments
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($case.Name)" }
        $fixtureArgs = @()
        if ($case.Name -eq 'test_can_trace') {
            $fixtureArgs = @((Join-Path $output 'can-trace-core.bin'))
        }
        if ($case.Name -eq 'test_yaw_identification_app') {
            $fixtureArgs = @((Join-Path $output 'short.bin'), (Join-Path $output 'remote.bin'),
                             (Join-Path $output 'can-trace-bench.bin'),
                             (Join-Path $output 'can-trace-dual-v5.bin'),
                             (Join-Path $output 'can-trace-cd.bin'),
                             (Join-Path $output 'can-trace-e.bin'),
                             (Join-Path $output 'can-trace-speed-s1.bin'),
                             (Join-Path $output 'can-trace-speed-s3.bin'))
        }
        & $exe @fixtureArgs
        if ($LASTEXITCODE -ne 0) { throw "Tests failed: $($case.Name)" }
    }
} finally {
    $env:TMP = $oldTmp
    $env:TEMP = $oldTemp
}
