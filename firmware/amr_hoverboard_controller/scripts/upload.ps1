param(
    [string]$ElfPath = ".pio/build/VARIANT_USART/firmware.elf"
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$openocd = Join-Path $env:USERPROFILE ".platformio\packages\tool-openocd\bin\openocd.exe"
$elf = $ElfPath

if (-not (Test-Path $openocd)) {
    Write-Error "OpenOCD not found at $openocd"
    exit 1
}

Push-Location $repoRoot
try {
    & $openocd `
        -f interface/stlink.cfg `
        -c "transport select swd" `
        -f target/stm32f1x.cfg `
        -c "adapter speed 950" `
        -c "init" `
        -c "reset init" `
        -c "halt" `
        -c "program $elf verify reset exit"
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
