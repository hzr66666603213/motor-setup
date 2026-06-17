param(
  [int]$ProbeId = 1,
  [string]$StLinkCli = "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
)

$ErrorActionPreference = "Stop"

$binPath = Join-Path $PSScriptRoot "build\stm32f103c8_pc13_blink.bin"

if (-not (Test-Path $binPath)) {
  & powershell -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "build_baremetal_firmware.ps1")
}

if (-not (Test-Path $StLinkCli)) {
  throw "ST-LINK_CLI.exe not found: $StLinkCli"
}

$connectOutput = & $StLinkCli -c ID=$ProbeId SWD UR -r32 0xE0042000 1 2>&1
$connectText = $connectOutput -join "`n"
Write-Host $connectText

if ($LASTEXITCODE -ne 0) {
  throw "Unable to connect to ST-LINK probe ID=$ProbeId."
}

if ($connectText -notmatch "Device ID:\s+0x410") {
  throw "Refusing to flash: probe ID=$ProbeId is not an STM32F103 medium-density target."
}

& $StLinkCli -c ID=$ProbeId SWD UR -ME -P $binPath 0x08000000 -V after_programming -Rst -Run

if ($LASTEXITCODE -ne 0) {
  throw "Flash operation failed."
}

Write-Host "Flash complete: $binPath"
