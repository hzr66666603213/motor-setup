param(
  [string]$OutDir = (Join-Path $PSScriptRoot "build")
)

$ErrorActionPreference = "Stop"

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$bytes = [System.Collections.Generic.List[byte]]::new()

function Add-U16 {
  param([UInt64]$Value)
  $bytes.Add([byte]($Value -band 0xFF))
  $bytes.Add([byte](($Value -shr 8) -band 0xFF))
}

function Add-U32 {
  param([UInt64]$Value)
  $bytes.Add([byte]($Value -band 0xFF))
  $bytes.Add([byte](($Value -shr 8) -band 0xFF))
  $bytes.Add([byte](($Value -shr 16) -band 0xFF))
  $bytes.Add([byte](($Value -shr 24) -band 0xFF))
}

function Add-HexRecord {
  param(
    [System.Text.StringBuilder]$Builder,
    [byte]$Count,
    [UInt16]$Address,
    [byte]$Type,
    [byte[]]$Data
  )

  $sum = [int]$Count + (($Address -shr 8) -band 0xFF) + ($Address -band 0xFF) + [int]$Type
  foreach ($b in $Data) {
    $sum += [int]$b
  }
  $checksum = ((- $sum) -band 0xFF)

  [void]$Builder.AppendFormat(":{0:X2}{1:X4}{2:X2}", $Count, $Address, $Type)
  foreach ($b in $Data) {
    [void]$Builder.AppendFormat("{0:X2}", $b)
  }
  [void]$Builder.AppendFormat("{0:X2}`r`n", $checksum)
}

# Vector table at 0x08000000.
Add-U32 0x20005000 # Initial stack pointer, 20 KB SRAM top.
Add-U32 0x08000041 # Reset_Handler at 0x08000040, Thumb bit set.
for ($i = 2; $i -lt 16; $i++) {
  Add-U32 0x08000073 # Default_Handler at 0x08000072, Thumb bit set.
}

if ($bytes.Count -ne 0x40) {
  throw "Unexpected vector table size: $($bytes.Count)"
}

# Reset_Handler, starting at 0x08000040.
Add-U16 0x480C # ldr r0, =RCC_APB2ENR
Add-U16 0x6801 # ldr r1, [r0]
Add-U16 0x2210 # movs r2, #0x10
Add-U16 0x4311 # orrs r1, r2
Add-U16 0x6001 # str r1, [r0]
Add-U16 0x480B # ldr r0, =GPIOC_CRH
Add-U16 0x6801 # ldr r1, [r0]
Add-U16 0x4A0B # ldr r2, =GPIOC_CRH_CLEAR_PC13
Add-U16 0x4011 # ands r1, r2
Add-U16 0x4A0B # ldr r2, =GPIOC_CRH_PC13_OUTPUT
Add-U16 0x4311 # orrs r1, r2
Add-U16 0x6001 # str r1, [r0]
Add-U16 0x480A # loop: ldr r0, =GPIOC_BRR
Add-U16 0x490B # ldr r1, =PC13_BIT
Add-U16 0x6001 # str r1, [r0], LED on for active-low PC13
Add-U16 0x4B0B # ldr r3, =DELAY_COUNT
Add-U16 0x3B01 # delay_on: subs r3, #1
Add-U16 0xD1FD # bne delay_on
Add-U16 0x480A # ldr r0, =GPIOC_BSRR
Add-U16 0x4908 # ldr r1, =PC13_BIT
Add-U16 0x6001 # str r1, [r0], LED off
Add-U16 0x4B08 # ldr r3, =DELAY_COUNT
Add-U16 0x3B01 # delay_off: subs r3, #1
Add-U16 0xD1FD # bne delay_off
Add-U16 0xE7F2 # b loop
Add-U16 0xE7FE # Default_Handler: b Default_Handler

# Literal pool, starting at 0x08000074.
Add-U32 0x40021018 # RCC_APB2ENR
Add-U32 0x40011004 # GPIOC_CRH
Add-U32 4279238655 # 0xFF0FFFFF, clear PC13 CNF/MODE bits
Add-U32 0x00200000 # PC13 output push-pull, 2 MHz
Add-U32 0x40011014 # GPIOC_BRR
Add-U32 0x00002000 # PC13 bit
Add-U32 0x00080000 # Delay loop count
Add-U32 0x40011010 # GPIOC_BSRR

$binPath = Join-Path $OutDir "stm32f103c8_pc13_blink.bin"
$hexPath = Join-Path $OutDir "stm32f103c8_pc13_blink.hex"

[System.IO.File]::WriteAllBytes($binPath, $bytes.ToArray())

$hex = [System.Text.StringBuilder]::new()
Add-HexRecord $hex 2 0 4 ([byte[]](0x08, 0x00))

$offset = 0
while ($offset -lt $bytes.Count) {
  $count = [Math]::Min(16, $bytes.Count - $offset)
  $chunk = New-Object byte[] $count
  $bytes.CopyTo($offset, $chunk, 0, $count)
  Add-HexRecord $hex ([byte]$count) ([UInt16]$offset) 0 $chunk
  $offset += $count
}

Add-HexRecord $hex 0 0 1 ([byte[]]@())
[System.IO.File]::WriteAllText($hexPath, $hex.ToString(), [System.Text.Encoding]::ASCII)

Write-Host "Generated $binPath"
Write-Host "Generated $hexPath"
Write-Host "Size: $($bytes.Count) bytes"
