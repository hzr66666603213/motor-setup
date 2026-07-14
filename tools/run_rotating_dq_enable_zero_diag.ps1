param(
  [string]$PortName = "COM3",
  [int]$BaudRate = 115200,
  [string]$ProjectRoot = "C:\Users\Shu\Documents\GitHub\motor_set",
  [string]$ProgrammerCli = "D:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe",
  [int]$CaptureSecondsAfterReset = 35,
  [int]$SwdFrequencyKHz = 200,
  [string]$LogPrefix = "rotating_dq_enable_zero_diag",
  [switch]$NoFlash,
  [switch]$NoReset
)

$ErrorActionPreference = "Stop"

$debugDir = Join-Path $ProjectRoot "firmware\odrive_v36_cube\Debug"
$hexPath = Join-Path $debugDir "odrive_v36_cube.hex"
$elfPath = Join-Path $debugDir "odrive_v36_cube.elf"
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"

$rawPath = Join-Path $debugDir "${LogPrefix}_raw_$timestamp.bin"
$comPath = Join-Path $debugDir "${LogPrefix}_com3_$timestamp.log"
$hostPath = Join-Path $debugDir "${LogPrefix}_host_$timestamp.log"
$flashPath = Join-Path $debugDir "${LogPrefix}_flash_$timestamp.log"
$resetPath = Join-Path $debugDir "${LogPrefix}_reset_$timestamp.log"
$resetGatePath = Join-Path $debugDir "${LogPrefix}_reset_seen_$timestamp.flag"

if (-not (Test-Path -LiteralPath $hexPath)) {
  throw "HEX not found: $hexPath"
}
if (-not (Test-Path -LiteralPath $elfPath)) {
  throw "ELF not found: $elfPath"
}
if (-not (Test-Path -LiteralPath $ProgrammerCli)) {
  throw "STM32_Programmer_CLI not found: $ProgrammerCli"
}

$elfHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $elfPath).Hash
$hexHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $hexPath).Hash

$captureJob = Start-Job -ArgumentList @(
  $PortName,
  $BaudRate,
  $rawPath,
  $comPath,
  $hostPath,
  $resetGatePath,
  $CaptureSecondsAfterReset,
  $elfHash,
  $hexHash,
  $flashPath,
  $resetPath
) -ScriptBlock {
  param($jobPortName, $jobBaudRate, $jobRawPath, $jobComPath, $jobHostPath, $jobResetGatePath, $jobCaptureSecondsAfterReset, $jobElfHash, $jobHexHash, $jobFlashPath, $jobResetPath)

  $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
  $hostWriter = New-Object System.IO.StreamWriter($jobHostPath, $false, $utf8NoBom)
  $textWriter = New-Object System.IO.StreamWriter($jobComPath, $false, $utf8NoBom)
  $rawStream = [System.IO.File]::Open($jobRawPath, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::Read)
  $serial = New-Object System.IO.Ports.SerialPort($jobPortName, $jobBaudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
  $serial.Handshake = [System.IO.Ports.Handshake]::None
  $serial.DtrEnable = $false
  $serial.RtsEnable = $false
  $serial.ReadTimeout = 100

  $buffer = New-Object byte[] 4096
  $totalBytes = 0
  $tail = ""
  $marker = ""
  $resetSeen = $false
  $resetTime = $null
  $postMarkerDeadline = $null

  try {
    $serial.Open()
    $hostWriter.WriteLine(("SERIAL_CAPTURE_ARMED=YES time={0:o} port={1} baud={2}" -f (Get-Date), $jobPortName, $jobBaudRate))
    $hostWriter.WriteLine("PRE_RESET_MARKERS_IGNORED=YES")
    $hostWriter.WriteLine(("ELF_SHA256={0}" -f $jobElfHash))
    $hostWriter.WriteLine(("HEX_SHA256={0}" -f $jobHexHash))
    $hostWriter.WriteLine(("RAW={0}" -f $jobRawPath))
    $hostWriter.WriteLine(("COM3={0}" -f $jobComPath))
    $hostWriter.WriteLine(("HOST={0}" -f $jobHostPath))
    $hostWriter.WriteLine(("FLASH={0}" -f $jobFlashPath))
    $hostWriter.WriteLine(("RESET={0}" -f $jobResetPath))
    $hostWriter.Flush()

    while ($true) {
      $readCount = 0
      try {
        $readCount = $serial.Read($buffer, 0, $buffer.Length)
      } catch [System.TimeoutException] {
        $readCount = 0
      }

      if ($readCount -gt 0) {
        $rawStream.Write($buffer, 0, $readCount)
        $chunk = [System.Text.Encoding]::ASCII.GetString($buffer, 0, $readCount)
        $textWriter.Write($chunk)
        $textWriter.Flush()
        $totalBytes += $readCount

        $tail = $tail + $chunk
        if ($tail.Length -gt 16384) {
          $tail = $tail.Substring($tail.Length - 16384)
        }
      }

      if (-not $resetSeen -and (Test-Path -LiteralPath $jobResetGatePath)) {
        $resetSeen = $true
        $resetTime = Get-Date
        $tail = ""
        $hostWriter.WriteLine(("RESET_GATE_SEEN=YES time={0:o}" -f $resetTime))
        $hostWriter.Flush()
      }

      if ($resetSeen) {
        foreach ($candidate in @(
            "ROTATING_DQ_ENABLE_ZERO_DIAGNOSTIC_RESULT_VALID=",
            "ROTATING_DQ_ENABLE_ZERO_PRIMARY_CLASSIFICATION",
            "ROTATING_DQ_DIRECTION_TEST_FAIL",
            "ROTATING_DQ_DIRECTION_TEST_PASS",
            "BREAKAWAY_HANDOFF_TEST_FAIL",
            "BREAKAWAY_HANDOFF_TEST_PASS",
            "ROTATING_DQ_BREAKAWAY_TO_2RPM_HANDOFF_FAIL",
            "ROTATING_DQ_BREAKAWAY_TO_2RPM_HANDOFF_PASS",
            "ROTATING_DQ_ENABLE_ZERO_DIAGNOSTIC COMPLETE",
            "ROTATING_DQ_ENABLE_ZERO_DIAGNOSTIC FAIL"
          )) {
          if ($tail.Contains($candidate)) {
            $marker = $candidate
            if (-not $postMarkerDeadline) {
              $postMarkerDeadline = (Get-Date).AddSeconds(3)
            }
            break
          }
        }

        if ($postMarkerDeadline -and (Get-Date) -ge $postMarkerDeadline) {
          break
        }
        if ($resetTime -and (Get-Date) -ge $resetTime.AddSeconds([double]$jobCaptureSecondsAfterReset)) {
          break
        }
      } else {
        Start-Sleep -Milliseconds 10
      }
    }

    $hostWriter.WriteLine(("SERIAL_CAPTURE_DONE time={0:o} bytes={1} marker={2} reset_seen={3}" -f (Get-Date), $totalBytes, $marker, $resetSeen))
  } catch {
    $hostWriter.WriteLine(("SERIAL_CAPTURE_ERROR time={0:o} error={1}" -f (Get-Date), $_.Exception.Message))
    throw
  } finally {
    if ($serial.IsOpen) {
      $serial.Close()
    }
    $rawStream.Flush()
    $rawStream.Close()
    $textWriter.Flush()
    $textWriter.Close()
    $hostWriter.Flush()
    $hostWriter.Close()
  }
}

Start-Sleep -Seconds 1

if (-not $NoFlash) {
  & $ProgrammerCli -c port=SWD freq=$SwdFrequencyKHz -w $hexPath -v 2>&1 | Tee-Object -FilePath $flashPath
  if ($LASTEXITCODE -ne 0) {
    throw "Flash/verify failed with exit code $LASTEXITCODE"
  }
} else {
  "FLASH_SKIPPED=YES" | Tee-Object -FilePath $flashPath
}

if (-not $NoReset) {
  & $ProgrammerCli -c port=SWD freq=$SwdFrequencyKHz -rst 2>&1 | Tee-Object -FilePath $resetPath
  if ($LASTEXITCODE -ne 0) {
    throw "Reset failed with exit code $LASTEXITCODE"
  }
  Set-Content -LiteralPath $resetGatePath -Value ("reset_complete={0:o}" -f (Get-Date)) -Encoding ASCII
} else {
  "RESET_SKIPPED=YES" | Tee-Object -FilePath $resetPath
  Set-Content -LiteralPath $resetGatePath -Value ("reset_skipped={0:o}" -f (Get-Date)) -Encoding ASCII
}

Wait-Job -Id $captureJob.Id | Out-Null
Receive-Job -Id $captureJob.Id
Remove-Job -Id $captureJob.Id
Remove-Item -LiteralPath $resetGatePath -ErrorAction SilentlyContinue

Get-Item -LiteralPath $rawPath, $comPath, $hostPath, $flashPath, $resetPath |
  Select-Object FullName, Length, LastWriteTime
