#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Headless build script for Stratosonde firmware (STM32WLE5).
.DESCRIPTION
    Locates the STM32CubeIDE ARM toolchain, runs the Debug makefile,
    and produces the .elf + .bin output. Returns exit code 0 on success.
.PARAMETER Clean
    If specified, runs 'make clean' before building.
.PARAMETER Jobs
    Parallel build jobs (default 8).
.EXAMPLE
    .\build.ps1
    .\build.ps1 -Clean
    .\build.ps1 -Jobs 4
#>
param(
    [switch]$Clean,
    [int]$Jobs = 8
)

$ErrorActionPreference = "Stop"

# ---------- locate toolchain ----------
$stRoot = "C:\ST"
$pluginsDir = Get-ChildItem "$stRoot" -Directory -Filter "STM32CubeIDE*" -ErrorAction SilentlyContinue |
              Sort-Object Name -Descending | Select-Object -First 1
if ($pluginsDir) {
    $pluginsPath = Join-Path $pluginsDir.FullName "STM32CubeIDE\plugins"
} else {
    $pluginsPath = $null
}
$ideDir = if ($pluginsPath -and (Test-Path $pluginsPath)) {
    Get-ChildItem $pluginsPath -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -like "com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*" } |
        Sort-Object Name -Descending | Select-Object -First 1
} else { $null }

if (-not $ideDir) {
    Write-Error "Cannot find STM32CubeIDE GNU toolchain plugin under $stRoot"
    exit 1
}

$toolchainBin = Join-Path $ideDir.FullName "tools\bin"
if (-not (Test-Path (Join-Path $toolchainBin "arm-none-eabi-gcc.exe"))) {
    Write-Error "arm-none-eabi-gcc not found in $toolchainBin"
    exit 1
}

$env:PATH = "$toolchainBin;$env:PATH"
Write-Host "Toolchain: $toolchainBin" -ForegroundColor Cyan

# ---------- check Debug makefile ----------
$debugDir = Join-Path $PSScriptRoot "Debug"
if (-not (Test-Path (Join-Path $debugDir "makefile"))) {
    Write-Error "Debug/makefile not found. Export the project from STM32CubeIDE first."
    exit 1
}

# ---------- optional clean ----------
if ($Clean) {
    Write-Host "`n>>> Cleaning build artifacts" -ForegroundColor Yellow
    # STM32CubeIDE makefiles use 'rm -rf' which fails on Windows without Unix tools.
    # Use PowerShell-native cleanup instead.
    # Clean compiled objects and outputs; preserve makefile support files (objects.list, objects.mk, etc.)
    Get-ChildItem $debugDir -Recurse -Include "*.o","*.d","*.su","*.cyclo" -ErrorAction SilentlyContinue |
        Remove-Item -Force -ErrorAction SilentlyContinue
    @("Radio_Sonde_E5_HF_EU.elf","Radio_Sonde_E5_HF_EU.bin","Radio_Sonde_E5_HF_EU.map","Radio_Sonde_E5_HF_EU.list") | ForEach-Object {
        $f = Join-Path $debugDir $_
        if (Test-Path $f) { Remove-Item $f -Force }
    }
    Write-Host "Clean complete" -ForegroundColor Green
}

# ---------- ensure objects.list exists (IDE-generated linker response file) ----------
$objectsList = Join-Path $debugDir "objects.list"
if (-not (Test-Path $objectsList)) {
    Write-Host "`n>>> Regenerating objects.list from subdir.mk files" -ForegroundColor Yellow
    $objs = @()
    Get-ChildItem $debugDir -Filter "subdir.mk" -Recurse -ErrorAction SilentlyContinue |
        Where-Object { $_.FullName -notlike "*\archive\*" } | ForEach-Object {
            $content = Get-Content $_.FullName -Raw
            [regex]::Matches($content, '\./(\S+\.o)\b') | ForEach-Object {
                $objs += "./" + $_.Groups[1].Value
            }
        }
    $objs = $objs | Sort-Object -Unique
    $objs | Out-File -FilePath $objectsList -Encoding ascii
    Write-Host "Generated objects.list with $($objs.Count) objects" -ForegroundColor Green
}

# ---------- build ----------
$jFlag = "-j$Jobs"
Write-Host "`n>>> make -C Debug $jFlag all" -ForegroundColor Yellow
& make -C $debugDir $jFlag all 2>&1 | Write-Host
if ($LASTEXITCODE -ne 0) {
    Write-Error "Build FAILED"
    exit $LASTEXITCODE
}

# ---------- produce .bin ----------
$elfFile = Join-Path $debugDir "Radio_Sonde_E5_HF_EU.elf"
$binFile = Join-Path $debugDir "Radio_Sonde_E5_HF_EU.bin"

if (Test-Path $elfFile) {
    Write-Host "`n>>> Generating binary: $binFile" -ForegroundColor Yellow
    & arm-none-eabi-objcopy -O binary $elfFile $binFile
    if ($LASTEXITCODE -ne 0) { Write-Error "objcopy failed"; exit $LASTEXITCODE }

    # ---------- size report ----------
    Write-Host "`n>>> Size report:" -ForegroundColor Yellow
    & arm-none-eabi-size $elfFile

    $fileSize = (Get-Item $binFile).Length
    Write-Host "`nBinary size: $([math]::Round($fileSize / 1024, 1)) KB  ($binFile)" -ForegroundColor Green
    Write-Host "Build SUCCEEDED" -ForegroundColor Green
} else {
    Write-Error "ELF file not found: $elfFile"
    exit 1
}

exit 0