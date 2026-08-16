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
    [switch]$Flight,
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

# ---------- F-009 (#209): first-class flight build ----------
# PIPE-04 (#265, Phase 6): the flight define now travels via the centralized
# PROJECT_CPPFLAGS make variable - NO source mutation, nothing to restore.
# tools/check_project_cppflags.py proves every active C recipe honours the
# variable; the embedded-marker gate below proves the binary is a flight
# build. Deliberately NOT a CubeMX make-target (regen would eat it).
if ($Flight) {
    Write-Host "`n>>> FLIGHT build: PROJECT_CPPFLAGS=-DSONDE_FLIGHT_BUILD (no source mutation)" -ForegroundColor Magenta
    & python tools\check_project_cppflags.py
    if ($LASTEXITCODE -ne 0) { Write-Error "PROJECT_CPPFLAGS gate failed"; exit $LASTEXITCODE }
    $Clean = $true   # flight builds are always from-scratch
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
$makeArgs = @("-C", $debugDir, $jFlag, "all")
if ($Flight) { $makeArgs += "PROJECT_CPPFLAGS=-DSONDE_FLIGHT_BUILD" }
Write-Host "`n>>> make $($makeArgs -join ' ')" -ForegroundColor Yellow
& make @makeArgs 2>&1 | Write-Host
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

# ---------- F-009 (#209): flight verification + artifact manifest ----------
if ($Flight) {
    # Nothing to restore: PROJECT_CPPFLAGS never touched the source tree.

    # Release gate: the binary must carry the embedded flight marker (F12/#173).
    $binText = [System.Text.Encoding]::ASCII.GetString([System.IO.File]::ReadAllBytes($binFile))
    if (-not $binText.Contains("SONDE_BUILD:flight")) {
        Write-Error "FATAL: SONDE_BUILD:flight marker missing from $binFile - refusing to package"
        exit 1
    }
    if ($binText.Contains("SONDE_BUILD:debug")) {
        Write-Error "FATAL: bench marker present in a flight binary - refusing to package"
        exit 1
    }
    Write-Host "Marker gate: SONDE_BUILD:flight verified in binary" -ForegroundColor Green

    # Manifest: everything needed to reproduce/identify this exact artifact.
    $gitSha = (git rev-parse HEAD).Trim()
    $gitDirty = if ((git status --porcelain).Length -gt 0) { "DIRTY" } else { "clean" }
    $gccVersion = (& arm-none-eabi-gcc -dumpversion).Trim()
    $binHash = (Get-FileHash $binFile -Algorithm SHA256).Hash
    $elfHash = (Get-FileHash $elfFile -Algorithm SHA256).Hash
    $mapFile = Join-Path $debugDir "Radio_Sonde_E5_HF_EU.map"

    $distDir = Join-Path $PSScriptRoot "dist\flight"
    New-Item -ItemType Directory -Force -Path $distDir | Out-Null
    Copy-Item $binFile (Join-Path $distDir "Radio_Sonde_E5_HF_EU_flight.bin") -Force
    Copy-Item $elfFile (Join-Path $distDir "Radio_Sonde_E5_HF_EU_flight.elf") -Force
    if (Test-Path $mapFile) { Copy-Item $mapFile (Join-Path $distDir "Radio_Sonde_E5_HF_EU_flight.map") -Force }

    $manifest = @"
stratosonde flight build manifest (F-009/#209)
==============================================
build_mode:        flight (SONDE_FLIGHT_BUILD)
git_sha:           $gitSha
git_tree:          $gitDirty
build_timestamp:   $(Get-Date -Format "yyyy-MM-ddTHH:mm:ssK")
compiler:          arm-none-eabi-gcc $gccVersion
toolchain_dir:     $toolchainBin
compile_defines:   -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WLE5xx + PROJECT_CPPFLAGS=-DSONDE_FLIGHT_BUILD
embedded_marker:   SONDE_BUILD:flight (verified in binary)
bin_sha256:        $binHash
elf_sha256:        $elfHash
bin_size_bytes:    $fileSize
artifacts:         Radio_Sonde_E5_HF_EU_flight.bin / .elf / .map
"@
    $manifestPath = Join-Path $distDir "manifest.txt"
    $manifest | Out-File -FilePath $manifestPath -Encoding ascii
    Write-Host "`n>>> Flight artifacts + manifest:" -ForegroundColor Magenta
    Write-Host $manifest
    Write-Host "Written to $distDir" -ForegroundColor Green
}

exit 0