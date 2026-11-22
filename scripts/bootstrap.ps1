<#
.SYNOPSIS
Native Windows PowerShell bootstrap for biasctrlaio (no WSL required).

.DESCRIPTION
Checks/installs native Windows host tools (CMake, Ninja, gperf, dtc, 7-Zip, uv, Git),
creates .venv with west/pyocd/pytest, initializes the Zephyr v4.5.0 workspace under
.west/ and deps/, installs the minimal arm-zephyr-eabi Zephyr SDK into .toolchain/,
verifies the STM32H503 pyOCD pack, and generates scripts\env.ps1.
#>
[CmdletBinding()]
param(
    [switch]$InstallDeps
)

$ErrorActionPreference = "Stop"

$RootDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$VenvDir = Join-Path $RootDir ".venv"
$VenvPython = Join-Path $VenvDir "Scripts\python.exe"
$VenvScripts = Join-Path $VenvDir "Scripts"
$WestExe = Join-Path $VenvScripts "west.exe"
$PyocdExe = Join-Path $VenvScripts "pyocd.exe"
$ToolchainDir = Join-Path $RootDir ".toolchain"
$SdkDir = Join-Path $ToolchainDir "zephyr-sdk"
$BridgeTmp = Join-Path $RootDir ".agent-bridge\tmp"
$EnvPs1 = Join-Path $RootDir "scripts\env.ps1"

New-Item -ItemType Directory -Force -Path $BridgeTmp | Out-Null
$env:TMP = $BridgeTmp
$env:TEMP = $BridgeTmp
$env:TMPDIR = $BridgeTmp

if ($InstallDeps) {
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        throw "winget is required for -InstallDeps. Install App Installer from Microsoft Store or install tools manually."
    }
    $packages = @(
        "Kitware.CMake",
        "Ninja-build.Ninja",
        "oss-winget.gperf",
        "oss-winget.dtc",
        "7zip.7zip",
        "astral-sh.uv",
        "Git.Git"
    )
    foreach ($pkg in $packages) {
        Write-Host "==> Ensuring $pkg via winget"
        winget install -e --id $pkg --accept-package-agreements --accept-source-agreements
    }
    $machinePath = [Environment]::GetEnvironmentVariable("Path", "Machine")
    $userPath = [Environment]::GetEnvironmentVariable("Path", "User")
    $env:Path = "$machinePath;$userPath"
}

function Require-Tool {
    param(
        [string]$Name,
        [string]$WingetId,
        [string[]]$ExtraPaths = @()
    )
    if (Get-Command $Name -ErrorAction SilentlyContinue) {
        return
    }
    foreach ($candidate in $ExtraPaths) {
        if (Test-Path $candidate) {
            $dir = Split-Path -Parent $candidate
            $env:Path = "$dir;$env:Path"
            return
        }
    }
    throw "Missing required tool '$Name'. Run '.\scripts\bootstrap.ps1 -InstallDeps' or 'winget install -e --id $WingetId' and reopen PowerShell."
}

Require-Tool -Name "git" -WingetId "Git.Git"
Require-Tool -Name "uv" -WingetId "astral-sh.uv" -ExtraPaths @("$env:USERPROFILE\.local\bin\uv.exe")
Require-Tool -Name "cmake" -WingetId "Kitware.CMake" -ExtraPaths @("C:\Program Files\CMake\bin\cmake.exe")
Require-Tool -Name "ninja" -WingetId "Ninja-build.Ninja"
Require-Tool -Name "dtc" -WingetId "oss-winget.dtc"
Require-Tool -Name "gperf" -WingetId "oss-winget.gperf"
Require-Tool -Name "7z" -WingetId "7zip.7zip" -ExtraPaths @("C:\Program Files\7-Zip\7z.exe")

if (-not (Test-Path $VenvPython)) {
    Write-Host "==> Creating Python virtual environment in .venv"
    & uv venv $VenvDir
    if ($LASTEXITCODE -ne 0) { throw "uv venv failed with exit code $LASTEXITCODE" }
}

Write-Host "==> Installing base Python tooling (west, pyocd, pytest)"
& uv pip install --python $VenvPython west pyocd pytest
if ($LASTEXITCODE -ne 0) { throw "uv pip install base tooling failed with exit code $LASTEXITCODE" }

if (-not (Test-Path (Join-Path $RootDir ".west"))) {
    Write-Host "==> Initializing west workspace inside $RootDir"
    & $WestExe init -l (Join-Path $RootDir "manifest")
    if ($LASTEXITCODE -ne 0) { throw "west init failed with exit code $LASTEXITCODE" }
}

Write-Host "==> Updating Zephyr modules (shallow narrow fetch)"
Push-Location $RootDir
try {
    & $WestExe update --narrow -o=--depth=1
    if ($LASTEXITCODE -ne 0) { throw "west update failed with exit code $LASTEXITCODE" }
} finally {
    Pop-Location
}

Write-Host "==> Installing Zephyr Python dependencies"
$reqBase = Join-Path $RootDir "deps\zephyr\scripts\requirements-base.txt"
& uv pip install --python $VenvPython -r $reqBase
if ($LASTEXITCODE -ne 0) { throw "uv pip install requirements-base.txt failed with exit code $LASTEXITCODE" }

function Find-ArmGcc {
    $candidates = @(
        (Join-Path $SdkDir "gnu\arm-zephyr-eabi\bin\arm-zephyr-eabi-gcc.exe"),
        (Join-Path $SdkDir "arm-zephyr-eabi\bin\arm-zephyr-eabi-gcc.exe")
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) { return $c }
    }
    return $null
}

$ArmGcc = Find-ArmGcc
if (-not $ArmGcc) {
    Write-Host "==> Installing minimal Zephyr SDK (arm-zephyr-eabi) into $SdkDir"
    New-Item -ItemType Directory -Force -Path $ToolchainDir | Out-Null

    $token = $env:GITHUB_TOKEN
    if (-not $token) { $token = $env:GH_TOKEN }
    if (-not $token -and (Get-Command gh -ErrorAction SilentlyContinue)) {
        try {
            $ghOut = (& gh auth token 2>$null)
            if ($LASTEXITCODE -eq 0 -and $ghOut) { $token = $ghOut.Trim() }
        } catch {
            # Ignore gh lookup failure; unauthenticated download still runs if rate limit allows
        }
    }

    $sdkArgs = @("sdk", "install", "-d", $SdkDir, "-t", "arm-zephyr-eabi", "-H")
    if ($token) {
        $sdkArgs += @("--personal-access-token", $token)
    }

    Push-Location $RootDir
    $prevZephyrBase = $env:ZEPHYR_BASE
    $env:ZEPHYR_BASE = Join-Path $RootDir "deps\zephyr"
    try {
        & $WestExe @sdkArgs
        if ($LASTEXITCODE -ne 0) { throw "west sdk install failed with exit code $LASTEXITCODE" }
    } finally {
        $env:ZEPHYR_BASE = $prevZephyrBase
        Pop-Location
    }
    $ArmGcc = Find-ArmGcc
} else {
    Write-Host "==> Zephyr SDK already present at $SdkDir"
}

if (-not $ArmGcc) {
    throw "Zephyr SDK installation failed; arm-zephyr-eabi-gcc.exe not found under $SdkDir"
}
$ArmBinDir = Split-Path -Parent $ArmGcc

Write-Host "==> Checking pyOCD target support for stm32h503cbtx"
$packHit = (& $PyocdExe list --targets 2>&1) | Select-String -Pattern "stm32h503cbtx"
if (-not $packHit) {
    Write-Host "==> Installing pyOCD STM32H5 CMSIS pack"
    & $PyocdExe pack update
    & $PyocdExe pack install stm32h503cbtx
    if ($LASTEXITCODE -ne 0) { throw "pyocd pack install stm32h503cbtx failed with exit code $LASTEXITCODE" }
}

$envContent = @"
# Generated by scripts\bootstrap.ps1 - do not edit manually
`$env:BIASCTRLAIO_ROOT = "$RootDir"
`$env:ZEPHYR_BASE = "$(Join-Path $RootDir 'deps\zephyr')"
`$env:ZEPHYR_SDK_INSTALL_DIR = "$SdkDir"
`$env:ZEPHYR_TOOLCHAIN_VARIANT = "zephyr"
`$env:TMP = "$BridgeTmp"
`$env:TEMP = "$BridgeTmp"
`$env:TMPDIR = "$BridgeTmp"
foreach (`$p in @("$ArmBinDir", "$VenvScripts")) {
    if (`$env:Path -notlike "*`$p*") {
        `$env:Path = "`$p;`$env:Path"
    }
}
"@
Set-Content -Path $EnvPs1 -Value $envContent -Encoding UTF8

Write-Host "==> Bootstrap complete."
Write-Host "Activate environment in PowerShell with:"
Write-Host "  . .\scripts\env.ps1"
