param(
    [string]$Port = "COM33",
    [int]$Baud = 921600,
    [int]$Rate = 100,
    [switch]$Mock,
    [switch]$UseTuna,
    [switch]$RecheckDeps,
    [string]$VenvDir = ""
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$CodeRoot = Split-Path -Parent $RepoRoot
$WorkspaceRoot = Split-Path -Parent $CodeRoot
$WorkspaceVenvDir = Join-Path $WorkspaceRoot ".venv-gui"

if ([string]::IsNullOrWhiteSpace($VenvDir)) {
    # Lock to shared workspace venv by default to avoid duplicate envs.
    $VenvDir = $WorkspaceVenvDir
}

$VenvPython = Join-Path $VenvDir "Scripts\\python.exe"
$ReqFile = Join-Path $RepoRoot "requirements-gui.txt"
$GuiEntry = Join-Path $RepoRoot "tools\\debuglink_gui.py"
$DepsStamp = Join-Path $VenvDir ".gui_deps_stamp"

Write-Host "[GUI] using venv: $VenvDir"

function Invoke-SetupVenv {
    if (Test-Path $VenvPython) {
        return
    }

    Write-Host "[GUI] creating venv: $VenvDir"
    $PyLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($null -ne $PyLauncher) {
        & py -3 -m venv $VenvDir
    } else {
        & python -m venv $VenvDir
    }
}

function Invoke-InstallDeps {
    Write-Host "[GUI] installing dependencies from requirements-gui.txt"
    & $VenvPython -m pip install -U pip
    if ($UseTuna) {
        & $VenvPython -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r $ReqFile
    } else {
        & $VenvPython -m pip install -r $ReqFile
    }
}

function Test-GuiDeps {
    try {
        & $VenvPython -c "import PySide6.QtCore, serial" 1>$null 2>$null
        return ($LASTEXITCODE -eq 0)
    } catch {
        return $false
    }
}

function Get-ReqFingerprint {
    return (Get-FileHash $ReqFile -Algorithm SHA256).Hash
}

function Test-FastDepsStamp {
    param(
        [string]$ExpectedFingerprint
    )

    if (-not (Test-Path $VenvPython)) {
        return $false
    }
    if (-not (Test-Path $DepsStamp)) {
        return $false
    }

    try {
        $stamp = (Get-Content $DepsStamp -Raw).Trim()
        return ($stamp -eq $ExpectedFingerprint)
    } catch {
        return $false
    }
}

Invoke-SetupVenv

$reqFingerprint = Get-ReqFingerprint
$needInstall = $false

if ($RecheckDeps) {
    if (-not (Test-GuiDeps)) {
        $needInstall = $true
    }
} else {
    if (-not (Test-FastDepsStamp -ExpectedFingerprint $reqFingerprint)) {
        if (-not (Test-GuiDeps)) {
            $needInstall = $true
        }
    }
}

if ($needInstall) {
    Invoke-InstallDeps
    if (-not (Test-GuiDeps)) {
        throw "[GUI] dependency check failed after install."
    }
}

try {
    Set-Content -Path $DepsStamp -Value $reqFingerprint -NoNewline -Encoding ASCII
} catch {
    Write-Host "[GUI] WARN: failed to update deps stamp: $($_.Exception.Message)"
}

$Args = @($GuiEntry, "--port", $Port, "--baud", "$Baud", "--rate", "$Rate")
if ($Mock) {
    $Args += "--mock"
}

Write-Host "[GUI] launch: $Port $Baud ${Rate}Hz$(if ($Mock) { ' (mock)' } else { '' })"
& $VenvPython @Args

