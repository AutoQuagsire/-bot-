@echo off
setlocal

set "REPO_ROOT=%~dp0"
for %%I in ("%REPO_ROOT%\..\..") do set "WORKSPACE_ROOT=%%~fI"
set "VENV_PY=%WORKSPACE_ROOT%\.venv-gui\Scripts\python.exe"
set "GUI_ENTRY=%REPO_ROOT%tools\debuglink_gui.py"

if not exist "%VENV_PY%" (
  echo [GUI] missing venv python: "%VENV_PY%"
  echo [GUI] first run setup with:
  echo   powershell -ExecutionPolicy Bypass -File "%REPO_ROOT%run_gui.ps1" -UseTuna
  exit /b 1
)

"%VENV_PY%" "%GUI_ENTRY%" %*
exit /b %errorlevel%

