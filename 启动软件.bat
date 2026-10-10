@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "PYTHONNOUSERSITE=1"
set "PYTHONUTF8=1"

REM Prefer project-local env only (standalone; no sibling project).
set "PYTHON=%~dp0.venv\Scripts\python.exe"
set "PYTHONW=%~dp0.venv\Scripts\pythonw.exe"
if not exist "%PYTHON%" set "PYTHON=%~dp0runtime\python.exe"
if not exist "%PYTHONW%" if exist "%~dp0runtime\pythonw.exe" set "PYTHONW=%~dp0runtime\pythonw.exe"

if not exist "%PYTHON%" (
  echo [ERROR] Local Python not found.
  echo Run 安装环境.bat once to create .venv
  pause
  exit /b 1
)

if exist "%PYTHONW%" (
  start "" "%PYTHONW%" "%~dp0main.py" %*
) else (
  start "" "%PYTHON%" "%~dp0main.py" %*
)
endlocal
exit /b 0
