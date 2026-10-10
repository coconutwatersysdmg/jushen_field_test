@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "PYTHONNOUSERSITE=1"
set "PYTHONUTF8=1"

set "PYTHON=%~dp0.venv\Scripts\python.exe"
if not exist "%PYTHON%" set "PYTHON=%~dp0runtime\python.exe"

echo executable=%PYTHON%
if not exist "%PYTHON%" (
  echo [ERROR] Local Python not found. Run 安装环境.bat first.
  pause
  exit /b 1
)

"%PYTHON%" -c "import sys; print(sys.version); print(sys.executable)"
"%PYTHON%" "%~dp0main.py" %*
echo exit=%ERRORLEVEL%
pause
endlocal
