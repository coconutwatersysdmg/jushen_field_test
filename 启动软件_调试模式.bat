@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "PYTHONNOUSERSITE=1"
set "PYTHONUTF8=1"

REM 现场大字号主题：一启动就能看清，别再用老的小字界面
set "JUSHEN_UI_THEME=field"

set "PYTHON=%~dp0.venv\Scripts\python.exe"
if not exist "%PYTHON%" set "PYTHON=%~dp0runtime\python.exe"

echo executable=%PYTHON%
if not exist "%PYTHON%" (
  echo [ERROR] Local Python not found. Run 安装环境.bat first.
  pause
  exit /b 1
)

"%PYTHON%" -c "import sys; print(sys.version); print(sys.executable)"
"%PYTHON%" -u "%~dp0main.py" %*
echo exit=%ERRORLEVEL%
pause
endlocal
