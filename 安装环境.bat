@echo off
setlocal EnableExtensions
cd /d "%~dp0"

echo ========================================
echo  jushen_field_test - create local .venv
echo ========================================
echo.

set "VENV_PY=%~dp0.venv\Scripts\python.exe"
if exist "%VENV_PY%" (
  echo [OK] .venv already exists
  goto :INSTALL
)

set "BASE_PY="
where py >nul 2>&1
if not errorlevel 1 (
  for /f "delims=" %%P in ('py -3.12 -c "import sys; print(sys.executable)" 2^>nul') do set "BASE_PY=%%P"
)
if not defined BASE_PY (
  for /f "delims=" %%P in ('py -3 -c "import sys; print(sys.executable)" 2^>nul') do set "BASE_PY=%%P"
)
if not defined BASE_PY if exist "D:\ProgramData\anaconda3\python.exe" set "BASE_PY=D:\ProgramData\anaconda3\python.exe"
if not defined BASE_PY if exist "%LocalAppData%\Programs\Python\Python312\python.exe" set "BASE_PY=%LocalAppData%\Programs\Python\Python312\python.exe"
if not defined BASE_PY (
  where python >nul 2>&1
  if not errorlevel 1 (
    for /f "delims=" %%P in ('where python') do (
      set "BASE_PY=%%P"
      goto :HAVE_BASE
    )
  )
)
:HAVE_BASE

if not defined BASE_PY (
  echo [ERROR] No Python found to create .venv
  echo Install Python 3.10+ / 3.12, then run this bat again.
  pause
  exit /b 1
)

echo [INFO] base python: %BASE_PY%
"%BASE_PY%" -m venv "%~dp0.venv"
if errorlevel 1 (
  echo [ERROR] venv create failed
  pause
  exit /b 1
)

:INSTALL
echo [INFO] pip install -r requirements.txt
"%VENV_PY%" -m pip install --upgrade pip
"%VENV_PY%" -m pip install -r "%~dp0requirements.txt"
if errorlevel 1 (
  echo [ERROR] pip install failed
  pause
  exit /b 1
)

echo.
echo [OK] Local env ready: .venv\Scripts\python.exe
echo Next: double-click 启动软件.bat
pause
endlocal
exit /b 0
