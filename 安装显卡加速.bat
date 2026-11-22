@echo off
setlocal EnableExtensions
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PYTHONNOUSERSITE=1"
set "PYTHON=%~dp0.venv\Scripts\python.exe"
if not exist "%PYTHON%" (
  echo [ERROR] Run the environment installer first.
  pause
  exit /b 1
)
echo Close the application before installing the GPU runtime.
echo Installing the official PyTorch CUDA 13.0 runtime...
"%PYTHON%" -m pip install --no-cache-dir -r "%~dp0requirements-gpu.txt"
if errorlevel 1 (
  echo [ERROR] GPU runtime installation failed.
  pause
  exit /b 1
)
"%PYTHON%" -B -c "import torch; assert torch.cuda.is_available(), 'CUDA GPU unavailable'; x=torch.ones((32,32),device='cuda'); assert (x@x).sum().item()==32768; print('[OK]',torch.__version__,torch.cuda.get_device_name(0))"
if errorlevel 1 (
  echo [ERROR] GPU verification failed. Check the NVIDIA driver.
  pause
  exit /b 1
)
echo [OK] Restart the application to use GPU inference automatically.
pause
endlocal
