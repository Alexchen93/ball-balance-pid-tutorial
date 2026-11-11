@echo off
setlocal
cd /d "%~dp0"
set "PYTHON=%~dp0.venv-win\Scripts\python.exe"

if exist "%PYTHON%" goto check_environment
echo First run: preparing the Python environment for Windows.
echo An internet connection is required to install the packages.
echo.

py -3.12 -c "import sys" >nul 2>&1
if not errorlevel 1 (
  set "PYTHON_LAUNCHER=py -3.12"
  goto create_environment
)
py -3.11 -c "import sys" >nul 2>&1
if not errorlevel 1 (
  set "PYTHON_LAUNCHER=py -3.11"
  goto create_environment
)
py -3.10 -c "import sys" >nul 2>&1
if not errorlevel 1 (
  set "PYTHON_LAUNCHER=py -3.10"
  goto create_environment
)
py -3 -c "import sys; sys.exit(0 if sys.version_info[:2] >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 (
  set "PYTHON_LAUNCHER=py -3"
  goto create_environment
)
python -c "import sys; sys.exit(0 if sys.version_info[:2] >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 (
  set "PYTHON_LAUNCHER=python"
  goto create_environment
)

echo Python 3.10 or newer was not found by the Python launcher or PATH.
echo Install Python 3.12 for Windows, including the Python launcher,
echo and select "Add python.exe to PATH" during installation.
echo Download: https://www.python.org/downloads/windows/
echo After installation, close this window and double-click this file again.
goto error_exit

:create_environment
%PYTHON_LAUNCHER% -m venv "%~dp0.venv-win"
if errorlevel 1 (
  echo Failed to create the Python environment. See the error above.
  goto error_exit
)

:check_environment
"%PYTHON%" -c "import sys" >nul 2>&1
if errorlevel 1 (
  echo The Windows environment is broken or was copied from another computer.
  echo Delete the .venv-win folder, then double-click this file again.
  goto error_exit
)
"%PYTHON%" -c "import cv2, numpy, serial" >nul 2>&1
if not errorlevel 1 goto start_app
echo Installing required packages. This may take a few minutes.
"%PYTHON%" -m pip install -r "%~dp0requirements.txt" > "%~dp0.venv-win\install.log" 2>&1
if errorlevel 1 (
  echo Package installation failed. Check your internet connection.
  echo Details: .venv-win\install.log
  echo If this Python version is unsupported, install Python 3.12,
  echo delete .venv-win, and double-click this file again.
  goto error_exit
)
"%PYTHON%" -c "import cv2, numpy, serial" >nul 2>&1
if errorlevel 1 (
  echo The required packages could not be loaded. Check .venv-win\install.log.
  goto error_exit
)

:start_app
echo Starting Camera Vision and searching for Arduino Nano.
echo The READY setup screen can open without a Nano connection.
"%PYTHON%" "%~dp0ball_vision.py" %*
if errorlevel 1 goto error_exit
exit /b 0

:error_exit
echo.
echo Camera Vision could not start. Please keep this window for troubleshooting.
echo Press any key to exit.
pause >nul
exit /b 1
