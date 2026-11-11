@echo off
setlocal
chcp 65001 >nul
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
cd /d "%~dp0"
set "PYTHON=%~dp0.venv-win\Scripts\python.exe"

if exist "%PYTHON%" goto check_environment
echo 第一次啟動：正在建立 Windows 的 Python 執行環境……
echo 首次安裝需要連接網路，請稍候。
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
python -c "import sys; sys.exit(0 if (3, 10) <= sys.version_info[:2] <= (3, 12) else 1)" >nul 2>&1
if not errorlevel 1 (
  set "PYTHON_LAUNCHER=python"
  goto create_environment
)

echo 找不到 Python 3.10、3.11 或 3.12。
echo 請先安裝 Windows 版 Python，並勾選安裝 Python Launcher，然後再雙擊此檔案。
echo 下載網址：https://www.python.org/downloads/windows/
goto error_exit

:create_environment
%PYTHON_LAUNCHER% -m venv "%~dp0.venv-win"
if errorlevel 1 (
  echo 無法建立 Python 執行環境。
  goto error_exit
)

:check_environment
"%PYTHON%" -c "import sys" >nul 2>&1
if errorlevel 1 (
  echo Windows 執行環境損壞，或是從另一台電腦複製而來。
  echo 請刪除 .venv-win 資料夾，再雙擊此啟動檔。
  goto error_exit
)
"%PYTHON%" -c "import cv2, numpy, serial" >nul 2>&1
if not errorlevel 1 goto start_app
echo 正在安裝必要套件，可能需要幾分鐘，請勿關閉視窗……
"%PYTHON%" -m pip install -r "%~dp0requirements.txt" > "%~dp0.venv-win\install.log" 2>&1
if errorlevel 1 (
  echo 套件安裝失敗，請檢查網路連線。
  echo 詳細資訊儲存在 .venv-win\install.log，可提供給協助排除問題的人。
  goto error_exit
)
"%PYTHON%" -c "import cv2, numpy, serial" >nul 2>&1
if errorlevel 1 (
  echo 必要套件安裝後仍無法載入，請檢查 .venv-win\install.log。
  goto error_exit
)

:start_app
echo 正在開啟攝影機程式，並自動搜尋 Arduino Nano。
echo 若未連接 Nano，仍可進入 READY 設定畫面。
"%PYTHON%" "%~dp0ball_vision.py" %*
if errorlevel 1 goto error_exit
exit /b 0

:error_exit
echo.
echo 程式無法啟動，請保留以上訊息以便排除問題。
echo 請按任意鍵結束。
pause >nul
exit /b 1
