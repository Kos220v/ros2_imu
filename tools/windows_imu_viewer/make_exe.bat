@echo off
chcp 65001 >nul
setlocal
rem ============================================================
rem  Сборка IMU viewer в один EXE (PyInstaller, Windows)
rem  Результат: dist\imu_viewer.exe
rem  Запуск без платы:  dist\imu_viewer.exe --sim
rem ============================================================
where python >nul 2>nul
if errorlevel 1 (
    echo Python not found. Install Python 3.9+ from python.org
    echo and tick "Add Python to PATH" during installation.
    pause
    exit /b 1
)
echo Installing dependencies: pyserial, pyinstaller ...
python -m pip install --quiet pyserial pyinstaller
if errorlevel 1 (
    echo Could not install dependencies. Run manually:
    echo     python -m pip install pyserial pyinstaller
    pause
    exit /b 1
)
cd /d "%~dp0"
echo Building EXE (1-2 min) ...
python -m PyInstaller --onefile --noconfirm --clean --name imu_viewer --distpath dist --workpath build --specpath build imu_viewer.py
if errorlevel 1 (
    echo Build FAILED. See messages above.
    pause
    exit /b 1
)
echo.
echo Done: dist\imu_viewer.exe
echo   real board : dist\imu_viewer.exe
echo   simulation : dist\imu_viewer.exe --sim
pause
endlocal
