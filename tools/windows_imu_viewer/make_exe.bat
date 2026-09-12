@echo off
chcp 65001 >nul
setlocal EnableExtensions
rem ============================================================
rem  Сборка IMU viewer в один EXE (PyInstaller, Windows)
rem  Результат: dist\imu_viewer.exe
rem  Запуск без платы:  dist\imu_viewer.exe --sim
rem ============================================================

rem ---------- Поиск настоящего Python (не алиас Microsoft Store) ----------
set "PYQ="
where py >nul 2>nul
if not errorlevel 1 (
    py -3 -c "import sys;print('ok')" > "%TEMP%\pychk.txt" 2>nul
    findstr /b "ok" "%TEMP%\pychk.txt" >nul && set "PYQ=py -3"
)
if not defined PYQ (
    where python >nul 2>nul
    if not errorlevel 1 (
        python -c "import sys;print('ok')" > "%TEMP%\pychk.txt" 2>nul
        findstr /b "ok" "%TEMP%\pychk.txt" >nul && set "PYQ=python"
    )
)
if not defined PYQ (
    where python3 >nul 2>nul
    if not errorlevel 1 (
        python3 -c "import sys;print('ok')" > "%TEMP%\pychk.txt" 2>nul
        findstr /b "ok" "%TEMP%\pychk.txt" >nul && set "PYQ=python3"
    )
)
del "%TEMP%\pychk.txt" >nul 2>nul
rem типовые места установки (python.org ставит в %LOCALAPPDATA%)
if not defined PYQ for /d %%D in ("%LOCALAPPDATA%\Programs\Python\Python3*") do if exist "%%~D\python.exe" set "PYQ="%%~D\python.exe""
if not defined PYQ for /d %%D in ("C:\Python3*") do if exist "%%~D\python.exe" set "PYQ="%%~D\python.exe""

if not defined PYQ (
    echo Python НЕ найден.
    echo.
    echo Сделайте одно из:
    echo   1) Установите Python 3.9+ с https://www.python.org/downloads/
    echo      и при установке ОТМЕТЬТЕ галочку "Add Python to PATH".
    echo   2) Если сообщение "Python was not found; run without arguments
    echo      to install from the Microsoft Store" - это алиас Windows,
    echo      а не интерпретатор: "Параметры" - "Приложения" -
    echo      "Дополнительные параметры приложений" - "Псевдонимы выполнения
    echo      приложений" - выключите алиасы python / python3, затем пункт 1.
    echo.
    echo После установки Python запустите этот файл снова.
    pause
    exit /b 1
)
echo Python: %PYQ%
%PYQ% --version

echo Installing dependencies: pyserial, pyinstaller ...
%PYQ% -m pip install --quiet pyserial pyinstaller
if errorlevel 1 %PYQ% -m pip install --quiet --user pyserial pyinstaller
if errorlevel 1 (
    echo Could not install dependencies. Run manually:
    echo     %PYQ% -m pip install pyserial pyinstaller
    pause
    exit /b 1
)

cd /d "%~dp0"
echo Building EXE (1-2 min) ...
%PYQ% -m PyInstaller --onefile --noconfirm --clean --name imu_viewer --distpath dist --workpath build --specpath build imu_viewer.py
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
