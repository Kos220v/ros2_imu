@echo off
chcp 65001 >nul
setlocal EnableExtensions
rem ============================================================
rem  IMU viewer - запуск тестового приложения (Windows)
rem  Python 3.9+ (tkinter входит в стандартный инсталлятор)
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
if not defined PYQ for /d %%D in ("%LOCALAPPDATA%\Programs\Python\Python3*") do if exist "%%~D\python.exe" set "PYQ="%%~D\python.exe""
if not defined PYQ for /d %%D in ("C:\Python3*") do if exist "%%~D\python.exe" set "PYQ="%%~D\python.exe""

if not defined PYQ (
    echo Python НЕ найден. Установите Python 3.9+ с
    echo https://www.python.org/downloads/ и отметьте "Add Python to PATH".
    pause
    exit /b 1
)
cd /d "%~dp0"
%PYQ% -c "import serial" >nul 2>nul
if errorlevel 1 (
    echo Установка pyserial...
    %PYQ% -m pip install --quiet pyserial
    if errorlevel 1 %PYQ% -m pip install --quiet --user pyserial
    if errorlevel 1 (
        echo Не удалось установить pyserial. Выполните вручную:
        echo     %PYQ% -m pip install pyserial
        pause
        exit /b 1
    )
)
%PYQ% "%~dp0imu_viewer.py" %*
pause
