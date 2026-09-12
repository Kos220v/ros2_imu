@echo off
rem ============================================================
rem  IMU viewer - запуск тестового приложения (Windows)
rem  Python 3.9+ (tkinter входит в стандартный инсталлятор)
rem ============================================================
where python >nul 2>nul
if errorlevel 1 (
    echo Python не найден. Установите Python 3.9+ с python.org
    echo и отметьте "Add Python to PATH" при установке.
    pause
    exit /b 1
)
python -c "import serial" >nul 2>nul
if errorlevel 1 (
    echo Установка pyserial...
    python -m pip install --quiet pyserial
    if errorlevel 1 (
        echo Не удалось установить pyserial. Выполните вручную:
        echo     python -m pip install pyserial
        pause
        exit /b 1
    )
)
python "%~dp0imu_viewer.py"
pause
