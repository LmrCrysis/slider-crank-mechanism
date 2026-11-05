@echo off
setlocal
cd /d "%~dp0"
title Slider-Crank Simulation GUI

:: Check for python in PATH
where python >nul 2>nul
if %ERRORLEVEL% equ 0 (
    python gui/app.py
    goto end
)

:: Check for py launcher
where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    py gui/app.py
    goto end
)

:: Fallback to installed Unreal Engine Python
set "UE_PYTHON=C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\ThirdParty\Python3\Win64\python.exe"
if exist "%UE_PYTHON%" (
    "%UE_PYTHON%" gui/app.py
    goto end
)

echo [ERROR] Python not found on system.
pause

:end
endlocal
