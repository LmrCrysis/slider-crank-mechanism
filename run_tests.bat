@echo off
setlocal
cd /d "%~dp0"
title Slider-Crank Validation Test Suite

set "PY_CMD=python"
where python >nul 2>nul
if %ERRORLEVEL% neq 0 (
    set "PY_CMD=C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\ThirdParty\Python3\Win64\python.exe"
)

"%PY_CMD%" -m pytest tests -v
pause
endlocal
