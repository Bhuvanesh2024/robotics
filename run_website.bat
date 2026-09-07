@echo off
title 4-DOF Robot Arm Kinematics Visualizer
echo ============================================================
echo   Launching 4-DOF Robot Arm Kinematics Visualizer Website
echo ============================================================
echo Starting local web server...

where python >nul 2>nul
if %ERRORLEVEL% equ 0 (
    python server.py
    goto end
)

where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    py server.py
    goto end
)

if exist "%USERPROFILE%\anaconda3\python.exe" (
    "%USERPROFILE%\anaconda3\python.exe" server.py
    goto end
)

echo Opening index.html directly in browser...
start index.html

:end
pause
