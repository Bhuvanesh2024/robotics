@echo off
title 4-DOF Robot Arm Kinematics CLI (Matplotlib)
echo ============================================================
echo   Running 4-DOF Robot Arm Kinematics CLI (Matplotlib)
echo ============================================================

if exist "%USERPROFILE%\anaconda3\python.exe" (
    "%USERPROFILE%\anaconda3\python.exe" kinematics_cli.py
    goto end
)

where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    py kinematics_cli.py
    goto end
)

python kinematics_cli.py

:end
pause
