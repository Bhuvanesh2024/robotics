@echo off
title 3D 4-DOF Robot Arm Kinematics (Matplotlib 3D)
echo ============================================================
echo   Running 3D 4-DOF Robot Arm Kinematics (Matplotlib 3D)
echo ============================================================

if exist "%USERPROFILE%\anaconda3\python.exe" (
    "%USERPROFILE%\anaconda3\python.exe" kinematics_3d_cli.py
    goto end
)

where py >nul 2>nul
if %ERRORLEVEL% equ 0 (
    py kinematics_3d_cli.py
    goto end
)

python kinematics_3d_cli.py

:end
pause
