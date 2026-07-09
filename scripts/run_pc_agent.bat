@echo off
setlocal
set "PYTHONIOENCODING=utf-8"
cd /d "%~dp0.."
conda run --no-capture-output -n yolov8_rtx python "pc_agent\pc_agent.py" %*
endlocal
