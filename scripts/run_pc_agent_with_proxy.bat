@echo off
setlocal
set "PYTHONIOENCODING=utf-8"

:: CyberClash 已启动，HTTP 代理端口为 7892
set "EDGE_TTS_PROXY=http://127.0.0.1:7892"

cd /d "%~dp0.."
conda run --no-capture-output -n yolov8_rtx python "pc_agent\pc_agent.py" --status --interval 5
endlocal
