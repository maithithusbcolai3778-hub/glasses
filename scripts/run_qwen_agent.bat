@echo off
setlocal
set "PYTHONIOENCODING=utf-8"

:: 把你的 DashScope API Key 填到这里，或者提前在系统环境变量里设置 DASHSCOPE_API_KEY
:: set "DASHSCOPE_API_KEY=sk-xxxxxxxx"

cd /d "%~dp0.."
conda run --no-capture-output -n yolov8_rtx python "pc_agent\qwen_agent.py" --status --interval 6 %*
endlocal
