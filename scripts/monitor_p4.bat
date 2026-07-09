@echo off
setlocal
set "IDF_PATH=D:\esp-idf\Espressif\v5.5.4\esp-idf"
set "IDF_TOOLS_PATH=C:\Espressif\tools"
set "IDF_PYTHON_ENV_PATH=C:\Espressif\tools\python_env\idf5.5_py3.11_env"
set "PATH=C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121\riscv32-esp-elf\bin;C:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts;%PATH%"
set "ESP_IDF_VERSION=5.5.4"
set "MSYSTEM="
cd /d "%~dp0.."
"C:\Espressif\tools\python_env\idf5.5_py3.11_env\Scripts\python.exe" -u "%IDF_PATH%\tools\idf.py" -p COM4 monitor
