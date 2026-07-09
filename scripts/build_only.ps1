$env:IDF_TOOLS_PATH = "C:\Espressif\tools"
$env:IDF_PATH = "D:\esp-idf\Espressif\v5.5.4\esp-idf"
$env:IDF_PYTHON_ENV_PATH = "C:\Espressif\tools\python\v5.5.4\venv"
$env:IDF_PYTHON_CHECK_CONSTRAINTS = "0"
$env:MSYSTEM = $null
$env:MSYS = $null

# Change to the project root (parent of this script's directory).
cd "$PSScriptRoot/.."

# Load ESP-IDF environment (PATH, git, cmake, ninja, etc.).
. "D:\esp-idf\Espressif\v5.5.4\esp-idf\export.ps1" | Out-Null

& "C:\Espressif\tools\python\v5.5.4\venv\Scripts\python.exe" `
  "D:\esp-idf\Espressif\v5.5.4\esp-idf\tools\idf.py" build
