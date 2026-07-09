$env:MSYSTEM = $null
$env:MSYS = $null
$env:MSYS2_PATH_TYPE = $null

. 'C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1'

# Change to the project root (parent of this script's directory).
cd "$PSScriptRoot/.."

$python = Join-Path $env:IDF_PYTHON_ENV_PATH 'Scripts\python.exe'
$idf    = Join-Path $env:IDF_PATH 'tools\idf.py'

& $python $idf build flash -p COM4 -b 460800
