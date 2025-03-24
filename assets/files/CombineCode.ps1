# Filename: CombineCode.ps1

# Description:
#  Gathers all *.cpp and *.h files from the src, include, and assets folders
#  (including any subfolders), and writes them to combined_code.txt.

# Make sure you run this script from the project's root folder (where src, include, and assets are located).
# Usage:
#   1) Right-click this file and select "Run with PowerShell" (may need to unblock the script).
#   2) Or open PowerShell in the project's folder and run:
#      .\CombineCode.ps1

Get-ChildItem -Path .\source, .\include -Recurse -Include *.cpp, *.h -File |
    Get-Content |
    Out-File Combined.txt
