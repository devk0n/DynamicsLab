# Description:
# Gathers all *.cpp and *.h files from source and include folders
# and includes the top-level CMakeLists.txt.
# Outputs everything to Combined.txt.

# Get .cpp and .h files from subdirectories
$codeFiles = Get-ChildItem -Path ".\source", ".\include" -Recurse -Include *.cpp, *.h -File

# Get the root-level CMakeLists.txt
$cmakeFilePath = ".\CMakeLists.txt"
if (Test-Path $cmakeFilePath) {
    $cmakeFile = Get-Item $cmakeFilePath
    $allFiles = $codeFiles + $cmakeFile
} else {
    Write-Warning "CMakeLists.txt not found at $cmakeFilePath"
    $allFiles = $codeFiles
}

# Clear previous output
$outputFile = "Combined.txt"
if (Test-Path $outputFile) {
    Remove-Item $outputFile
}

# Output each file with a header
foreach ($file in $allFiles) {
    Add-Content -Path $outputFile -Value ("`n// -------- " + $file.FullName + " --------`n")
    Get-Content $file.FullName | Add-Content -Path $outputFile
}
