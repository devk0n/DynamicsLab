# Description:
# Gathers all *.cpp and *.h files from source and include folders
# and includes the top-level CMakeLists.txt.
# Outputs everything to Combined.txt.

# Get .cpp and .h files from subdirectories and store in a variable
$allFiles = Get-ChildItem -Path ".\source\physics", ".\include\physics" -Recurse -Include *.cpp, *.h -File

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