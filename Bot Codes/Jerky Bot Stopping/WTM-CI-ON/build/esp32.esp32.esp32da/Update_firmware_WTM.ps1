# Define source and destination directories
$sourceDir = $PSScriptRoot
$destinationDir = "C:\Users\User\OneDrive\Desktop\DebuggerLogs"

# Define the source file and the target file
$sourceFile = Get-ChildItem -Path $sourceDir -Filter "*.bin" | Select-Object -First 1
$targetFile = Join-Path -Path $destinationDir -ChildPath "WTMFirmware.bin"

# Check if a .bin file exists in the source directory
if ($null -eq $sourceFile) {
    Write-Host "No .bin file found in the source directory." -ForegroundColor Red
    exit
}

# If Firmware.bin exists, delete it; otherwise, proceed to copy
if (Test-Path -Path $targetFile) {
    Write-Host "Firmware.bin exists in the destination directory. Deleting it..."
    Remove-Item -Path $targetFile -Force
} else {
    Write-Host "No Firmware.bin exists in the destination directory. Proceeding to copy..."
}

# Copy and rename the new file
Write-Host "Copying and renaming the new file to WTMFirmware.bin..."
Copy-Item -Path $sourceFile.FullName -Destination $targetFile

Write-Host "Operation completed successfully!" -ForegroundColor Green

Read-Host "Press any key to exit"
