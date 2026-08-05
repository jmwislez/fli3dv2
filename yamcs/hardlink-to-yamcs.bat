@echo off
setlocal enabledelayedexpansion

:: ==========================================
:: CONFIGURATION SECTION
:: ==========================================
:: Change these paths to your actual source and destination folders.
:: Keep the trailing slashes!
set "SOURCE_DIR=C:\Users\jmw\Documents\PlatformIO\Projects\fli3dv2\yamcs\"
set "DEST_DIR=C:\Users\jmw\Documents\yamcs-5.13.2\"

:: List the filenames you want to link, separated by spaces.
set "FILE_LIST=bin\yamcs_serial_frontend.cmd bin\yamcs_serial_frontend.py etc\processor.yaml etc\yamcs.yaml etc\yamcs.fli3d.yaml mdb\xtce.fli3d.xml"
:: ==========================================

title Hard Link Installer Script

:: 1. CHECK FOR ADMINISTRATOR PRIVILEGES
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo [ERROR] This script MUST be run as an Administrator.
    echo Please right-click this batch file and select "Run as administrator".
    goto :END
)

:: 2. CHECK IF CONFIGURATION WAS UPDATED
if "%SOURCE_DIR%"=="C:\CHANGE_ME\SOURCE_FOLDER\" (
    echo [WARNING] Source directory has not been configured yet.
    echo Please edit this .bat file and update the SOURCE_DIR variable.
    goto :END
)
if "%DEST_DIR%"=="C:\CHANGE_ME\DEST_FOLDER\" (
    echo [WARNING] Destination directory has not been configured yet.
    echo Please edit this .bat file and update the DEST_DIR variable.
    goto :END
)

:: 3. VERIFY SOURCE DIRECTORY EXISTS
if not exist "%SOURCE_DIR%" (
    echo [ERROR] The configured source directory does not exist:
    echo %SOURCE_DIR%
    goto :END
)

:: 4. ENSURE DESTINATION DIRECTORY EXISTS
if not exist "%DEST_DIR%" (
    echo Destination directory does not exist. Creating it now...
    mkdir "%DEST_DIR%"
)

echo [INFO] Configuration verified. Starting link process...
echo --------------------------------------------------

:: 5. LOOP AND LINK FILES
for %%F in (%FILE_LIST%) do (
    set "SRC_FILE=%SOURCE_DIR%%%F"
    set "DST_FILE=%DEST_DIR%%%F"
    
    if exist "!SRC_FILE!" (
        if exist "!DST_FILE!" (
            echo [SKIP] Link or file already exists at destination: %%F
        ) else (
            mklink /H "!DST_FILE!" "!SRC_FILE!"
        )
    ) else (
        echo [ERROR] Source file missing, cannot link: %%F
    )
)

echo --------------------------------------------------
echo [SUCCESS] Script finished processing links.

:END
echo.
pause
