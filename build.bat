@echo off
REM Headless build for Stratosonde firmware - thin wrapper around build.ps1
REM Usage: build.bat [-Clean] [-Jobs N]
powershell -ExecutionPolicy Bypass -File "%~dp0build.ps1" %*
exit /b %ERRORLEVEL%