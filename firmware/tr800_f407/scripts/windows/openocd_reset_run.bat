@echo off
setlocal

set "TR800_ROOT=%~dp0..\.."

call "%~dp0resolve_openocd_env.bat"
if errorlevel 1 exit /b 1

"%OPENOCD_EXE%" -s "%OPENOCD_SCRIPTS%" -f "%TR800_ROOT%\openocd\tr800_daplink.cfg" -c "init" -c "reset run" -c "shutdown"

endlocal
