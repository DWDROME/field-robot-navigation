@echo off
setlocal

set "TR800_ROOT=%~dp0..\.."
set "IMAGE=%~1"

if not defined IMAGE set "IMAGE=%TR800_ROOT%\build\tr800-st-preset-win\dlrobot.elf"

if not exist "%IMAGE%" (
  echo [ERROR] Firmware image not found:
  echo         %IMAGE%
  exit /b 1
)

call "%~dp0resolve_openocd_env.bat"
if errorlevel 1 exit /b 1

set "IMAGE_TCL=%IMAGE:\=/%"

"%OPENOCD_EXE%" -s "%OPENOCD_SCRIPTS%" -f "%TR800_ROOT%\openocd\tr800_daplink.cfg" -c "program {%IMAGE_TCL%} verify reset exit"

endlocal
