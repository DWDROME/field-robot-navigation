@echo off
setlocal

if defined OPENOCD_EXE if not exist "%OPENOCD_EXE%" (
  echo [ERROR] OPENOCD_EXE points to a missing file:
  echo         %OPENOCD_EXE%
  exit /b 1
)

if defined OPENOCD_SCRIPTS if not exist "%OPENOCD_SCRIPTS%" (
  echo [ERROR] OPENOCD_SCRIPTS points to a missing directory:
  echo         %OPENOCD_SCRIPTS%
  exit /b 1
)

if not defined OPENOCD_EXE call :find_openocd_exe
if not defined OPENOCD_SCRIPTS call :find_openocd_scripts

if not defined OPENOCD_EXE (
  echo [ERROR] Could not find openocd.exe automatically.
  echo         Set OPENOCD_EXE before running this script.
  exit /b 1
)

if not defined OPENOCD_SCRIPTS (
  echo [ERROR] Could not find OpenOCD st_scripts automatically.
  echo         Set OPENOCD_SCRIPTS before running this script.
  exit /b 1
)

echo OPENOCD_EXE=%OPENOCD_EXE%
echo OPENOCD_SCRIPTS=%OPENOCD_SCRIPTS%

endlocal & (
  set "OPENOCD_EXE=%OPENOCD_EXE%"
  set "OPENOCD_SCRIPTS=%OPENOCD_SCRIPTS%"
)
exit /b 0

:find_openocd_exe
for /f "delims=" %%F in ('where openocd.exe 2^>nul') do (
  if not defined OPENOCD_EXE set "OPENOCD_EXE=%%~fF"
)

if defined OPENOCD_EXE goto :eof

for /d %%D in (D:\ST\STM32CubeIDE_* C:\ST\STM32CubeIDE_*) do (
  call :probe_openocd_exe "%%~fD"
  if defined OPENOCD_EXE goto :eof
)
goto :eof

:probe_openocd_exe
for /d %%F in ("%~1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.openocd.win32_*") do (
  if exist "%%~fF\tools\bin\openocd.exe" (
    set "OPENOCD_EXE=%%~fF\tools\bin\openocd.exe"
    goto :eof
  )
)
goto :eof

:find_openocd_scripts
for /d %%D in (D:\ST\STM32CubeIDE_* C:\ST\STM32CubeIDE_*) do (
  call :probe_openocd_scripts "%%~fD"
  if defined OPENOCD_SCRIPTS goto :eof
)
goto :eof

:probe_openocd_scripts
for /d %%F in ("%~1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.debug.openocd_*") do (
  if exist "%%~fF\resources\openocd\st_scripts" (
    set "OPENOCD_SCRIPTS=%%~fF\resources\openocd\st_scripts"
    goto :eof
  )
)
goto :eof
