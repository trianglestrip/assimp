@echo off
rem ============================================================
rem GPU visual acceptance: render the PBRT-v4 bistro cafe scene
rem through the assetpack DX12 viewer.
rem
rem Usage:   tools\run_bistro.bat [extra viewer args...]
rem          e.g. tools\run_bistro.bat --frames 300 --stat bistro
rem
rem First-time build (SDL2 expected under F:\project\third_party):
rem   cmake -G "Visual Studio 17 2022" -A x64 -DASSETPACK_WITH_VIEWER=ON ^
rem         -S "%~dp0.." -B "%~dp0..\build"
rem   cmake --build "%~dp0..\build" --config Release --target assetpack_viewer
rem
rem Note: bistro materials reference .exr textures which stb_image
rem cannot decode; meshes render with their diffuse colors instead.
rem ============================================================
setlocal
set "ROOT=%~dp0.."
set "VIEWER=%ROOT%\build\Release\assetpack_viewer.exe"
set "SCENE=D:\models\pbrt-v4-scenes\bistro\bistro_cafe.pbrt"

if not exist "%VIEWER%" (
    echo [run_bistro] viewer not found: %VIEWER%
    echo [run_bistro] build it first:
    echo   cmake -G "Visual Studio 17 2022" -A x64 -DASSETPACK_WITH_VIEWER=ON -S "%ROOT%" -B "%ROOT%\build"
    echo   cmake --build "%ROOT%\build" --config Release --target assetpack_viewer
    pause
    exit /b 1
)
if not exist "%SCENE%" (
    echo [run_bistro] scene not found: %SCENE%
    pause
    exit /b 1
)

"%VIEWER%" "%SCENE%" %*
echo [run_bistro] viewer exited with code %ERRORLEVEL%
pause
endlocal
