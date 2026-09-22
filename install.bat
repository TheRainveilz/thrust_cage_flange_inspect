@echo off
rem ====================================================================
rem  Windows 一键安装依赖 (双击运行即可)
rem  实际逻辑在 install.py: 优先 uv，无 uv 则退回 pip
rem ====================================================================
chcp 65001 >nul
cd /d "%~dp0"

rem 优先用 Windows Python launcher (py -3)，找不到再退回 python
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 install.py %*
) else (
    where python >nul 2>nul
    if %errorlevel%==0 (
        python install.py %*
    ) else (
        echo [ERROR] 未找到 Python。请先安装 Python 3.11 及以上并勾选 "Add to PATH"。
        pause
        exit /b 1
    )
)

echo.
pause
