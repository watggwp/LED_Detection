@echo off
REM Start LED colorwatch bound to this machine's Tailscale IP, so phones/tablets
REM signed in to the same Tailscale account can open the web UI.
REM
REM Difference from run_windows.bat: that one is localhost-only. This one is
REM reachable from your tailnet but NOT from the LAN/Wi-Fi, so no Windows
REM Firewall change is needed (Tailscale's installer already allows its own
REM interface).
REM
REM Extra options pass through, e.g.:  run_tailscale.bat --thresh 6
REM
REM NOTE: kept ASCII-only on purpose. cmd.exe reads .bat files with the OEM
REM codepage and mis-tracks line boundaries when the file holds UTF-8 Thai,
REM which chops commands mid-word. All Thai text lives in webapp.py instead.
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
  echo [!] .venv not found. Run these once:
  echo       py -m venv .venv
  echo       .venv\Scripts\pip install -r requirements.txt
  pause
  exit /b 1
)

REM webapp.py --tailscale finds the Tailscale IP itself and prints the URL.
.venv\Scripts\python webapp.py --tailscale %*

echo.
echo [server stopped]
pause
