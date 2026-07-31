@echo off
REM ดับเบิลคลิกเพื่อเปิดโปรแกรมตรวจสีไฟ LED บน Windows
REM (ติดตั้งครั้งแรกก่อน: py -m venv .venv  แล้ว  .venv\Scripts\pip install -r requirements.txt)
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo ยังไม่ได้ติดตั้ง .venv — รันก่อน:
  echo    py -m venv .venv
  echo    .venv\Scripts\pip install -r requirements.txt
  pause
  exit /b 1
)
start "" http://localhost:8000
.venv\Scripts\python webapp.py
pause
