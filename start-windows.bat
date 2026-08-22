@echo off
setlocal
cd /d "%~dp0"
if not exist .venv py -m venv .venv
call .venv\Scripts\activate.bat
python -m pip install -r backend\requirements.txt
start "RoboForge API" cmd /k "call .venv\Scripts\activate.bat && cd backend && python -m uvicorn app.main:app --reload --port 8000"
cd frontend
call npm install
call npm run dev
