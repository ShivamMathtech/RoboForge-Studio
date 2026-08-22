#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python -m venv "$PROJECT_DIR/.venv"
source "$PROJECT_DIR/.venv/bin/activate"
python -m pip install -r "$PROJECT_DIR/backend/requirements.txt"
(cd "$PROJECT_DIR/backend" && python -m uvicorn app.main:app --reload --port 8000) &
BACKEND_PID=$!
trap 'kill "$BACKEND_PID" 2>/dev/null || true' EXIT
cd "$PROJECT_DIR/frontend"
npm install
npm run dev

