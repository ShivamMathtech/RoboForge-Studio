.PHONY: install dev test backend frontend

install:
	cd backend && python -m pip install -r requirements.txt
	cd frontend && npm install

backend:
	cd backend && uvicorn app.main:app --reload --port 8000

frontend:
	cd frontend && npm run dev

test:
	cd backend && pytest -q
	cd frontend && npm run typecheck

