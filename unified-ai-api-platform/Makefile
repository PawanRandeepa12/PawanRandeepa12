.PHONY: install dev run test lint format docker-build docker-up docker-down

install:
	pip install -r requirements-dev.txt

dev:
	uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

run:
	uvicorn app.main:app --host 0.0.0.0 --port 8000

test:
	pytest

lint:
	ruff check app tests

format:
	ruff format app tests

docker-build:
	docker build -t unified-ai-api-platform .

docker-up:
	docker compose up --build

docker-down:
	docker compose down
