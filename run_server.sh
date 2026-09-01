#!/usr/bin/env bash
# the pipeline as a service:  ./run_server.sh  then open http://127.0.0.1:8011
cd "$(dirname "$0")"
exec .venv/bin/python -m uvicorn app:app --app-dir server \
     --host "${HOST:-127.0.0.1}" --port "${PORT:-8011}" "$@"
