#!/bin/zsh
set -e
cd -- "${0:A:h}/.."
exec .venv-reconstruction/bin/python rover.py start --open
