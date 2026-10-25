#!/bin/zsh
cd -- "${0:A:h}"
exec python3 rover_control/serve.py "$@"
