#!/bin/sh
set -eu
nav_root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
nav_python=${NAV_PYTHON:-"$nav_root/../../isaacprotocol/simulation/venv/bin/python"}
if [ ! -x "$nav_python" ]; then
    echo "Set NAV_PYTHON to a Python with the dependencies in auto-nav/requirements.txt." >&2
    exit 1
fi
export PYTHONPATH="$nav_root/.deps:$nav_root${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR="${TMPDIR:-/tmp}/mark-rover-nav-matplotlib"
action=${1:-}
if [ "$#" -gt 0 ]; then shift; fi
case "$action" in
    simulate) exec "$nav_python" -m rover_nav.mission "$@" ;;
    test) exec "$nav_python" -m pytest "$nav_root/rover_nav/test_navigation.py" "$nav_root/tests" "$@" ;;
    *) echo "Usage: auto-nav/run.sh {simulate|test} [options]" >&2; exit 2 ;;
esac
