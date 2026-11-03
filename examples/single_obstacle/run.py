"""Run all four straight-ahead single-obstacle demonstrations."""
import sys
from hocbf.__main__ import main

if __name__ == '__main__':
    raise SystemExit(main(['demo', *sys.argv[1:]]))
