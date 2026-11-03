"""Regenerate demo figures from recorded data, without executing a controller."""
from pathlib import Path
import argparse
from hocbf.single_obstacle import render

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path('evidence/single_obstacle'))
    parser.add_argument('--no-animation', action='store_true')
    args = parser.parse_args()
    render(args.data, animate=not args.no_animation)
