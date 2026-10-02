#!/usr/bin/env python3
import compileall
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/"ros2_ws/src/we_meet_vio"),str(ROOT/"tools"),str(ROOT/"tests")]
if not compileall.compile_dir(ROOT/"ros2_ws/src/we_meet_vio",quiet=1):
    raise SystemExit(1)
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.discover(str(ROOT/"tests")))
raise SystemExit(0 if result.wasSuccessful() else 1)
