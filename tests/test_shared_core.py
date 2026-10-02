"""Catch drift between the shipped core and its declared cross-branch identity."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from verify_shared_core import verify


class SharedCoreTests(unittest.TestCase):
    def test_core_matches_declared_source_manifest(self):
        verify(Path(__file__).resolve().parents[1])
