"""Keep transient test files in the repository's ignored artifact directory."""
import tempfile
from pathlib import Path


def temporary_directory(prefix):
    root = Path(__file__).resolve().parents[2] / "artifacts" / "test-temp"
    root.mkdir(parents=True, exist_ok=True)
    return tempfile.TemporaryDirectory(prefix=prefix, dir=root)
