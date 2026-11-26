"""Keep transient test files outside the source workspace."""
import tempfile


def temporary_directory(prefix):
    return tempfile.TemporaryDirectory(prefix="motionarm-" + prefix)
