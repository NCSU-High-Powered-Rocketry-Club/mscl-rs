from pathlib import Path

import pytest


@pytest.fixture
def mock_dataset_path() -> Path:
    return Path("datasets/500hz_10secs.bin")
