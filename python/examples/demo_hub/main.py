from pathlib import Path
import sys

# Ensure the demo_hub package (under python/) is importable when running as an example.
REPO_PYTHON = Path(__file__).resolve().parents[2] / "python"
if str(REPO_PYTHON) not in sys.path:
    sys.path.insert(0, str(REPO_PYTHON))

from demo_hub.app import main


if __name__ == "__main__":
    main()
