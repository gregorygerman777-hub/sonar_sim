"""Where a run writes its results.

Demos write through here instead of hard-coding `output/`, for two reasons. A
fresh clone has no `output/` directory, and every demo used to die on
FileNotFoundError at the point it tried to save. And a run can now be pointed at
its own directory by setting SONAR_OUTPUT_DIR, which is how run_all.sh keeps each
run's figures instead of overwriting the last one's.
"""

import os


def output_path(name):
    directory = os.environ.get("SONAR_OUTPUT_DIR", "output")
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory, name)
