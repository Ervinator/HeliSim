"""Scratch: every line over 79 columns in the files this change touched.

    python attic/_cols.py
"""

import io
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FILES = ("controls.py", "main.py", "default_controls.xml",
         "README.md", "TODO.md")

for name in FILES:
    path = os.path.join(ROOT, name)
    with io.open(path, encoding="utf-8") as handle:
        for number, line in enumerate(handle, 1):
            line = line.rstrip()
            if len(line) > 79:
                print("%s:%d:%d:%s" % (name, number, len(line), line))
print("done")
