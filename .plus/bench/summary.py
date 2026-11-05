import re
import sys

scope = re.compile(r"\[plus-timing\] (.+?) ms: (\d+) visited: (\d+)")
event = re.compile(r"\[plus-timing\] (.+)$")

run = 0
for line in open(sys.argv[1], errors="replace"):
    m = scope.search(line)
    if m:
        name, ms, visited = m.group(1), int(m.group(2)), int(m.group(3))
        indent = 0 if name == "route total" else 2 if name.startswith("subroute") or name == "redress route" else 4
        if name.startswith("stage 2 candidate ") and not name.startswith("stage 2 candidate scoring"):
            indent = 6
        if name.startswith("stage 3 leg"):
            indent = 6
        print(f"{ms:>9} ms {visited:>11} visited  {' ' * indent}{name}")
        if name == "route total":
            run += 1
            print(f"---- end of route build {run}")
        continue
    m = event.search(line)
    if m:
        print(f"{'':>34}  note: {m.group(1).strip()}")
