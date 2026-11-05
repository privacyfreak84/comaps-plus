import os
import re
import sys

scope = re.compile(r"\[plus-timing\] (.+?) ms: (\d+) visited: (\d+)(?: loads: (\d+))?")
event = re.compile(r"\[plus-timing\] (.+)$")
cand_score = re.compile(r"\[plus-timing\] candidate (\d+) score ([\d.e+]+)")
cand_time = re.compile(r"^stage 2 candidate (\d+) ")
start = re.compile(r"Start building route, checkpoints: Checkpoints\(ms::LatLon\(([-\d.]+), ([-\d.]+)\); ms::LatLon\(([-\d.]+), ([-\d.]+)\)")
length = re.compile(r"Route length: ([\d.e+]+) meters\. ETA: ([\d.e+]+) seconds")

log_path = sys.argv[1]
names = {}
names_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "routes-names.txt")
routes_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "routes.txt")
if os.path.exists(names_file) and os.path.exists(routes_file):
    for r, n in zip(open(routes_file), open(names_file)):
        parts = r.split()
        if len(parts) == 4:
            names[tuple(round(float(x), 4) for x in parts)] = n.strip()


def report(state):
    scores, times = state["scores"], state["times"]
    if not scores:
        return
    best_i = min(scores, key=scores.get)
    best = scores[best_i]
    order = sorted(scores)
    print(f"  winner: candidate {best_i} (rank {order.index(best_i) + 1} of {len(order)} by leaps estimate)")
    fixed = state["stage1"] + state["stage3"]
    for k in (1, 2, 3, 5, 8):
        if k >= len(order):
            break
        top = [i for i in order if i < k]
        if not top:
            continue
        kb = min(scores[i] for i in top)
        loss = (kb - best) / best * 100
        est = fixed + sum(times.get(i, 0) for i in top)
        print(f"  top {k}: route {'same' if loss == 0 else f'{loss:+.1f}% slower ETA'}, build about {est / 1000:.0f} s")


run = 0
state = None
for line in open(log_path, errors="replace"):
    m = start.search(line)
    if m:
        key = tuple(round(float(x), 4) for x in m.groups())
        state = {"scores": {}, "times": {}, "stage1": 0, "stage3": 0}
        print(f"==== {names.get(key, ' '.join(m.groups()))}")
        continue
    m = length.search(line)
    if m:
        print(f"  route: {float(m.group(1)) / 1000:.0f} km, ETA {float(m.group(2)) / 3600:.1f} h")
        continue
    m = cand_score.search(line)
    if m and state is not None:
        state["scores"][int(m.group(1))] = float(m.group(2))
        continue
    m = scope.search(line)
    if m:
        name, ms, visited = m.group(1), int(m.group(2)), int(m.group(3))
        if state is not None:
            c = cand_time.match(name)
            if c:
                state["times"][int(c.group(1))] = ms
            elif name == "stage 1 leaps search":
                state["stage1"] = ms
            elif name == "stage 3 expand best candidate":
                state["stage3"] = ms
        if name.startswith("stage 2 candidate ") and not name.startswith("stage 2 candidate scoring"):
            continue
        if name.startswith("stage 3 leg"):
            continue
        loads = f"{int(m.group(4)):>10} loads" if m.group(4) else ""
        print(f"{ms:>9} ms {visited:>11} visited {loads}  {name}")
        if name == "route total":
            run += 1
            if state is not None:
                report(state)
        continue
    m = event.search(line)
    if m and not m.group(1).startswith(("candidate ", "cache hit")):
        print(f"{'':>32}  note: {m.group(1).strip()}")
