import os
from pxr import Sdf

ROOT = "/ubt_sim/assets/scenes/parlor"

print("=== Scanning for broken ps/Downloads references ===\n")
ps_hits = []
for dirpath, _, files in os.walk(ROOT):
    for fn in sorted(files):
        if not fn.endswith((".usd", ".usda", ".usdc")):
            continue
        path = os.path.join(dirpath, fn)
        try:
            layer = Sdf.Layer.FindOrOpen(path)
        except Exception as e:
            print("ERR open", path, e)
            continue
        if not layer:
            continue
        txt = layer.ExportToString()
        for line in txt.splitlines():
            s = line.strip()
            if ("ps/Downloads" in s or "ps\\Downloads" in s
                    or "ps:/Downloads" in s):
                ps_hits.append((path, s))

for path, s in ps_hits:
    rel = path.replace("/ubt_sim/", "")
    print(f"{rel}:\n    {s}")

print(f"\n=== total ps/Downloads hits: {len(ps_hits)} ===")

# Distinct broken asset path values (the substring from ps/ onward)
print("\n=== distinct broken path substrings ===")
distinct = sorted({s[s.find("ps/Downloads"):].split('"')[0].split("'")[0] for s in (h[1] for h in ps_hits)})
for d in distinct:
    print("   ", d)
