#!/usr/bin/env python3
import itertools, subprocess, time, os, json, csv, argparse
parser = argparse.ArgumentParser()
parser.add_argument("--duration_s", type=int, default=25)
parser.add_argument("--out_dir", default="results")
parser.add_argument("--matrix", default="reliability,history,depth;best_effort|reliable,keep_last|keep_all,5|10")
args = parser.parse_args()
os.makedirs(args.out_dir, exist_ok=True)
left,right = args.matrix.split(";")
keys = left.split(",")
vals = [x.split("|") for x in right.split(",")]
runs = [dict(zip(keys, combo)) for combo in itertools.product(*vals)]
rows = []
for i, cfg in enumerate(runs, 1):
    run_id = f"m{i:03d}"
    cmd = ["ros2","launch","qos_rt_lab","lab.launch.py",
        f"reliability:={cfg.get('reliability','reliable')}",
        f"history:={cfg.get('history','keep_last')}",
        f"depth:={cfg.get('depth','10')}",
        f"run_id:={run_id}",
        f"out_dir:={args.out_dir}"]
    print("[*]", " ".join(cmd), flush=True)
    p = subprocess.Popen(cmd)
    try: time.sleep(args.duration_s)
    finally:
        p.terminate()
        try: p.wait(5)
        except subprocess.TimeoutExpired: p.kill(); p.wait()
    summary = os.path.join(args.out_dir, f"qos_rt_{run_id}.json")
    if os.path.exists(summary):
        with open(summary) as f: data = json.load(f)
        rows.append({"run_id": run_id, **cfg, **data})
    else:
        rows.append({"run_id": run_id, **cfg, "error":"no_summary"})
csv_path = os.path.join(args.out_dir, "matrix_results.csv")
with open(csv_path, "w", newline="") as f:
    import csv
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader(); writer.writerows(rows)
print("[*] Wrote", csv_path)
