"""Build a speed table and field map from the Pedro Shooter Tuning CSV."""

import argparse
import csv
import io
import math
import statistics
from collections import defaultdict
from html import escape
from pathlib import Path
from urllib.request import urlopen


NUMBERS = (
    "x_in y_in heading_deg distance_in speed_percent target_ticks_s actual_ticks_s "
    "pivot_forward_in pivot_left_in pollen min_ticks_s movement_in heading_change_deg "
    "max_aim_error_deg hits misses high low"
).split()


def read_shots(source):
    reader = csv.DictReader(source)
    if not set(NUMBERS + ["hood", "session"]).issubset(reader.fieldnames or []):
        raise ValueError("Expected a Pedro Shooter Tuning CSV from the Control Hub")
    accepted, excluded = [], 0
    for row in reader:
        try:
            shot = {field: float(row[field]) for field in NUMBERS}
        except (ValueError, TypeError):
            excluded += 1
            continue
        counts = [shot[k] for k in ("hits", "misses", "high", "low")]
        if (not all(math.isfinite(value) for value in shot.values())
                or shot["pollen"] not in (1, 4)
                or any(n < 0 or not n.is_integer() for n in counts)
                or sum(counts) != shot["pollen"]
                or shot["target_ticks_s"] <= 0 or shot["actual_ticks_s"] <= 0
                or shot["movement_in"] > 2 or shot["heading_change_deg"] > 5
                or shot["max_aim_error_deg"] > 5):
            excluded += 1
            continue
        shot.update(hood=row["hood"], session=row["session"])
        accepted.append(shot)
    return accepted, excluded


def setup(shot):
    return tuple(shot[k] for k in ("hood", "pivot_forward_in", "pivot_left_in"))


def group_poses(shots, minimum, threshold):
    by_setting = defaultdict(list)
    for shot in shots:
        key = (*setup(shot), shot["speed_percent"], shot["target_ticks_s"])
        for group in by_setting[key]:
            first = group[0]
            shift = math.hypot(shot["x_in"] - first["x_in"], shot["y_in"] - first["y_in"])
            turn = abs(math.remainder(shot["heading_deg"] - first["heading_deg"], 360))
            if shift <= 2 and turn <= 5 and abs(shot["distance_in"] - first["distance_in"]) <= 2:
                group.append(shot)
                break
        else:
            by_setting[key].append([shot])

    results = []
    for groups in by_setting.values():
        for group in groups:
            first = group[0]
            pollen = int(sum(shot["pollen"] for shot in group))
            hits = int(sum(shot["hits"] for shot in group))
            bursts = [shot for shot in group if shot["pollen"] == 4]
            burst_hits = int(sum(shot["hits"] for shot in bursts))
            rate = hits / pollen
            status = "tested" if pollen < minimum else "pass" if rate >= threshold else "fail"
            results.append(dict(
                x_in=first["x_in"], y_in=first["y_in"], heading_deg=first["heading_deg"] % 360,
                distance_in=first["distance_in"], hood=first["hood"],
                pivot_forward_in=first["pivot_forward_in"], pivot_left_in=first["pivot_left_in"],
                speed_percent=int(first["speed_percent"]), target_ticks_s=first["target_ticks_s"],
                median_actual_ticks_s=round(statistics.median(s["actual_ticks_s"] for s in group), 1),
                lowest_ticks_s=round(min(s["min_ticks_s"] for s in group), 1),
                hits=hits, pollen=pollen, hit_rate=round(rate, 4), status=status,
                sessions=len({shot["session"] for shot in group}),
                burst_hits=burst_hits, burst_pollen=4 * len(bursts),
                high=int(sum(shot["high"] for shot in group)), low=int(sum(shot["low"] for shot in group)),
            ))
    return sorted(results, key=lambda row: (*setup(row), row["distance_in"], row["speed_percent"]))


def speed_table(poses, threshold):
    best = {}
    for row in poses:
        if row["status"] != "pass":
            continue
        bin_start = int(row["distance_in"] // 6) * 6
        key = (*setup(row), bin_start)
        candidate = {"distance_band_start_in": bin_start, **row}
        if key not in best or rank(candidate, threshold) > rank(best[key], threshold):
            best[key] = candidate
    return [best[key] for key in sorted(best)]


def rank(row, threshold):
    burst_pollen = row["burst_pollen"]
    burst_ok = burst_pollen >= 8 and row["burst_hits"] / burst_pollen >= threshold
    return burst_ok, row["sessions"], row["hit_rate"], row["pollen"], -row["speed_percent"]


FIELDS = ("x_in y_in heading_deg distance_in hood pivot_forward_in pivot_left_in speed_percent "
          "target_ticks_s median_actual_ticks_s lowest_ticks_s hits pollen hit_rate status sessions "
          "burst_hits burst_pollen high low").split()


def write_csv(path, fields, rows):
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def field_map(poses):
    def x(value):
        return 55 + value * 3

    def y(value):
        return 480 - value * 3

    colors = {"pass": "#198754", "fail": "#c33b40", "tested": "#a3660d"}
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="535" height="540" viewBox="0 0 535 540">',
           '<rect width="535" height="540" fill="white"/>',
           '<g font-family="sans-serif" fill="#172333" font-size="12">',
           '<text x="55" y="23" font-size="17">Pollen shooting poses · Pedro field (in)</text>']
    for tick in range(0, 145, 24):
        svg.extend([f'<path d="M{x(tick)} 48V480 M55 {y(tick)}H487" stroke="#dbe1e7"/>',
                    f'<text x="{x(tick)}" y="499" text-anchor="middle">{tick}</text>',
                    f'<text x="46" y="{y(tick)+4}" text-anchor="end">{tick}</text>'])
    svg.append(f'<path d="M{x(83)-5} {y(83)}h10 m-5 -5v10" stroke="#172333" stroke-width="2"/>')
    svg.append(f'<text x="{x(83)+8}" y="{y(83)+4}">Hive</text>')
    # Pass markers draw last so a successful setting stays visible at a shared pose.
    for row in sorted(poses, key=lambda item: item["status"] == "pass"):
        px, py = x(row["x_in"]), y(row["y_in"])
        heading = math.radians(row["heading_deg"])
        label = escape(f'{row["status"]} | {row["x_in"]}, {row["y_in"]}, {row["heading_deg"]} deg '
                       f'| {row["hits"]}/{row["pollen"]} | speed {row["speed_percent"]}% | hood {row["hood"]}')
        color = colors[row["status"]]
        svg.append(f'<g><title>{label}</title><circle cx="{px}" cy="{py}" r="5" fill="{color}"/>'
                   f'<path d="M{px} {py}l{11*math.cos(heading)} {-11*math.sin(heading)}" '
                   f'stroke="{color}" stroke-width="2"/></g>')
    for index, (status, label) in enumerate((("pass", "Pass"), ("fail", "Below threshold"),
                                            ("tested", "Needs more trials"))):
        px = 60 + 151 * index
        svg.append(f'<circle cx="{px}" cy="522" r="5" fill="{colors[status]}"/>'
                   f'<text x="{px+10}" y="526">{label}</text>')
    return "\n".join(svg + ["</g></svg>"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", help="Hub shot-tuning.csv URL or local CSV path")
    parser.add_argument("--out", type=Path, required=True, help="Output folder")
    parser.add_argument("--min-pollen", type=int, default=10)
    parser.add_argument("--hit-rate", type=float, default=0.9)
    args = parser.parse_args()
    if args.min_pollen < 1 or not 0 < args.hit_rate <= 1:
        parser.error("Use a positive minimum count and a hit rate in (0, 1]")
    if args.source.startswith(("http://", "https://")):
        with urlopen(args.source, timeout=10) as response:
            raw = response.read().decode("utf-8-sig")
    else:
        raw = Path(args.source).read_text(encoding="utf-8-sig")
    shots, excluded = read_shots(io.StringIO(raw))
    poses = group_poses(shots, args.min_pollen, args.hit_rate)
    speeds = speed_table(poses, args.hit_rate)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "pollen-shots.csv").write_text(raw, encoding="utf-8", newline="")
    write_csv(args.out / "shooting-poses.csv", FIELDS, poses)
    write_csv(args.out / "shooter-speeds.csv", ["distance_band_start_in", *FIELDS], speeds)
    (args.out / "shooting-map.svg").write_text(field_map(poses), encoding="utf-8")
    print(f"{len(shots)} usable trials, {excluded} excluded, {len(poses)} pose settings, "
          f"{len(speeds)} passing distance bands")


if __name__ == "__main__":
    main()
