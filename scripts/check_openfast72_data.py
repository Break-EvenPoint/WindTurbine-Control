"""Validate the uploaded 72-path CSV, representative time series, and selection records.

This checks data consistency, not physical validity or reproduction of OpenFAST runs.
"""
from __future__ import annotations
import csv
import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "openfast_72"

def rows(name: str) -> list[dict[str, str]]:
    with (DATA / name).open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))

def close(actual: float, expected: float, tolerance: float = 0.001) -> None:
    assert abs(actual - expected) <= tolerance, (actual, expected)

def main() -> None:
    search = rows("search_results.csv")
    assert len(search) == 72
    assert len({r["stem"] for r in search}) == 72
    counts = Counter((int(float(r["on_s"])), int(float(r["yaw_deg"]))) for r in search)
    for t in (15, 30, 45):
        assert [counts[t, y] for y in (-3, 0, 3)] == [10, 4, 10]
    assert sum(r["eligible"] == "True" for r in search) == 72
    assert sum(float(r["delta_gen_kwh"]) > 0 for r in search) == 26
    assert sum(float(r["delta_gen_kwh"]) < 0 for r in search) == 46
    selected = json.loads((DATA / "search_selection.json").read_text(encoding="utf-8"))
    best = max(search, key=lambda r: float(r["delta_gen_kwh"]))
    worst = min(search, key=lambda r: float(r["delta_gen_kwh"]))
    close(float(best["delta_gen_kwh"]), selected["top"]["delta_gen_kwh"], 1e-8)
    close(float(worst["delta_gen_kwh"]), selected["worst"]["delta_gen_kwh"], 1e-8)
    for name, expected in [("best_timeseries.csv", best), ("worst_timeseries.csv", worst)]:
        trace = rows(name)
        assert len(trace) == 2301
        assert float(trace[0]["time_s"]) == 0
        assert float(trace[-1]["time_s"]) == 230.0
        assert all(abs(float(r["time_s"]) - idx * 0.1) <= 1e-8 for idx, r in enumerate(trace))
        close(float(trace[-1]["delta_cumulative_kwh"]), float(expected["delta_gen_kwh"]), 1e-8)
    print("PASS: 72 unique paths, 26 gains / 46 losses, map counts 10/4/10 per start time")
    print("PASS: best/worst search results agree with 2301-point time series")

if __name__ == "__main__":
    main()
