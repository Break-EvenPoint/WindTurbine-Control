"""Create OpenFAST uniform wind input and C-wrapper command schedule from JSON.

This is a new input-preparation utility; it does not execute or reproduce
historical OpenFAST simulations without the separately built model/wrapper.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def wind_rows(config: dict):
    t0 = float(config['transition_start_s'])
    t1 = float(config['transition_end_s'])
    tmax = float(config['duration_s'])
    dt = float(config.get('sample_dt_s', 0.5))
    v0 = float(config['initial_speed_ms'])
    v1 = float(config['final_speed_ms'])
    d0 = float(config['initial_dir_deg'])
    d1 = float(config['final_dir_deg'])
    if not (0 < dt <= 10 and 0 <= t0 < t1 <= tmax and min(v0, v1) >= 0):
        raise ValueError('Invalid wind transition bounds, sample step or wind speed')
    dchange = (d1 - d0 + 180) % 360 - 180
    if not math.isclose((tmax / dt), round(tmax / dt), abs_tol=1e-8):
        raise ValueError('For deterministic sampling, duration must be divisible by dt')
    for idx in range(int(round(tmax / dt)) + 1):
        t = idx * dt
        a = min(1.0, max(0.0, (t - t0) / (t1 - t0)))
        yield (t, v0 + (v1 - v0)*a, d0 + dchange*a, 0.0, 0.0, 0.2, 0.0, 0.0)


def schedule_rows(config: dict, end_time: float):
    rows = config.get('events', [])
    if not isinstance(rows, list):
        raise ValueError('events must be a list')
    prev = -1.0
    state = {'torque_offset_nm': 0.0, 'pitch_offset_deg': 0.0, 'yaw_offset_deg': 0.0}
    for event in rows:
        time = float(event['time_s'])
        if not (math.isfinite(time) and prev < time <= end_time):
            raise ValueError('Event times must be strictly increasing within the run')
        prev = time
        for key in state:
            if key in event:
                state[key] = float(event[key])
                if not math.isfinite(state[key]):
                    raise ValueError(f'Non-finite {key}')
        yield {'time_s': time, **state}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--wind', type=Path, default=ROOT/'configs/openfast72_wind.json')
    ap.add_argument('--events', type=Path, default=ROOT/'configs/openfast72_best_schedule.json')
    ap.add_argument('--out', type=Path, default=ROOT/'build')
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    wind = json.loads(args.wind.read_text(encoding='utf-8'))
    events = json.loads(args.events.read_text(encoding='utf-8'))
    output_wind = args.out/'uniform_wind.hh'
    with output_wind.open('w',encoding='ascii') as f:
        f.write('! OpenFAST InflowWind WindType=2 uniform wind\n')
        f.write('! Time[s] HorSpd[m/s] Dir[deg] Vz[m/s] HLinShr VShr VLinShr VGust[m/s]\n')
        for row in wind_rows(wind):
            f.write(' '.join(f'{x:.6f}' for x in row)+'\n')
    output_schedule = args.out/'control_schedule.csv'
    with output_schedule.open('w',encoding='ascii',newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['time_s','torque_offset_nm','pitch_offset_deg','yaw_offset_deg'])
        writer.writeheader()
        writer.writerows(schedule_rows(events, float(wind['duration_s'])))
    print(f'Wind input: {output_wind}')
    print(f'Offset events: {output_schedule}')
    print('Preparation only. Configure the actual OpenFAST/ROSCO DISCON wrapper separately.')

if __name__ == '__main__':
    main()
