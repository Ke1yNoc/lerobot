#!/usr/bin/env python3
import json
import subprocess
import sys
import os
import argparse

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(BASE_DIR, 'experiments_small_motion.json')


def run_experiment(exp, defaults):
    cmd = [sys.executable, os.path.join(BASE_DIR, 'policy_to_joint_demo.py')]
    if exp.get('use_orient', False):
        cmd.append('--use_orient')
    # current joints
    cmd.extend(['--current'] + [str(v) for v in exp['current']])
    # delta: prefer explicit delta; fallback to pikaA/B difference if provided
    if 'delta' in exp:
        cmd.extend(['--delta'] + [str(v) for v in exp['delta']])
    else:
        pikaA = exp.get('pikaA')
        pikaB = exp.get('pikaB')
        if pikaA is None or pikaB is None:
            raise ValueError(f"Experiment '{exp.get('name', '<unnamed>')}' missing 'delta' or 'pikaA'/'pikaB'.")
        # Simple small-angle difference for xyzrpy (sufficient for small motions)
        delta = [float(pikaB[i]) - float(pikaA[i]) for i in range(6)]
        cmd.extend(['--delta'] + [str(v) for v in delta])
    # delta frame (ee/world), default to ee
    delta_frame = exp.get('delta_frame', 'ee')
    cmd.extend(['--delta_frame', delta_frame])
    # thresholds & interpolation settings
    cmd.extend(['--pos_thresh', str(defaults.get('pos_thresh', 0.3))])
    cmd.extend(['--ori_thresh', str(defaults.get('ori_thresh', 1.0))])
    cmd.extend(['--interp_threshold_deg', str(defaults.get('interp_threshold_deg', 30.0))])
    cmd.extend(['--interp_step_deg', str(defaults.get('interp_step_deg', 1.0))])
    # optional IK weights (defaults-level or per-experiment override)
    pos_w = exp.get('pos_weight', defaults.get('pos_weight', 1.0))
    ori_w = exp.get('ori_weight', defaults.get('ori_weight', 0.5))
    cmd.extend(['--pos_weight', str(pos_w)])
    cmd.extend(['--ori_weight', str(ori_w)])

    print(f"\n=== Running {exp['name']} ===")
    print('Command:', ' '.join(cmd))
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    output = proc.stdout
    print(output)
    return {
        'name': exp['name'],
        'returncode': proc.returncode,
        'output': output,
    }


def main():
    parser = argparse.ArgumentParser(description='Run policy_to_joint demo experiments')
    parser.add_argument('--config', type=str, default=DEFAULT_CONFIG_PATH, help='Path to experiments JSON config')
    args = parser.parse_args()

    with open(args.config, 'r') as f:
        cfg = json.load(f)

    defaults = cfg.get('defaults', {})
    exps = cfg.get('experiments', [])

    results = []
    for exp in exps:
        res = run_experiment(exp, defaults)
        results.append(res)

    # Save a summary file
    summary_name = os.path.splitext(os.path.basename(args.config))[0] + '_summary.txt'
    summary_path = os.path.join(BASE_DIR, summary_name)
    with open(summary_path, 'w') as f:
        for r in results:
            f.write(f"Experiment: {r['name']}\n")
            f.write(f"Return code: {r['returncode']}\n")
            # brief status inference
            ok = ('Solved joint angles' in r['output']) and ('over_limit: False' in r['output'])
            f.write(f"Status: {'OK' if ok else 'CHECK'}\n")
            f.write("---\n")
    print(f"\nSummary written to {summary_path}")


if __name__ == '__main__':
    main()