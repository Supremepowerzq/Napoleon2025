import os
import json
import csv
from pathlib import Path
from datetime import datetime

REPO_ROOT = Path(__file__).resolve().parents[3]
REC_DIR = REPO_ROOT / 'recorded_actions'
OUT_CSV = Path(__file__).resolve().parent / '01_navigation_trials.csv'


def load_json(path: Path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def summarize_action_file(path: Path):
    data = load_json(path)
    meta = data.get('metadata', {}) if isinstance(data, dict) else {}
    created_at = data.get('created_at') or meta.get('created_at')
    if created_at:
        try:
            dt = created_at
            # try parse ISO
            dt_obj = datetime.fromisoformat(dt)
            date_time = dt_obj.isoformat()
        except Exception:
            date_time = str(created_at)
    else:
        date_time = datetime.fromtimestamp(path.stat().st_mtime).isoformat()

    actions = data.get('actions') if isinstance(data, dict) else None
    if not actions:
        # fallback older format
        actions = data.get('recorded_actions') or []

    num_steps = len(actions)
    duration = 0.0
    initial = {'motor_0': '', 'motor_1': '', 'motor_2': ''}
    final = {'motor_0': '', 'motor_1': '', 'motor_2': ''}
    if num_steps > 0:
        try:
            t0 = actions[0].get('timestamp', 0.0)
            t1 = actions[-1].get('timestamp', t0)
            duration = float(t1) - float(t0)
        except Exception:
            duration = 0.0

        for k in ['motor_0', 'motor_1', 'motor_2']:
            initial[k] = actions[0].get(k, '')
            final[k] = actions[-1].get(k, '')

    trial_id = path.stem
    target = meta.get('target') or data.get('target') or ''
    phantom = meta.get('phantom_id', '')

    summary = {
        'trial_id': trial_id,
        'date_time': date_time,
        'formal_or_pilot': meta.get('formal_or_pilot', 'pilot'),
        'model_variant': meta.get('model_variant', ''),
        'checkpoint_id': meta.get('checkpoint_id', ''),
        'git_commit': meta.get('git_commit', ''),
        'randomization_block': meta.get('randomization_block', ''),
        'target': target,
        'initial_pose_id': meta.get('initial_pose_id', ''),
        'initial_M0_deg': initial.get('motor_0', ''),
        'initial_M1_deg': initial.get('motor_1', ''),
        'initial_M2_deg': initial.get('motor_2', ''),
        'illumination_condition': meta.get('illumination_condition', ''),
        'lens_condition': meta.get('lens_condition', ''),
        'phantom_id': phantom,
        'max_time_s': meta.get('max_time_s', ''),
        'confirmation_time_s': meta.get('confirmation_time_s', ''),
        'success': meta.get('success', ''),
        'wrong_branch': meta.get('wrong_branch', ''),
        'operator_takeover': meta.get('operator_takeover', ''),
        'emergency_stop': meta.get('emergency_stop', ''),
        'perception_loss': meta.get('perception_loss', ''),
        'completion_time_s': duration,
        'time_to_termination_s': meta.get('time_to_termination_s', ''),
        'contact_count': meta.get('contact_count', ''),
        'contact_duration_s': meta.get('contact_duration_s', ''),
        'primary_failure_mode': meta.get('primary_failure_mode', ''),
        'secondary_failure_modes': meta.get('secondary_failure_modes', ''),
        'end_M0_deg': final.get('motor_0', ''),
        'end_M1_deg': final.get('motor_1', ''),
        'end_M2_deg': final.get('motor_2', ''),
        'video_file': meta.get('video_file', ''),
        'state_log_file': meta.get('state_log_file', ''),
        'command_log_file': meta.get('command_log_file', ''),
        'excluded_from_primary': meta.get('excluded_from_primary', ''),
        'exclusion_reason': meta.get('exclusion_reason', ''),
        'notes': meta.get('notes', '')
    }

    return summary


def main():
    csv_path = OUT_CSV
    if not REC_DIR.exists():
        print('recorded_actions directory not found:', REC_DIR)
        return

    files = sorted(REC_DIR.glob('*.json'))
    if not files:
        print('no json files in recorded_actions')
        return

    # read header from existing csv
    if not csv_path.exists():
        print('CSV header not found at', csv_path)
        return

    with open(csv_path, 'r', encoding='utf-8') as f:
        header = f.readline().strip()
    header_cols = header.split(',')

    existing_ids = set()
    with open(csv_path, 'r', encoding='utf-8') as f:
        for line in f.readlines()[1:]:
            if line.strip():
                existing_ids.add(line.split(',')[0])

    rows_added = 0
    with open(csv_path, 'a', encoding='utf-8', newline='') as outf:
        writer = csv.DictWriter(outf, fieldnames=header_cols)
        for p in files:
            tid = p.stem
            if tid in existing_ids:
                continue
            try:
                s = summarize_action_file(p)
                # ensure all header cols present
                row = {k: s.get(k, '') for k in header_cols}
                writer.writerow(row)
                rows_added += 1
            except Exception as e:
                print('failed to summarize', p, e)

    print(f'Appended {rows_added} rows to {csv_path}')


if __name__ == '__main__':
    main()
