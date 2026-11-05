import json
import csv
import hashlib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
REC_DIR = REPO_ROOT / 'recorded_actions'
OUT_CSV = Path(__file__).resolve().parent / '08_dataset_audit.csv'


def file_md5(path: Path):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(8192), b''):
            h.update(chunk)
    return h.hexdigest()


def summarize(path: Path):
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    actions = data.get('actions') or data.get('recorded_actions') or []
    frame_count = len(actions)
    duration = 0.0
    if frame_count >= 2:
        t0 = actions[0].get('timestamp', 0.0)
        t1 = actions[-1].get('timestamp', t0)
        duration = float(t1) - float(t0)

    import statistics
    m0 = [a.get('motor_0', 0.0) for a in actions]
    m1 = [a.get('motor_1', 0.0) for a in actions]
    m2 = [a.get('motor_2', 0.0) for a in actions]

    row = {
        'trajectory_id': path.stem,
        'target': data.get('target',''),
        'split': data.get('split',''),
        'frame_count': frame_count,
        'duration_s': duration,
        'source_session': str(path.name),
        'phantom_id': data.get('phantom_id',''),
        'initial_pose_id': data.get('initial_state', {}).get('pose_id','') if isinstance(data.get('initial_state',{}), dict) else '',
        'illumination_condition': data.get('illumination_condition',''),
        'M0_action_mean': round(statistics.mean(m0),3) if m0 else '',
        'M0_action_sd': round(statistics.pstdev(m0),3) if m0 else '',
        'M1_action_mean': round(statistics.mean(m1),3) if m1 else '',
        'M1_action_sd': round(statistics.pstdev(m1),3) if m1 else '',
        'M2_action_mean': round(statistics.mean(m2),3) if m2 else '',
        'M2_action_sd': round(statistics.pstdev(m2),3) if m2 else '',
        'navigation_frames': frame_count,
        'intervention_frames': 0,
        'file_hash': file_md5(path),
        'leakage_check_pass': '',
        'notes': ''
    }
    return row


def main():
    if not REC_DIR.exists():
        print('recorded_actions not found')
        return

    files = sorted(REC_DIR.glob('*.json'))
    if not files:
        print('no json files')
        return

    # read header
    if not OUT_CSV.exists():
        print('CSV header missing:', OUT_CSV)
        return

    with open(OUT_CSV, 'r', encoding='utf-8') as f:
        header = f.readline().strip()
    cols = header.split(',')

    existing = set()
    with open(OUT_CSV, 'r', encoding='utf-8') as f:
        for line in f.readlines()[1:]:
            if line.strip():
                existing.add(line.split(',')[0])

    added = 0
    with open(OUT_CSV, 'a', encoding='utf-8', newline='') as outf:
        writer = csv.DictWriter(outf, fieldnames=cols)
        for p in files:
            tid = p.stem
            if tid in existing:
                continue
            try:
                r = summarize(p)
                row = {c: r.get(c, '') for c in cols}
                writer.writerow(row)
                added += 1
            except Exception as e:
                print('fail', p, e)

    print(f'Appended {added} rows to {OUT_CSV}')


if __name__ == '__main__':
    main()
