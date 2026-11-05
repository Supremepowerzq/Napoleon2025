import csv
from pathlib import Path

CSV_PATH = Path(__file__).resolve().parent / '06_latency_events.csv'
ROW = {
    'event_id': 'latency1',
    'trial_id': 'offline_benchmark',
    'frame_id': 'N/A',
    'condition': 'inference_only',
    'hardware': 'RTX 4090',
    'camera_ms': '',
    'structure_ms': '',
    'anatomy_ms': '',
    'depth_ms': '62.53',
    'fusion_ms': '',
    'gru_ms': '0.87',
    'command_send_ms': '',
    'motor_ack_ms': '',
    'end_to_end_to_command_ms': '',
    'end_to_end_to_ack_ms': '',
    'control_period_ms': '',
    'deadline_ms': '',
    'deadline_miss': '',
    'state_age_ms': '',
    'notes': 'UNet 26.62ms, YOLO 7.31ms, Depth 62.53ms, BC 0.87ms'
}

with open(CSV_PATH, 'a', encoding='utf-8', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=list(ROW.keys()))
    writer.writerow(ROW)
print('Appended latency event to', CSV_PATH)
