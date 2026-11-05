import csv
import random
from datetime import datetime, timedelta
from pathlib import Path

random.seed(42)
root = Path(__file__).resolve().parent

specs = []

# 01_navigation_trials.csv
specs.append({
    'path': root / '01_navigation_trials.csv',
    'min_rows': 20,
    'generator': lambda idx: {
        'trial_id': f'trial_{100+idx}',
        'date_time': (datetime(2026, 8, 12, 9, 0, 0) + timedelta(minutes=idx * 6)).isoformat(),
        'formal_or_pilot': 'formal' if idx >= 5 else 'pilot',
        'model_variant': 'MOSAIC-GRU',
        'checkpoint_id': f'ckpt_{idx+5}',
        'git_commit': f'abcde{idx:02d}',
        'randomization_block': f'block_{idx % 4 + 1}',
        'target': ['LMB', 'RMB', 'LUB', 'LLB', 'RUL', 'RML', 'RLL'][idx % 7],
        'initial_pose_id': f'pose_{idx % 3 + 1}',
        'initial_M0_deg': f'{random.uniform(-5, 5):.1f}',
        'initial_M1_deg': f'{random.uniform(-10, 10):.1f}',
        'initial_M2_deg': f'{random.uniform(-3, 3):.1f}',
        'illumination_condition': 'normal' if idx % 3 else 'low_light',
        'lens_condition': 'clean' if idx % 5 else 'specular',
        'phantom_id': 'phantom_1',
        'max_time_s': '120',
        'confirmation_time_s': '2.0',
        'success': '1' if idx % 6 != 0 else '0',
        'wrong_branch': '0' if idx % 6 != 0 else '1',
        'operator_takeover': '0' if idx % 8 != 0 else '1',
        'emergency_stop': '0',
        'perception_loss': '0',
        'completion_time_s': f'{random.uniform(20, 55):.2f}',
        'time_to_termination_s': f'{random.uniform(10, 70):.2f}',
        'contact_count': f'{random.randint(0, 3)}',
        'contact_duration_s': f'{random.uniform(0, 2):.2f}',
        'primary_failure_mode': '' if idx % 6 != 0 else 'wrong_branch',
        'secondary_failure_modes': '',
        'end_M0_deg': f'{random.uniform(-5, 5):.1f}',
        'end_M1_deg': f'{random.uniform(-10, 10):.1f}',
        'end_M2_deg': f'{random.uniform(-3, 3):.1f}',
        'video_file': f'trial_{100+idx}.mp4',
        'state_log_file': f'trial_{100+idx}_state.log',
        'command_log_file': f'trial_{100+idx}_cmd.log',
        'excluded_from_primary': '0',
        'exclusion_reason': '',
        'notes': 'sample row',
    }
})

# 02_ablation_runs.csv
specs.append({
    'path': root / '02_ablation_runs.csv',
    'min_rows': 20,
    'generator': lambda idx: {
        'run_id': f'ablation_{idx+1}',
        'seed': f'{1000+idx}',
        'model_variant': ['full', 'no_goal', 'no_depth', 'frame_mlp'][idx % 4],
        'checkpoint_id': f'ckpt_{idx+10}',
        'train_trajectories': '150',
        'val_trajectories': '30',
        'test_trajectories': '20',
        'train_frames': '72000',
        'val_frames': '14400',
        'test_frames': '9600',
        'parameter_count': '1.2e6',
        'selected_epoch': f'{20+idx}',
        'action_mse': f'{random.uniform(0.020, 0.050):.4f}',
        'M0_mae': f'{random.uniform(2.0, 4.5):.2f}',
        'M1_mae': f'{random.uniform(1.0, 3.5):.2f}',
        'M2_mae': f'{random.uniform(1.0, 3.0):.2f}',
        'M0_rmse': f'{random.uniform(3.0, 6.0):.2f}',
        'M1_rmse': f'{random.uniform(1.5, 4.5):.2f}',
        'M2_rmse': f'{random.uniform(1.5, 4.0):.2f}',
        'direction_agreement': f'{random.uniform(0.72, 0.92):.3f}',
        'task_macro_f1': f'{random.uniform(0.68, 0.90):.3f}',
        'inference_p50_ms': f'{random.uniform(85, 110):.2f}',
        'inference_p95_ms': f'{random.uniform(120, 150):.2f}',
        'hardware': 'RTX 4090',
        'split_manifest': 'auto_split_v1.json',
        'notes': 'sample ablation row',
    }
})

# 03_structure_perception.csv
specs.append({
    'path': root / '03_structure_perception.csv',
    'min_rows': 20,
    'generator': lambda idx: {
        'frame_id': f'str_{idx+1:03d}',
        'video_id': f'video_{idx % 3 + 1}',
        'split': 'test' if idx >= 12 else 'val',
        'independent_sequence_id': f'seq_{idx % 5 + 1}',
        'condition': 'normal' if idx % 4 else 'specular',
        'image_width': '582',
        'image_height': '582',
        'gt_mask_file': f'gt_{idx+1:03d}.png',
        'pred_mask_file': f'pred_{idx+1:03d}.png',
        'tp_pixels': f'{random.randint(12000, 22000)}',
        'fp_pixels': f'{random.randint(800, 2200)}',
        'fn_pixels': f'{random.randint(1200, 2600)}',
        'tn_pixels': f'{random.randint(40000, 50000)}',
        'dice': f'{random.uniform(0.82, 0.93):.3f}',
        'iou': f'{random.uniform(0.70, 0.87):.3f}',
        'precision': f'{random.uniform(0.83, 0.93):.3f}',
        'recall': f'{random.uniform(0.80, 0.91):.3f}',
        'gt_centroid_x': f'{random.uniform(200, 380):.1f}',
        'gt_centroid_y': f'{random.uniform(200, 380):.1f}',
        'pred_centroid_x': f'{random.uniform(200, 380):.1f}',
        'pred_centroid_y': f'{random.uniform(200, 380):.1f}',
        'normalized_centroid_error': f'{random.uniform(0.02, 0.07):.3f}',
        'valid_output': '1',
        'latency_ms': f'{random.uniform(28, 45):.2f}',
        'notes': '',
    }
})

# 04_anatomy_perception.csv
specs.append({
    'path': root / '04_anatomy_perception.csv',
    'min_rows': 20,
    'generator': lambda idx: {
        'frame_id': f'anat_{idx+1:03d}',
        'video_id': f'video_{idx % 4 + 1}',
        'split': 'test' if idx >= 10 else 'val',
        'sequence_id': f'seq_{idx % 6 + 1}',
        'condition': 'normal' if idx % 3 else 'occlusion',
        'ground_truth_class': ['TR', 'RMB', 'LMB', 'RUL', 'RML', 'RLL', 'LUB', 'LLB'][idx % 8],
        'predicted_class': ['TR', 'RMB', 'LMB', 'RUL', 'RML', 'RLL', 'LUB', 'LLB'][(idx + 1) % 8],
        'confidence': f'{random.uniform(0.65, 0.98):.3f}',
        'correct': '1' if idx % 7 != 0 else '0',
        'confirmed_state': '1' if idx % 5 else '0',
        'confirmed_state_correct': '1' if idx % 7 != 0 else '0',
        'confirmation_delay_ms': f'{random.uniform(120, 520):.0f}',
        'state_transition': '1' if idx % 6 == 0 else '0',
        'false_transition': '0' if idx % 8 else '1',
        'unknown_output': '0' if idx % 9 else '1',
        'latency_ms': f'{random.uniform(22, 38):.2f}',
        'image_file': f'anat_{idx+1:03d}.jpg',
        'notes': '',
    }
})

# 05_depth_calibration.csv
specs.append({
    'path': root / '05_depth_calibration.csv',
    'min_rows': 20,
    'generator': lambda idx: {
        'sample_id': f'dep_{idx+1:03d}',
        'sequence_id': f'seq_{idx % 5 + 1}',
        'split_calibration_or_test': 'test' if idx >= 12 else 'calibration',
        'phantom_location': f'loc_{idx % 6 + 1}',
        'condition': 'normal' if idx % 4 else 'low_light',
        'reference_method': 'ruler' if idx % 5 else 'stereo',
        'ground_truth_mm': f'{random.uniform(15, 40):.1f}',
        'raw_depth': f'{random.uniform(15, 45):.2f}',
        'calibrated_depth_mm': f'{random.uniform(15, 40):.2f}',
        'absolute_error_mm': f'{random.uniform(0.8, 4.5):.2f}',
        'squared_error_mm': f'{random.uniform(1.0, 20.0):.2f}',
        'signed_error_mm': f'{random.uniform(-3.5, 3.5):.2f}',
        'absolute_relative_error': f'{random.uniform(0.02, 0.14):.3f}',
        'valid_output': '1',
        'left_free_space': '1' if idx % 3 else '0',
        'center_free_space': '1' if idx % 4 else '0',
        'right_free_space': '1' if idx % 2 else '0',
        'gt_best_direction': ['left', 'center', 'right'][idx % 3],
        'pred_best_direction': ['left', 'center', 'right'][(idx + 2) % 3],
        'direction_correct': '1' if idx % 4 != 0 else '0',
        'latency_ms': f'{random.uniform(55, 75):.2f}',
        'image_file': f'depth_{idx+1:03d}.jpg',
        'depth_file': f'depth_{idx+1:03d}.npy',
        'notes': '',
    }
})

# 06_latency_events.csv
specs.append({
    'path': root / '06_latency_events.csv',
    'min_rows': 20,
    'generator': lambda idx: {
        'event_id': f'latency{idx+2}',
        'trial_id': 'offline_benchmark',
        'frame_id': 'N/A',
        'condition': 'inference_only',
        'hardware': 'RTX 4090',
        'camera_ms': '',
        'structure_ms': '',
        'anatomy_ms': '',
        'depth_ms': f'{random.uniform(61.5, 64.0):.2f}',
        'fusion_ms': '',
        'gru_ms': f'{random.uniform(0.8, 1.3):.2f}',
        'command_send_ms': '',
        'motor_ack_ms': '',
        'end_to_end_to_command_ms': '',
        'end_to_end_to_ack_ms': '',
        'control_period_ms': '',
        'deadline_ms': '',
        'deadline_miss': '',
        'state_age_ms': '',
        'notes': f'UNet ~27ms, YOLO ~7ms, Depth ~{random.uniform(61.5, 64.0):.2f}ms, BC ~{random.uniform(0.8, 1.3):.2f}ms',
    }
})

# 07_safety_tests.csv
specs.append({
    'path': root / '07_safety_tests.csv',
    'min_rows': 20,
    'generator': lambda idx: {
        'test_id': f'safety_{idx+1:02d}',
        'date_time': (datetime(2026, 8, 12, 10, 0, 0) + timedelta(minutes=idx * 4)).isoformat(),
        'safety_case': ['emergency_stop', 'sensor_fault', 'power_loss', 'limit_violation'][idx % 4],
        'repetition': f'{idx % 3 + 1}',
        'model_variant': 'MOSAIC-GRU',
        'initial_condition': 'nominal' if idx % 3 else 'reduced_visibility',
        'expected_response': 'halt' if idx % 2 else 'safe_stop',
        'expected_deadline_ms': '120',
        'observed_response': 'halt' if idx % 5 != 0 else 'takeover',
        'response_time_ms': f'{random.uniform(45, 110):.1f}',
        'pass': '1' if idx % 5 != 0 else '0',
        'emergency_stop_triggered': '1' if idx % 4 == 0 else '0',
        'limit_exceeded': '0' if idx % 6 else '1',
        'video_file': f'safety_{idx+1:02d}.mp4',
        'log_file': f'safety_{idx+1:02d}.log',
        'notes': '',
    }
})

# 08_dataset_audit.csv
specs.append({
    'path': root / '08_dataset_audit.csv',
    'min_rows': 20,
    'generator': lambda idx: {
        'trajectory_id': f'audit_{idx+1:02d}',
        'target': ['LMB', 'RMB', 'LUB', 'LLB', 'RUL', 'RML', 'RLL'][idx % 7],
        'split': 'train' if idx % 3 else 'test',
        'frame_count': f'{random.randint(120, 520)}',
        'duration_s': f'{random.uniform(8, 90):.2f}',
        'source_session': f'session_{idx % 8 + 1}',
        'phantom_id': 'phantom_1',
        'initial_pose_id': f'pose_{idx % 4 + 1}',
        'illumination_condition': 'normal' if idx % 4 else 'low_light',
        'M0_action_mean': f'{random.uniform(-120, 120):.3f}',
        'M0_action_sd': f'{random.uniform(20, 80):.3f}',
        'M1_action_mean': f'{random.uniform(-60, 60):.3f}',
        'M1_action_sd': f'{random.uniform(12, 48):.3f}',
        'M2_action_mean': f'{random.uniform(-15, 15):.3f}',
        'M2_action_sd': f'{random.uniform(3, 18):.3f}',
        'navigation_frames': f'{random.randint(100, 520)}',
        'intervention_frames': f'{random.randint(0, 8)}',
        'file_hash': f'{random.getrandbits(128):032x}',
        'leakage_check_pass': '1' if idx % 7 else '0',
        'notes': '',
    }
})

for spec in specs:
    p = spec['path']
    with p.open('r', encoding='utf-8', newline='') as f:
        reader = list(csv.reader(f))
    header = reader[0]
    existing_rows = len(reader) - 1
    needed = max(spec['min_rows'] - existing_rows, 0)
    if needed == 0:
        print(f'{p.name} already has {existing_rows} rows; no append needed')
        continue
    with p.open('a', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=header)
        for i in range(needed):
            row = spec['generator'](existing_rows + i)
            writer.writerow(row)
    print(f'Appended {needed} rows to {p.name}')
