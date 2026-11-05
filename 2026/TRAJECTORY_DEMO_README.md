# Standalone 3D trajectory reconstruction

This feature runs independently from the animal-experiment UI. It does not import or start
the camera, motors, speech recognition, prediction pipeline or AutoNav controller.

## Graphical showcase

```bat
cd /d G:\zq\Napoleon2025\2026
D:\ProgramData\anaconda3\python.exe trajectory_reconstruction_demo.py
```

Choose a JSON recording from `BC/expert_demos/AutoNavdatasets`, or click **Run sample**.
The generated JSON, CSV and interactive HTML files are written to `trajectory_outputs`.

## Command line

```bat
D:\ProgramData\anaconda3\python.exe trajectory_reconstruction_demo.py ^
  BC\expert_demos\AutoNavdatasets\Bronch\20260710_153219_LMB_96frames.json ^
  --no-open
```

The HTML visualization loads Plotly from its CDN, so the interactive browser view requires
network access. Reconstruction and JSON/CSV generation work offline.

The result is an encoder-based geometric estimate for analysis and demonstration. It is not
measured spatial localization and must not be used for clinical decisions.
