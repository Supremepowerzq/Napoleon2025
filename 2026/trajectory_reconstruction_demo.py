"""Standalone showcase for encoder-based bronchoscope trajectory reconstruction."""

from __future__ import annotations

import argparse
import os
import threading
import webbrowser
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox

from trajectory_reconstruction_integration import DEFAULT_OUTPUT_DIR, reconstruct_file


HERE = Path(__file__).resolve().parent
DATA_DIR = HERE / "BC" / "expert_demos" / "AutoNavdatasets"
SAMPLE_FILE = HERE / "TrajectoryReconstruction" / "examples" / "minimal_session.json"


def open_path(path: str | Path) -> None:
    target = Path(path).resolve()
    if os.name == "nt" and target.is_dir():
        os.startfile(str(target))  # type: ignore[attr-defined]
    else:
        webbrowser.open_new_tab(target.as_uri())


class TrajectoryDemo:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Bronchoscope 3D Trajectory Reconstruction")
        self.root.geometry("820x540")
        self.root.minsize(700, 480)
        self.root.configure(bg="#edf1f4")
        self.source_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Ready")
        self.details_var = tk.StringVar(
            value="Choose a BC recording or run the bundled synthetic example."
        )
        self.last_result: dict | None = None
        self._build()

    def _build(self) -> None:
        header = tk.Frame(self.root, bg="#1746a2", height=96)
        header.pack(fill=tk.X)
        header.pack_propagate(False)
        tk.Label(
            header, text="3D TRAJECTORY RECONSTRUCTION",
            font=("Segoe UI", 20, "bold"), bg="#1746a2", fg="white",
        ).pack(anchor="w", padx=32, pady=(18, 0))
        tk.Label(
            header, text="Standalone encoder-based simulation and visualization",
            font=("Segoe UI", 10), bg="#1746a2", fg="#dbeafe",
        ).pack(anchor="w", padx=34, pady=(2, 0))

        card = tk.Frame(self.root, bg="white", padx=28, pady=24)
        card.pack(fill=tk.BOTH, expand=True, padx=28, pady=24)
        tk.Label(
            card, text="Input recording", font=("Segoe UI", 11, "bold"),
            bg="white", fg="#243b53",
        ).pack(anchor="w")
        entry_row = tk.Frame(card, bg="white")
        entry_row.pack(fill=tk.X, pady=(8, 16))
        tk.Entry(
            entry_row, textvariable=self.source_var, font=("Segoe UI", 10),
            relief=tk.SOLID, bd=1,
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=7)
        tk.Button(
            entry_row, text="Browse...", command=self.choose_file,
            font=("Segoe UI", 10), bg="#edf1f4", fg="#243b53", relief=tk.FLAT,
            padx=16, pady=7,
        ).pack(side=tk.LEFT, padx=(8, 0))

        actions = tk.Frame(card, bg="white")
        actions.pack(fill=tk.X, pady=(0, 18))
        self.run_button = tk.Button(
            actions, text="RECONSTRUCT & VIEW 3D", command=self.run,
            font=("Segoe UI", 10, "bold"), bg="#c7e6ba", fg="#1746a2",
            activebackground="#b5dca5", relief=tk.FLAT, padx=20, pady=10,
        )
        self.run_button.pack(side=tk.LEFT)
        tk.Button(
            actions, text="Run sample", command=self.run_sample,
            font=("Segoe UI", 10), bg="#b6d5ec", fg="#243b53",
            relief=tk.FLAT, padx=18, pady=10,
        ).pack(side=tk.LEFT, padx=8)
        tk.Button(
            actions, text="Open output folder", command=lambda: open_path(DEFAULT_OUTPUT_DIR),
            font=("Segoe UI", 10), bg="#edf1f4", fg="#243b53",
            relief=tk.FLAT, padx=18, pady=10,
        ).pack(side=tk.LEFT)

        status = tk.Frame(card, bg="#f4f8ff", padx=16, pady=14)
        status.pack(fill=tk.X)
        tk.Label(
            status, textvariable=self.status_var, font=("Segoe UI", 11, "bold"),
            bg="#f4f8ff", fg="#1746a2", anchor="w",
        ).pack(fill=tk.X)
        tk.Label(
            status, textvariable=self.details_var, font=("Segoe UI", 9),
            bg="#f4f8ff", fg="#52667a", anchor="w", justify=tk.LEFT,
            wraplength=690,
        ).pack(fill=tk.X, pady=(5, 0))

        tk.Label(
            card,
            text=("Analysis-only geometric estimate. It is not measured spatial localization, "
                  "does not control motors, and must not be used for clinical decisions."),
            font=("Segoe UI", 9), bg="white", fg="#8a4b08", justify=tk.LEFT,
            wraplength=700,
        ).pack(anchor="w", pady=(18, 0))

    def choose_file(self) -> None:
        selected = filedialog.askopenfilename(
            parent=self.root,
            title="Select a BC trajectory recording",
            initialdir=str(DATA_DIR if DATA_DIR.is_dir() else HERE),
            filetypes=(("Trajectory recording", "*.json"), ("All files", "*.*")),
        )
        if selected:
            self.source_var.set(selected)

    def run_sample(self) -> None:
        self.source_var.set(str(SAMPLE_FILE))
        self.run()

    def run(self) -> None:
        source = self.source_var.get().strip()
        if not source:
            self.choose_file()
            source = self.source_var.get().strip()
        if not source:
            return
        self.run_button.configure(state=tk.DISABLED)
        self.status_var.set("Reconstructing...")
        self.details_var.set(source)

        def worker() -> None:
            try:
                result = reconstruct_file(source)
            except Exception as exc:
                self.root.after(0, lambda: self._finish_error(str(exc)))
            else:
                self.root.after(0, lambda: self._finish_success(result))

        threading.Thread(target=worker, name="trajectory_demo", daemon=True).start()

    def _finish_success(self, result: dict) -> None:
        self.last_result = result
        self.run_button.configure(state=tk.NORMAL)
        self.status_var.set(f"Complete - {result['frames']} trajectory points")
        self.details_var.set(
            f"HTML: {result['html']}\nJSON: {result['json']}\nCSV: {result['csv']}"
        )
        open_path(result["html"])

    def _finish_error(self, error: str) -> None:
        self.run_button.configure(state=tk.NORMAL)
        self.status_var.set("Reconstruction failed")
        self.details_var.set(error)
        messagebox.showerror("Trajectory reconstruction", error, parent=self.root)

    def start(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path, help="BC JSON recording")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--no-open", action="store_true", help="Do not open the generated HTML")
    args = parser.parse_args()
    if args.input:
        result = reconstruct_file(args.input, args.output_dir)
        print(f"Generated {result['frames']} trajectory points")
        print(result["json"])
        print(result["csv"])
        print(result["html"])
        if not args.no_open:
            open_path(result["html"])
        return
    TrajectoryDemo().start()


if __name__ == "__main__":
    main()
