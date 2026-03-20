#!/usr/bin/env python3
"""
测试Tkinter是否可用
"""

try:
    import tkinter as tk
    from tkinter.scrolledtext import ScrolledText
    print("Tkinter available")
    print(f"  tk version: {tk.TkVersion}")
    print(f"  Tcl version: {tk.TclVersion}")

    # 测试创建窗口
    root = tk.Tk()
    root.title("Test Window")
    root.geometry("300x200")

    label = tk.Label(root, text="Tkinter works!")
    label.pack(pady=20)

    button = tk.Button(root, text="Close", command=root.quit)
    button.pack(pady=10)

    print("Tkinter window creation successful")

    # 自动关闭窗口
    root.after(1000, root.quit)  # 1秒后自动关闭
    root.mainloop()

    print("Tkinter test completed")

except ImportError as e:
    print(f"Tkinter import failed: {e}")
    print("Please install python-tk or tkinter package")
except Exception as e:
    print(f"Tkinter test failed: {e}")
