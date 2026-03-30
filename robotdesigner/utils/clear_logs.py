
def clearLogs():
    import os
    import shutil
    from pathlib import Path

    log_names = {"logs", "results", "tensorboard_logs"}
    root = Path(__file__).resolve().parents[2]  # package root (../skillcomp)

    found = False
    for p in root.rglob("*"):
        if p.is_dir() and p.name in log_names:
            found = True
            try:
                shutil.rmtree(str(p))
                print(f"Cleared log directory: {p}")
            except Exception as e:
                print(f"Failed to remove {p}: {e}")

    if not found:
        print(f"No log directories ({', '.join(sorted(log_names))}) found under {root}")