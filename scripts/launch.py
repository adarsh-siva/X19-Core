#!/usr/bin/env python3
"""
ROV X19-Core Multi-Node Orchestrator.

Launches Core's Python nodes as tmux panes in one session, and - on stop -
explicitly terminates each node's real process (SIGTERM, then SIGKILL if it
doesn't exit in time) before tearing down the tmux session itself.

This is deliberate rather than just calling `tmux kill-session`: killing a
pane sends SIGHUP to whatever's attached to its pty (the same signal an SSH
disconnect sends), not SIGTERM. Core's nodes (e.g. imu_node.py) only
register handlers for SIGTERM/SIGINT, so a bare pane-kill would bypass their
graceful shutdown path entirely - ZMQ sockets wouldn't get closed, and a
DepthAI device wouldn't get released cleanly - leaving an orphaned process
that can block the next run from opening the same port/camera.
"""
import os
import sys
import time
import signal
import argparse
import subprocess
import shutil
import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
YAML_PATH = os.path.join(ROOT_DIR, "launch.yaml")

GRACEFUL_TIMEOUT_S = 5  # how long to wait for SIGTERM before escalating to SIGKILL


def check_dependencies():
    if not shutil.which("tmux"):
        print("❌ Error: 'tmux' is not installed on this system. Please install it using 'sudo apt install tmux'.")
        sys.exit(1)


def load_config():
    if not os.path.exists(YAML_PATH):
        print(f"❌ Error: Configuration file '{YAML_PATH}' not found.")
        sys.exit(1)
    with open(YAML_PATH, "r") as f:
        return yaml.safe_load(f)


def run_cmd(cmd_str, check=True, capture_output=True):
    return subprocess.run(cmd_str, shell=True, check=check, capture_output=capture_output, text=True)


def session_exists(session_name):
    return run_cmd(f"tmux has-session -t {session_name}", check=False).returncode == 0


def pane_pids(session_name):
    """Real PIDs of every node process currently running in this session's panes."""
    res = run_cmd(f"tmux list-panes -a -t {session_name} -F '#{{pane_pid}}'", check=False)
    if res.returncode != 0:
        return []
    return [int(pid) for pid in res.stdout.split() if pid.strip()]


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # exists, just not ours - treat as alive


def terminate_children(pids):
    """SIGTERM every tracked node PID, wait, escalate to SIGKILL for stragglers."""
    if not pids:
        return

    print(f"🛑 Sending SIGTERM to {len(pids)} node process(es)...")
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass  # already gone

    deadline = time.time() + GRACEFUL_TIMEOUT_S
    remaining = set(pids)
    while remaining and time.time() < deadline:
        remaining = {pid for pid in remaining if pid_alive(pid)}
        if remaining:
            time.sleep(0.2)

    if remaining:
        print(f"⚠️ {len(remaining)} node(s) did not exit within {GRACEFUL_TIMEOUT_S}s, sending SIGKILL...")
        for pid in remaining:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    else:
        print("✅ All nodes exited gracefully.")


def stop_session(session_name):
    if not session_exists(session_name):
        print(f"ℹ️ No active session named '{session_name}' found.")
        return

    terminate_children(pane_pids(session_name))

    print(f"🛑 Tearing down tmux session '{session_name}'...")
    run_cmd(f"tmux kill-session -t {session_name}", check=False)
    print("✅ Session stopped.")


def wrap_command(cmd):
    # PYTHONPATH so nodes can `from src.python...` regardless of cwd.
    # `exec` replaces the wrapping shell with the node's own process, so the
    # pane's PID *is* the real node PID - without it, pane_pids() would only
    # ever see the shell wrapper, and SIGTERM sent to that shell is not
    # automatically forwarded to the node process running under it.
    return f"export PYTHONPATH='{ROOT_DIR}'; export PYTHONUNBUFFERED=1; exec {cmd}"


def launch_session(mode):
    check_dependencies()
    config = load_config()
    session_name = config.get("session_name", "rov_x19_core")

    # Stop any existing session (and its real node processes) first.
    stop_session(session_name)

    nodes = config.get("nodes", [])
    active_nodes = [n for n in nodes if n.get("mode", "all") in ("all", mode)]

    if not active_nodes:
        print("❌ No active nodes configured for mode:", mode)
        sys.exit(1)

    print(f"\n🚀 Launching ROV X19-Core Nodes in [{mode.upper()}] mode inside tmux session '{session_name}'...")
    print("--------------------------------------------------------------------------------")

    first_node = active_nodes[0]
    first_cwd = os.path.abspath(os.path.join(ROOT_DIR, first_node.get("cwd", ".")))
    first_cmd = wrap_command(first_node["cmd"])
    first_name = first_node.get("name", "Node 1")

    print(f"  [Pane 1] Launching '{first_name}' ({first_node['cmd']})")
    run_cmd(
        f"tmux new-session -d -s {session_name} "
        f"-n 'Core_Nodes' -c '{first_cwd}' \"{first_cmd}\""
    )

    for idx, node in enumerate(active_nodes[1:], start=2):
        name = node.get("name", f"Node {idx}")
        cwd = os.path.abspath(os.path.join(ROOT_DIR, node.get("cwd", ".")))
        cmd = wrap_command(node["cmd"])

        print(f"  [Pane {idx}] Launching '{name}' ({node['cmd']})")
        run_cmd(
            f"tmux split-window -d -t {session_name}:0 "
            f"-c '{cwd}' \"{cmd}\""
        )

    run_cmd(f"tmux select-layout -t {session_name}:0 tiled", check=False)
    run_cmd(f"tmux set-option -t {session_name} mouse on", check=False)
    run_cmd("tmux bind-key -T prefix q kill-session", check=False)
    run_cmd(f"tmux set-option -t {session_name} remain-on-exit on", check=False)
    run_cmd(f"tmux select-layout -t {session_name}:0 tiled", check=False)

    print("--------------------------------------------------------------------------------")
    print(f"✅ All {len(active_nodes)} nodes launched successfully!")
    print("\n📌 Mouse & Keyboard Controls:")
    print("  • Focus pane: CLICK any pane with your mouse")
    print("  • Scroll logs: SCROLL UP/DOWN with your mouse wheel")
    print("  • Stop all nodes & exit: Press 'Ctrl+B' then 'q', or run 'python3 scripts/launch.py --stop'")
    print("  • Detach session (keep running in background): Press 'Ctrl+B' then 'd'\n")

    os.execvp("tmux", ["tmux", "attach-session", "-t", session_name])


def main():
    parser = argparse.ArgumentParser(description="ROV X19-Core Multi-Node Orchestrator")
    parser.add_argument("--dev", action="store_true", help="Launch in Development mode")
    parser.add_argument("--field", "--prod", dest="field", action="store_true", help="Launch in Field Deployment mode")
    parser.add_argument("--stop", action="store_true", help="Stop all running Core nodes and kill the tmux session")

    args = parser.parse_args()

    config = load_config()
    session_name = config.get("session_name", "rov_x19_core")

    if args.stop:
        stop_session(session_name)
        return

    if args.field:
        mode = "field"
    elif args.dev:
        mode = "dev"
    else:
        mode = config.get("default_mode", "dev")

    launch_session(mode)


if __name__ == "__main__":
    main()
