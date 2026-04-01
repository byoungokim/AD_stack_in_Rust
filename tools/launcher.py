#!/usr/bin/env python3
"""Process launcher and supervisor for Limo Drive.

Launches the 3 main processes, monitors health, and handles restarts.

Usage:
    python tools/launcher.py config/system.yaml
    python tools/launcher.py config/system.yaml --sim
    python tools/launcher.py config/system.yaml --mode e2e
"""

import argparse
import logging
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Optional

import yaml

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s"
)
logger = logging.getLogger("launcher")

PROJECT_ROOT = Path(__file__).resolve().parent.parent


class ProcessManager:
    """Manages child processes with health monitoring and restart."""

    def __init__(self, config: dict) -> None:
        self.config = config
        self.processes: Dict[str, subprocess.Popen] = {}
        self.restart_counts: Dict[str, int] = {}
        self._running = True

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def launch_all(self) -> None:
        """Launch all enabled processes."""
        proc_config = self.config.get("processes", {})
        for name, cfg in proc_config.items():
            if cfg.get("enabled", True):
                self._launch_process(name)

    def _launch_process(self, name: str) -> None:
        """Launch a single process by name."""
        # Determine the executable/script for each process
        cmd = self._get_command(name)
        if not cmd:
            logger.error("No command found for process '%s'", name)
            return

        logger.info("Launching process '%s': %s", name, " ".join(cmd))
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=str(PROJECT_ROOT),
                env={**os.environ, "LIMO_PROCESS": name},
            )
            self.processes[name] = proc
            self.restart_counts.setdefault(name, 0)
            logger.info("Process '%s' started (PID %d)", name, proc.pid)
        except Exception:
            logger.exception("Failed to launch process '%s'", name)

    def _get_command(self, name: str) -> Optional[list]:
        """Get the launch command for a process."""
        build_dir = PROJECT_ROOT / "build"
        commands = {
            "sensperc": [sys.executable, "-m", "sensperc.main"],
            "planning": [sys.executable, "-m", "planning.main"],
            "control": [str(build_dir / "limo_control")],
        }
        return commands.get(name)

    def monitor(self) -> None:
        """Monitor processes and restart if needed."""
        proc_config = self.config.get("processes", {})

        while self._running:
            for name, proc in list(self.processes.items()):
                retcode = proc.poll()
                if retcode is not None:
                    logger.warning(
                        "Process '%s' exited with code %d", name, retcode)
                    cfg = proc_config.get(name, {})
                    if cfg.get("auto_restart", False) and self._running:
                        delay = cfg.get("restart_delay_sec", 2.0)
                        self.restart_counts[name] += 1
                        logger.info(
                            "Restarting '%s' in %.1fs (restart #%d)",
                            name, delay, self.restart_counts[name])
                        time.sleep(delay)
                        if self._running:
                            self._launch_process(name)
            time.sleep(0.5)

    def shutdown(self) -> None:
        """Gracefully stop all processes."""
        logger.info("Shutting down all processes...")
        self._running = False

        for name, proc in self.processes.items():
            if proc.poll() is None:
                logger.info("Sending SIGTERM to '%s' (PID %d)",
                            name, proc.pid)
                proc.terminate()

        # Wait for graceful shutdown
        for name, proc in self.processes.items():
            try:
                proc.wait(timeout=5.0)
                logger.info("Process '%s' stopped", name)
            except subprocess.TimeoutExpired:
                logger.warning("Force killing '%s'", name)
                proc.kill()

    def _signal_handler(self, signum: int, frame) -> None:
        logger.info("Received signal %d", signum)
        self.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Limo Drive Launcher")
    parser.add_argument("config", help="Path to system.yaml")
    parser.add_argument("--sim", action="store_true",
                        help="Launch in simulation mode")
    parser.add_argument("--mode", choices=["traditional", "e2e", "shadow"],
                        default="traditional",
                        help="Pipeline mode (default: traditional)")
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    config["pipeline_mode"] = args.mode.upper()
    config["simulation"] = args.sim

    logger.info("Limo Drive Launcher — mode=%s, sim=%s",
                config["pipeline_mode"], config["simulation"])

    manager = ProcessManager(config)
    manager.launch_all()
    manager.monitor()


if __name__ == "__main__":
    main()
