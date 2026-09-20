"""Linux /proc based CPU and RSS monitor with no optional dependency."""

import os
from pathlib import Path
import threading
import time


def _descendants(root):
    parents = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().split()
            parents[int(entry.name)] = int(fields[3])
        except (FileNotFoundError, IndexError, PermissionError, ValueError):
            pass
    found = {root}
    changed = True
    while changed:
        changed = False
        for pid, parent in parents.items():
            if parent in found and pid not in found:
                found.add(pid)
                changed = True
    return found


class ResourceMonitor:
    def __init__(self, root_pid, interval=0.5):
        self.root_pid = root_pid
        self.interval = interval
        self.samples = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._clock_ticks = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
        self._page_size = os.sysconf('SC_PAGE_SIZE')

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _sample(self):
        ticks = rss = 0
        live = set()
        for pid in _descendants(self.root_pid):
            try:
                stat = (Path('/proc') / str(pid) / 'stat').read_text().split()
                ticks += int(stat[13]) + int(stat[14])
                rss += int(stat[23]) * self._page_size
                live.add(pid)
            except (FileNotFoundError, IndexError, PermissionError, ValueError):
                pass
        return ticks / self._clock_ticks, rss / (1024 * 1024), live

    def _run(self):
        previous_cpu = previous_time = None
        while not self._stop.wait(self.interval):
            now = time.monotonic()
            cpu, rss, live = self._sample()
            percent = 0.0 if previous_cpu is None else (
                100.0 * (cpu - previous_cpu) / max(now - previous_time, 1e-6))
            self.samples.append((percent, rss, live))
            previous_cpu, previous_time = cpu, now

    @property
    def cpu_mean(self):
        return sum(s[0] for s in self.samples) / max(len(self.samples), 1)

    @property
    def cpu_peak(self):
        return max((s[0] for s in self.samples), default=0.0)

    @property
    def memory_peak_mb(self):
        return max((s[1] for s in self.samples), default=0.0)

    @property
    def remaining_pids(self):
        return sorted(_descendants(self.root_pid) if Path(f'/proc/{self.root_pid}').exists()
                      else set())
