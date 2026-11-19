# msg_log.py
import json, time, threading

class MsgLog:
    """Writes one JSON object per line: easy to read, easy to load with pandas later."""
    def __init__(self, path="robot_log.jsonl"):
        self._f = open(path, "a", encoding="utf-8")
        self._lock = threading.Lock()   # so two threads never write half a line each

    def write(self, direction, msg):
        # direction: "out" (laptop -> robot), "in" (robot -> laptop), or "event"
        rec = {"t": time.time(), "dir": direction, "msg": msg}
        with self._lock:
            self._f.write(json.dumps(rec) + "\n")
            self._f.flush()             # flush so the log survives a crash

    def close(self):
        self._f.close()