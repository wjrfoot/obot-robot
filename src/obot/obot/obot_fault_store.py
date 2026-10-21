"""Persistent storage for OBot fault events and voltage extrema.

Files in the log directory (default ~/obot_logs):
  faults.jsonl   append-only fault/fault_detail/fault_window/recovery records
  events.jsonl   append-only operator events (session extrema resets, starts)
  extrema.json   all-time bus voltage min/max per controller (atomic rewrite)

Every append is flushed and fsync'd so a power loss does not silently drop
the only record of a fault. Existing records are never deleted or rewritten.
"""
import json
import os
import time


def _fsync_dir(path):
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(fd)
    except OSError:
        pass
    finally:
        os.close(fd)


class FaultStore:
    def __init__(self, log_dir, logger=None):
        self.log_dir = os.path.expanduser(log_dir)
        self._logger = logger
        os.makedirs(self.log_dir, exist_ok=True)
        self.faults_path = os.path.join(self.log_dir, 'faults.jsonl')
        self.events_path = os.path.join(self.log_dir, 'events.jsonl')
        self.extrema_path = os.path.join(self.log_dir, 'extrema.json')

    def _warn(self, text):
        if self._logger is not None:
            self._logger.warn(text)

    @staticmethod
    def _append(path, record):
        line = json.dumps(record, sort_keys=True, allow_nan=False) + '\n'
        with open(path, 'a', encoding='utf-8') as f:
            f.write(line)
            f.flush()
            os.fsync(f.fileno())

    @staticmethod
    def _clean(value):
        """Replace NaN/inf with None so records are strict JSON."""
        if isinstance(value, float):
            return value if value == value and abs(value) != float('inf') else None
        if isinstance(value, dict):
            return {k: FaultStore._clean(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [FaultStore._clean(v) for v in value]
        return value

    def append_fault_record(self, record):
        self._append(self.faults_path, self._clean(record))

    def append_event(self, record):
        self._append(self.events_path, self._clean(record))

    def read_records(self, path=None):
        """Return every valid JSON record; malformed lines are skipped."""
        path = path or self.faults_path
        records = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        records.append(json.loads(line))
                    except ValueError:
                        self._warn('Skipping malformed record in %s' % path)
        except FileNotFoundError:
            pass
        return records

    def fault_summary(self):
        """Return {node_id: {'count': n, 'last_epoch': t or None}} from faults.jsonl."""
        summary = {}
        for rec in self.read_records():
            if rec.get('type') != 'fault':
                continue
            node = rec.get('node')
            entry = summary.setdefault(node, {'count': 0, 'last_epoch': None})
            entry['count'] += 1
            t = rec.get('t_epoch')
            if t is not None and (entry['last_epoch'] is None or t > entry['last_epoch']):
                entry['last_epoch'] = t
        return summary

    def load_extrema(self):
        """Return {node_id: {'min','max','min_epoch','max_epoch'}} (may be empty)."""
        try:
            with open(self.extrema_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            out = {}
            for key, val in data['nodes'].items():
                out[int(key)] = {
                    'min': val.get('alltime_min_v'),
                    'max': val.get('alltime_max_v'),
                    'min_epoch': val.get('alltime_min_epoch'),
                    'max_epoch': val.get('alltime_max_epoch'),
                }
            return out
        except FileNotFoundError:
            return {}
        except (ValueError, KeyError, TypeError, AttributeError):
            corrupt = self.extrema_path + '.corrupt.%d' % int(time.time())
            try:
                os.replace(self.extrema_path, corrupt)
            except OSError:
                pass
            self._warn('extrema.json was unreadable; kept as %s and starting fresh' % corrupt)
            return {}

    def save_extrema(self, extrema, t_epoch):
        """Atomically rewrite extrema.json (temp file, fsync, rename)."""
        nodes = {}
        for node, val in extrema.items():
            nodes[str(node)] = {
                'alltime_min_v': val.get('min'),
                'alltime_max_v': val.get('max'),
                'alltime_min_epoch': val.get('min_epoch'),
                'alltime_max_epoch': val.get('max_epoch'),
            }
        payload = self._clean({'version': 1, 'saved_epoch': t_epoch, 'nodes': nodes})
        tmp = self.extrema_path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(payload, f, sort_keys=True, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.extrema_path)
        _fsync_dir(self.log_dir)
