"""Passive ODrive S1 CAN monitoring: fault detection, voltage tracking, history.

This module only interprets frames handed to it. CanReader is a receive-only
SocketCAN socket; nothing in this module ever transmits on the bus.
"""
import collections
import math
import socket
import struct
import time

from obot import odrive_s1_codes as codes

NODE_NAMES = {0: 'left', 1: 'right'}
CAN_FRAME_FMT = '=IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


class _Node:
    def __init__(self, history_len):
        self.hb = None
        self.hb_t = None
        self.err = None
        self.err_t = None
        self.volt = None
        self.volt_t = None
        self.current_a = None
        self.temp = None
        self.temp_t = None
        self.history = collections.deque(maxlen=history_len)
        self.sess_min = None
        self.sess_max = None
        self.all_min = None
        self.all_max = None
        self.all_min_epoch = None
        self.all_max_epoch = None
        self.last_fault_mask = 0
        self.open_fault_id = None
        self.open_fault_has_detail = False
        self.fault_count = 0
        self.last_fault_epoch = None
        self.pending = []


class OdriveMonitor:
    def __init__(self, store, node_ids=(0, 1), history_len=3000, pre_s=10.0,
                 post_s=10.0, stale_s=3.0, voltage_range=(0.0, 80.0),
                 extrema_save_period_s=10.0, max_window_samples=400):
        self.store = store
        self.node_ids = tuple(node_ids)
        self.pre_s = pre_s
        self.post_s = post_s
        self.stale_s = stale_s
        self.vmin_valid, self.vmax_valid = voltage_range
        self.save_period = extrema_save_period_s
        self.max_window_samples = max_window_samples
        self.nodes = {n: _Node(history_len) for n in self.node_ids}
        self.session_reset_epoch = 0.0
        self._dirty = False
        self._last_save = None
        self._restore()

    def _restore(self):
        stored = self.store.load_extrema()
        for n, node in self.nodes.items():
            e = stored.get(n)
            if e:
                node.all_min, node.all_max = e['min'], e['max']
                node.all_min_epoch, node.all_max_epoch = e['min_epoch'], e['max_epoch']
        for n, s in self.store.fault_summary().items():
            if n in self.nodes:
                self.nodes[n].fault_count = s['count']
                self.nodes[n].last_fault_epoch = s['last_epoch']

    # ---- frame input -------------------------------------------------
    def handle_frame(self, can_id, data, t, epoch, rtr=False):
        """Process one received frame. t is monotonic seconds, epoch wall time."""
        if rtr:
            return
        node_id, cmd = codes.split_arbitration_id(can_id)
        node = self.nodes.get(node_id)
        decoder = codes.DECODERS.get(cmd)
        if node is None or decoder is None:
            return
        d = decoder(bytes(data))
        if d is None:
            return
        if cmd == codes.CMD_HEARTBEAT:
            node.hb, node.hb_t = d, t
            self._check_fault(node_id, t, epoch)
        elif cmd == codes.CMD_GET_ERROR:
            node.err, node.err_t = d, t
            self._check_fault(node_id, t, epoch)
        elif cmd == codes.CMD_GET_TEMPERATURE:
            node.temp = d
            node.temp_t = t
        elif cmd == codes.CMD_GET_BUS_VOLTAGE_CURRENT:
            node.current_a = d['bus_current_a']
            self._voltage(node_id, d['bus_voltage_v'], t, epoch)

    # ---- voltage -----------------------------------------------------
    def _valid_v(self, v):
        return (isinstance(v, float) and not math.isnan(v)
                and self.vmin_valid < v <= self.vmax_valid)

    def _voltage(self, n, v, t, epoch):
        if not self._valid_v(v):
            return
        node = self.nodes[n]
        node.volt, node.volt_t = v, t
        node.history.append((t, v))
        for p in node.pending:
            if len(p['post']) < self.max_window_samples:
                p['post'].append((round(t - p['t0'], 3), v))
        if node.sess_min is None or v < node.sess_min:
            node.sess_min = v
        if node.sess_max is None or v > node.sess_max:
            node.sess_max = v
        if node.all_min is None or v < node.all_min:
            node.all_min, node.all_min_epoch = v, epoch
            self._dirty = True
        if node.all_max is None or v > node.all_max:
            node.all_max, node.all_max_epoch = v, epoch
            self._dirty = True

    def voltage_now(self, n, t):
        node = self.nodes[n]
        if node.volt is None or t - node.volt_t > self.stale_s:
            return None
        return node.volt

    # ---- faults ------------------------------------------------------
    def combined_mask(self, n, t):
        """Heartbeat Axis_Error (active|disarm) OR fresh Get_Error active errors."""
        node = self.nodes[n]
        mask = None
        if node.hb is not None and t - node.hb_t <= self.stale_s:
            mask = node.hb['axis_error']
        if node.err is not None and t - node.err_t <= self.stale_s:
            mask = (mask or 0) | node.err['active_errors']
        return mask

    def _check_fault(self, n, t, epoch):
        node = self.nodes[n]
        mask = self.combined_mask(n, t)
        if mask is None:
            return
        fmask = mask & ~codes.BENIGN_ERROR_MASK
        new_bits = fmask & ~node.last_fault_mask
        if new_bits:
            self._log_fault(n, node, mask, new_bits, t, epoch)
        elif (fmask and node.open_fault_id and not node.open_fault_has_detail
              and node.err is not None):
            self._log_detail(n, node, epoch)
        if fmask == 0 and node.last_fault_mask:
            self._log_recovery(n, node, t, epoch)
        node.last_fault_mask = fmask

    def _code_block(self, node):
        active = node.err['active_errors'] if node.err else None
        disarm = node.err['disarm_reason'] if node.err else None
        return {
            'active_errors': active,
            'active_errors_hex': None if active is None else '0x%08X' % active,
            'active_errors_decoded': (None if active is None
                                      else codes.decode_error_mask(active)),
            'disarm_reason': disarm,
            'disarm_reason_hex': None if disarm is None else '0x%08X' % disarm,
            'disarm_reason_decoded': (None if disarm is None
                                      else codes.decode_error_mask(disarm)),
        }

    def _log_fault(self, n, node, mask, new_bits, t, epoch):
        fault_id = '%s-%d-%d' % (NODE_NAMES.get(n, n), n, int(epoch * 1000))
        hb = node.hb or {}
        pre = [(round(ts - t, 3), v) for ts, v in node.history
               if t - ts <= self.pre_s][-self.max_window_samples:]
        volt = self.voltage_now(n, t)
        rec = {
            'type': 'fault', 'fault_id': fault_id, 'node': n,
            'controller': NODE_NAMES.get(n, str(n)),
            't_epoch': epoch, 't_iso': time.strftime(
                '%Y-%m-%dT%H:%M:%S', time.localtime(epoch)),
            'heartbeat_axis_error': hb.get('axis_error'),
            'heartbeat_axis_error_hex': (None if 'axis_error' not in hb
                                         else '0x%08X' % hb['axis_error']),
            'combined_mask': mask,
            'combined_mask_hex': '0x%08X' % mask,
            'new_bits': new_bits,
            'new_bits_hex': '0x%08X' % new_bits,
            'combined_decoded': codes.decode_error_mask(mask),
            'axis_state': hb.get('axis_state'),
            'axis_state_name': (None if 'axis_state' not in hb
                                else codes.axis_state_name(hb['axis_state'])),
            'procedure_result': hb.get('procedure_result'),
            'procedure_result_name': (
                None if 'procedure_result' not in hb
                else codes.procedure_result_name(hb['procedure_result'])),
            'bus_voltage_v': volt,
            'bus_voltage_age_s': (None if node.volt_t is None
                                  else round(t - node.volt_t, 3)),
            'pre_fault_min_v': min((v for _, v in pre), default=None),
            'pre_fault_max_v': max((v for _, v in pre), default=None),
        }
        rec.update(self._code_block(node))
        self.store.append_fault_record(rec)
        node.fault_count += 1
        node.last_fault_epoch = epoch
        node.open_fault_id = fault_id
        node.open_fault_has_detail = node.err is not None
        node.pending.append({'fault_id': fault_id, 'node': n, 't0': t,
                             't_end': t + self.post_s, 'epoch': epoch,
                             'pre': pre, 'post': []})
        self._save(epoch)

    def _log_detail(self, n, node, epoch):
        rec = {'type': 'fault_detail', 'fault_id': node.open_fault_id,
               'node': n, 't_epoch': epoch}
        rec.update(self._code_block(node))
        self.store.append_fault_record(rec)
        node.open_fault_has_detail = True

    def _log_recovery(self, n, node, t, epoch):
        self.store.append_fault_record({
            'type': 'recovery', 'fault_id': node.open_fault_id, 'node': n,
            'controller': NODE_NAMES.get(n, str(n)), 't_epoch': epoch,
            'bus_voltage_v': self.voltage_now(n, t)})
        node.open_fault_id = None
        node.open_fault_has_detail = False

    # ---- periodic ----------------------------------------------------
    def tick(self, t, epoch):
        """Finalize post-fault windows and rate-limited extrema saving."""
        for n, node in self.nodes.items():
            keep = []
            for p in node.pending:
                if t >= p['t_end']:
                    self.store.append_fault_record({
                        'type': 'fault_window', 'fault_id': p['fault_id'],
                        'node': n, 't_epoch': p['epoch'],
                        'samples_relative_to_fault_s': {
                            'pre': p['pre'], 'post': p['post']}})
                else:
                    keep.append(p)
            node.pending = keep
        if self._dirty and (self._last_save is None
                            or t - self._last_save >= self.save_period):
            self._last_save = t
            self._save(epoch)

    def flush(self, t, epoch):
        """Write everything pending (used at shutdown)."""
        for n, node in self.nodes.items():
            for p in node.pending:
                self.store.append_fault_record({
                    'type': 'fault_window', 'fault_id': p['fault_id'],
                    'node': n, 't_epoch': p['epoch'], 'truncated': True,
                    'samples_relative_to_fault_s': {
                        'pre': p['pre'], 'post': p['post']}})
            node.pending = []
        self._save(epoch)

    def _save(self, epoch):
        self.store.save_extrema({
            n: {'min': nd.all_min, 'max': nd.all_max,
                'min_epoch': nd.all_min_epoch, 'max_epoch': nd.all_max_epoch}
            for n, nd in self.nodes.items()}, epoch)
        self._dirty = False

    # ---- session reset -----------------------------------------------
    def reset_session_extrema(self, epoch):
        """Reset session min/max only. All-time extrema and logs are untouched."""
        for node in self.nodes.values():
            node.sess_min = node.sess_max = None
            cur = node.volt
            if cur is not None:
                node.sess_min = node.sess_max = cur
        self.session_reset_epoch = epoch
        self.store.append_event({'type': 'session_extrema_reset',
                                 't_epoch': epoch})

    # ---- snapshot ----------------------------------------------------
    def snapshot(self, n, t):
        node = self.nodes[n]
        nan = float('nan')
        hb_fresh = node.hb is not None and t - node.hb_t <= self.stale_s
        mask = self.combined_mask(n, t)
        err_fresh = node.err is not None and t - node.err_t <= self.stale_s
        v = self.voltage_now(n, t)
        temp = nan
        if node.temp is not None and t - node.temp_t <= self.stale_s * 3:
            temp = max(node.temp['fet_temp_c'], node.temp['motor_temp_c'])

        def f(x):
            return nan if x is None else x
        return {
            'heartbeat_seen': node.hb is not None,
            'heartbeat_fresh': hb_fresh,
            'axis_state': node.hb['axis_state'] if hb_fresh else 0,
            'mask': mask,
            'active_errors': (node.err['active_errors'] if err_fresh
                              else (mask or 0)),
            'disarm_reason': (node.err['disarm_reason'] if err_fresh
                              else codes.CODE_UNAVAILABLE),
            'voltage': f(v),
            'sess_min': f(node.sess_min), 'sess_max': f(node.sess_max),
            'all_min': f(node.all_min), 'all_max': f(node.all_max),
            'temp_c': temp,
            'fault_count': node.fault_count,
            'last_fault_epoch': node.last_fault_epoch,
        }


class CanReader:
    """Receive-only raw SocketCAN reader. It never calls send."""

    def __init__(self, interface='can0'):
        self.interface = interface
        self.sock = None

    def open(self):
        s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        filters = b''
        for n in (0, 1):
            for cmd in codes.MONITORED_CMDS:
                filters += struct.pack(
                    '=II', codes.arbitration_id(n, cmd), 0x7FF)
        s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 0)
        s.bind((self.interface,))
        s.setblocking(False)
        self.sock = s

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def read_all(self):
        """Return [(can_id, data, rtr)] for every frame currently queued."""
        out = []
        while self.sock is not None:
            try:
                raw = self.sock.recv(CAN_FRAME_SIZE)
            except BlockingIOError:
                break
            if len(raw) < CAN_FRAME_SIZE:
                break
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
            if can_id & codes.CAN_ERR_FLAG:
                continue
            out.append((can_id & 0x7FF, data[:dlc],
                        bool(can_id & codes.CAN_RTR_FLAG)))
        return out
