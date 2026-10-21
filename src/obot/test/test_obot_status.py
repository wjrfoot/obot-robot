import json
import math
import os
import struct

from obot import obot_status_lib as lib
from obot import odrive_s1_codes as codes
from obot.obot_fault_store import FaultStore
from obot.obot_odrive_monitor import OdriveMonitor

HB = codes.CMD_HEARTBEAT


def hb(err=0, state=8, res=0):
    return struct.pack('<IBBB', err, state, res, 0) + b'\0'


def volt(v, a=1.0):
    return struct.pack('<ff', v, a)


def mk(tmp_path, **kw):
    store = FaultStore(str(tmp_path))
    return OdriveMonitor(store, **kw), store


def cid(node, cmd):
    return codes.arbitration_id(node, cmd)


def test_decode_codes():
    assert codes.describe_error_mask(0) == 'NONE'
    names = [e['name'] for e in codes.decode_error_mask(0x100 | 0x2000000)]
    assert names == ['DC_BUS_OVER_VOLTAGE', 'ESTOP_REQUESTED']
    assert codes.decode_error_mask(1 << 20)[0]['name'].startswith('UNKNOWN_')
    assert codes.split_arbitration_id(0x037) == (1, 0x17)
    assert codes.decode_get_error(struct.pack('<II', 5, 6)) == {
        'active_errors': 5, 'disarm_reason': 6}
    assert codes.decode_heartbeat(b'\0' * 3) is None


def test_fault_logged_with_codes_and_voltage(tmp_path):
    m, store = mk(tmp_path)
    m.handle_frame(cid(1, 0x17), volt(24.0), 10.0, 1000.0)
    m.handle_frame(cid(1, 0x03), struct.pack('<II', 0x200, 0x200), 10.5, 1000.5)
    m.handle_frame(cid(1, HB), hb(0x200), 11.0, 1001.0)
    recs = store.read_records()
    fault = [r for r in recs if r['type'] == 'fault']
    assert len(fault) == 1
    f = fault[0]
    assert f['node'] == 1 and f['controller'] == 'right'
    assert f['active_errors'] == 0x200 and f['disarm_reason'] == 0x200
    assert f['active_errors_decoded'][0]['name'] == 'DC_BUS_UNDER_VOLTAGE'
    assert f['heartbeat_axis_error'] == 0x200 or f['heartbeat_axis_error'] is None
    assert f['bus_voltage_v'] == 24.0
    assert f['t_epoch'] == 1000.5 or f['t_epoch'] == 1001.0
    # repeated identical frames do not create duplicate faults
    m.handle_frame(cid(1, HB), hb(0x200), 11.1, 1001.1)
    assert len([r for r in store.read_records() if r['type'] == 'fault']) == 1
    assert m.nodes[1].fault_count == 1 and m.nodes[0].fault_count == 0


def test_heartbeat_only_fault_then_detail(tmp_path):
    m, store = mk(tmp_path)
    m.handle_frame(cid(0, HB), hb(0x4000), 1.0, 100.0)
    f = [r for r in store.read_records() if r['type'] == 'fault'][0]
    assert f['active_errors'] is None and f['heartbeat_axis_error'] == 0x4000
    m.handle_frame(cid(0, 0x03), struct.pack('<II', 0x4000, 0x4000), 2.0, 101.0)
    types = [r['type'] for r in store.read_records()]
    assert types == ['fault', 'fault_detail']
    d = store.read_records()[1]
    assert d['fault_id'] == f['fault_id'] and d['disarm_reason'] == 0x4000


def test_benign_initializing_not_a_fault_and_recovery(tmp_path):
    m, store = mk(tmp_path)
    m.handle_frame(cid(0, HB), hb(0x1), 1.0, 100.0)
    assert store.read_records() == []
    m.handle_frame(cid(0, HB), hb(0x2000), 2.0, 101.0)
    m.handle_frame(cid(0, HB), hb(0), 3.0, 102.0)
    assert [r['type'] for r in store.read_records()] == ['fault', 'recovery']


def test_rtr_and_unknown_ignored(tmp_path):
    m, store = mk(tmp_path)
    m.handle_frame(cid(0, HB), b'', 1.0, 1.0, rtr=True)
    m.handle_frame(cid(5, HB), hb(2), 1.0, 1.0)
    m.handle_frame(cid(0, 0x0B), b'12345678', 1.0, 1.0)
    assert m.nodes[0].hb is None and store.read_records() == []


def test_voltage_extrema_independent_and_invalid_ignored(tmp_path):
    m, _ = mk(tmp_path)
    for t, (n, v) in enumerate([(0, 24.0), (0, 22.0), (1, 25.0), (0, 26.0),
                                (1, float('nan')), (1, 0.0), (1, 500.0)]):
        m.handle_frame(cid(n, 0x17), volt(v), float(t), 100.0 + t)
    s0, s1 = m.snapshot(0, 4.0), m.snapshot(1, 4.0)
    assert (s0['sess_min'], s0['sess_max']) == (22.0, 26.0)
    assert (s1['sess_min'], s1['sess_max']) == (25.0, 25.0)
    assert s1['voltage'] == 25.0


def test_stale_voltage_is_nan(tmp_path):
    m, _ = mk(tmp_path, stale_s=3.0)
    m.handle_frame(cid(0, 0x17), volt(24.0), 1.0, 1.0)
    assert m.snapshot(0, 2.0)['voltage'] == 24.0
    assert math.isnan(m.snapshot(0, 10.0)['voltage'])
    assert math.isnan(m.snapshot(1, 10.0)['voltage'])


def test_windows_and_bounded_history(tmp_path):
    m, store = mk(tmp_path, history_len=50, pre_s=5.0, post_s=2.0)
    t = 0.0
    while t < 100:
        m.handle_frame(cid(0, 0x17), volt(24.0 - t / 100), t, 1000 + t)
        t += 0.1
    assert len(m.nodes[0].history) == 50
    m.handle_frame(cid(0, HB), hb(0x200), 100.0, 1100.0)
    m.handle_frame(cid(0, 0x17), volt(20.0), 100.5, 1100.5)
    m.tick(103.0, 1103.0)
    win = [r for r in store.read_records() if r['type'] == 'fault_window'][0]
    pre = win['samples_relative_to_fault_s']['pre']
    post = win['samples_relative_to_fault_s']['post']
    assert pre and all(-5.0 <= s <= 0 for s, _ in pre)
    assert post == [[0.5, 20.0]]


def test_extrema_persist_and_reset_keeps_history(tmp_path):
    m, store = mk(tmp_path, extrema_save_period_s=0.0)
    m.handle_frame(cid(0, 0x17), volt(21.0), 1.0, 100.0)
    m.handle_frame(cid(0, 0x17), volt(27.0), 2.0, 101.0)
    m.handle_frame(cid(0, HB), hb(0x200), 3.0, 102.0)
    m.tick(4.0, 103.0)
    m.flush(5.0, 104.0)
    faults_before = open(store.faults_path).read()
    m.reset_session_extrema(200.0)
    assert m.snapshot(0, 4.0)['sess_min'] == 27.0   # reseeded with current only
    assert open(store.faults_path).read().startswith(faults_before)
    ev = store.read_records(store.events_path)
    assert ev[-1]['type'] == 'session_extrema_reset'
    # restart: all-time extrema and fault counts are restored, session is fresh
    m2, _ = mk(tmp_path)
    s = m2.snapshot(0, 0.0)
    assert s['all_min'] == 21.0 and s['all_max'] == 27.0
    assert math.isnan(s['sess_min']) and s['fault_count'] == 1
    assert s['last_fault_epoch'] == 102.0


def test_corrupt_extrema_file(tmp_path):
    store = FaultStore(str(tmp_path))
    with open(store.extrema_path, 'w') as f:
        f.write('{not json')
    assert store.load_extrema() == {}
    assert any('corrupt' in n for n in os.listdir(str(tmp_path)))


def test_log_is_strict_json_with_nan(tmp_path):
    store = FaultStore(str(tmp_path))
    store.append_fault_record({'type': 'x', 'v': float('nan')})
    assert json.loads(open(store.faults_path).readline())['v'] is None


def test_pi_metrics():
    assert lib.pi_temperature_c('x', lambda p: '52300\n') == 52.3
    assert math.isnan(lib.pi_temperature_c('x', lambda p: 'bad'))
    texts = iter(['cpu  100 0 100 800 0 0 0 0\n', 'cpu  200 0 200 1600 0 0 0 0\n'])
    meter = lib.CpuMeter(reader=lambda p: next(texts))
    assert math.isnan(meter.sample())
    assert abs(meter.sample() - 20.0) < 1e-6
    assert lib.read_throttled(lambda c: 'throttled=0x50005\n') == (True, 0x50005)
    assert lib.read_throttled(lambda c: (_ for _ in ()).throw(FileNotFoundError())) \
        == (False, 0)
    d = lib.decode_throttled(0x50005)
    assert d['undervoltage_now'] and d['throttled_now']
    assert d['undervoltage_since_boot'] and d['throttled_since_boot']
    assert not lib.decode_throttled(0)['throttled_since_boot']


def test_freshness():
    f = lib.Freshness({'scan': 2.0})
    assert f.state('scan', 0) == lib.UNKNOWN
    f.mark('scan', 1.0)
    assert f.state('scan', 2.5) == lib.OK
    assert f.state('scan', 3.5) == lib.STALE


def test_lifecycle_mapping():
    a, i = lib.LC_ACTIVE, lib.LC_INACTIVE
    assert lib.map_lifecycle({'a': a, 'b': a}) == lib.OK
    assert lib.map_lifecycle({'a': i, 'b': lib.LC_UNCONFIGURED}) == lib.WARN
    assert lib.map_lifecycle({'a': a, 'b': lib.LC_FINALIZED}) == lib.FAULT
    assert lib.map_lifecycle({'a': a, 'b': None}) == lib.FAULT
    assert lib.map_lifecycle({'a': None, 'b': None}) == lib.UNKNOWN


CAN_OK = ('3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16\n    can state '
          'ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100\n')


def test_can_iface_parse():
    assert lib.parse_can_iface(CAN_OK) == (lib.OK, 1, 0, 0)
    warn = CAN_OK.replace('ERROR-ACTIVE (berr-counter tx 0 rx 0)',
                          'ERROR-WARNING (berr-counter tx 100 rx 3)')
    assert lib.parse_can_iface(warn) == (lib.WARN, 2, 3, 100)
    assert lib.parse_can_iface(CAN_OK.replace('ERROR-ACTIVE', 'BUS-OFF'))[0] == lib.FAULT
    assert lib.parse_can_iface(CAN_OK.replace('UP,', ''))[0] == lib.FAULT
    assert lib.parse_can_iface('')[0] == lib.FAULT


def test_no_can_transmit_in_source():
    base = os.path.dirname(os.path.dirname(os.path.abspath(codes.__file__)))
    forbidden = ('.send(', '.sendall(', '.sendto(', 'cansend', 'cangen',
                 'SOCK_DGRAM', 'sdo_write', 'set_input')
    for name in ('obot_odrive_monitor.py', 'obot_status_node.py',
                 'obot_status_lib.py', 'odrive_s1_codes.py',
                 'obot_fault_store.py'):
        text = open(os.path.join(base, 'obot', name)).read()
        for word in forbidden:
            assert word not in text, (name, word)
