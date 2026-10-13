"""code.py for the gimbal's Feather (CircuitPython 10): 8-channel servo console over USB serial.

Pair it with examples/servo_console.html (Web Serial, Chrome/Edge). Plain serial terminals work
too: one command per line, replies are JSON lines.

Startup never writes PWM. The PCA9685 is attached without a reset, so outputs held from a
previous session stay as they were. Channels 0-3 are the gimbal (tilt / cam / roll / pan);
4-7 are spare.

Commands (ch = 0-7 or ALL where noted; us = pulse width in microseconds):
  PING | STATUS | HELP
  US ch us                 jump to a pulse        DUTY ch percent       jump to a duty (% of period)
  ANGLE ch deg             angle via the channel's mapping (cam: degrees from rest)
  MOVE ch us rate          ramp to a pulse at rate us/s
  CENTER ch|ALL            neutral / home pulse   REST ch|ALL           rest pulse (cam: 500 us)
  OFF ch|ALL               output fully off (no holding torque)
  LIMITS ch ON|OFF         toggle software end stops     SETLIMITS ch min_us max_us
  SWEEP ch start_us end_us rate [cycles]   back-and-forth sweep at rate us/s (cycles 0 = forever)
  STOP [ch|ALL]            stop sweeps / sequences (outputs hold where they are)
  SEQ name                 run a sequence (see SEQS)  SEQS  list sequences
  INTERLOCK ON|OFF         block tilt/roll moves unless the hammer cam is parked
"""

import json
import sys
import time

import board
import supervisor

ADDRESS = 0x40
OSC_HZ = 25_000_000
TARGET_HZ = 50
ABS_MIN, ABS_MAX = 400, 2600

# Channels 0-3 follow examples/gimbal_servos.py and docs/servo_bench_2026-10-11.md (held trims).
# label: accelerometer-style axis naming (roll = X, tilt = Y, pan = Z; the cam drives the tap).
CH = [
    dict(name="tilt", label="Y - tilt (pitch)", center=1522, rest=1522, lo=500, hi=2500, deg=180, cam=False),
    dict(name="cam", label="Tap - hammer cam (MG995)", center=1500, rest=500, lo=500, hi=2500, deg=180, cam=True),
    dict(name="roll", label="X - roll", center=1492, rest=1492, lo=500, hi=2400, deg=180, cam=False),
    dict(name="pan", label="Z - pan (yaw)", center=1500, rest=1500, lo=500, hi=2500, deg=270, cam=False),
]
for i in range(4, 8):
    CH.append(dict(name="ch%d" % i, label="Spare %d" % i, center=1500, rest=1500, lo=500, hi=2500, deg=180, cam=False))
for i, c in enumerate(CH):
    c["limits"] = i < 4  # software end stops on for the gimbal channels, off for spares

# Hammer cam pulses (MG995 model, uncalibrated: us = 500 + angle * 2000/180), docs/servo_control.md
CAM_REST, CAM_PARK, CAM_SINGLE, CAM_DOUBLE = 500, 1267, 1467, 2333
PARK_TOL = 40
interlock = False  # optional guard; never needed for CENTER / REST / OFF
HELP = ("PING STATUS HELP SEQS | US ch us | DUTY ch % | ANGLE ch deg | MOVE ch us rate | "
        "CENTER/REST/OFF ch|ALL | LIMITS ch ON|OFF | SETLIMITS ch min max | "
        "SWEEP ch start end rate [cycles] | STOP [ch] | SEQ name | INTERLOCK ON|OFF | BOOTOFF ON|OFF")


class PCA:
    """Minimal PCA9685 register access that never resets the chip."""

    def __init__(self):
        self.i2c = board.I2C()
        self.buf = bytearray(5)
        mode1 = self.read(0x00)
        prescale = self.read(0xFE)
        want = round(OSC_HZ / (4096 * TARGET_HZ)) - 1
        if mode1 & 0x10 or prescale != want:  # asleep or wrong rate: (re)configure without touching outputs
            self.write(0x00, (mode1 & 0x7F) | 0x10)
            self.write(0xFE, want)
            self.write(0x00, mode1 & 0x6F | 0x20)
            time.sleep(0.001)
            self.write(0x00, (mode1 & 0x6F) | 0xA0)
            prescale = want
        else:
            self.write(0x00, mode1 | 0x20)  # auto-increment
        self.period_us = 4096 * (prescale + 1) * 1e6 / OSC_HZ

    def _lock(self):
        while not self.i2c.try_lock():
            pass

    def read(self, reg):
        self._lock()
        try:
            out = bytearray(1)
            self.i2c.writeto_then_readfrom(ADDRESS, bytes([reg]), out)
            return out[0]
        finally:
            self.i2c.unlock()

    def write(self, reg, value):
        self._lock()
        try:
            self.i2c.writeto(ADDRESS, bytes([reg, value]))
        finally:
            self.i2c.unlock()

    def set_us(self, ch, us):
        b = self.buf
        b[0] = 0x06 + 4 * ch
        if us is None:
            b[1:5] = bytes([0, 0, 0, 0x10])  # full off
        else:
            count = max(0, min(4095, round(us * 4096 / self.period_us)))
            b[1:5] = bytes([0, 0, count & 0xFF, count >> 8])
        self._lock()
        try:
            self.i2c.writeto(ADDRESS, b)
        finally:
            self.i2c.unlock()

    def get_us(self, ch):
        self._lock()
        try:
            out = bytearray(4)
            self.i2c.writeto_then_readfrom(ADDRESS, bytes([0x06 + 4 * ch]), out)
        finally:
            self.i2c.unlock()
        if out[3] & 0x10 or (out[2] | (out[3] & 0x0F) << 8) == 0:
            return None
        return round(((out[2] | (out[3] & 0x0F) << 8) - (out[0] | (out[1] & 0x0F) << 8)) * self.period_us / 4096, 1)


try:
    import microcontroller
    NVM = microcontroller.nvm  # byte 0: 0xA5 = keep held outputs at boot, anything else = all off
except (ImportError, AttributeError):
    NVM = None


def boot_off():
    return NVM is None or NVM[0] != 0xA5


pca = PCA()
if boot_off():
    for _i in range(16):  # every output full-off at boot, just in case
        pca.set_us(_i, None)
pos = [pca.get_us(i) for i in range(8)]  # what each output is actually holding now
ramps = {}  # ch -> [target_us, rate_us_per_s]
sweeps = {}  # ch -> dict(start, end, rate, cycles, legs, toward)
queue = []  # sequence steps: ("move", ch, us, rate|None) / ("wait", seconds) / ("say", text)
wait_until = 0.0


def reply(**kw):
    print(json.dumps(kw))


def clamp_check(ch, us, guard=True):
    c = CH[ch]
    if not ABS_MIN <= us <= ABS_MAX:
        raise ValueError("pulse %d outside %d-%d us" % (us, ABS_MIN, ABS_MAX))
    if c["limits"] and not c["lo"] <= us <= c["hi"]:
        raise ValueError("%s end stops are %d-%d us (LIMITS %d OFF to override)" % (c["name"], c["lo"], c["hi"], ch))
    if guard and interlock and c["name"] in ("tilt", "roll") and us != pos[ch]:
        cam = pos[1]
        if cam is None or abs(cam - CAM_PARK) > PARK_TOL:
            raise ValueError("interlock: park the hammer (SEQ park) before moving tilt/roll")


def put(ch, us, guard=True):
    if us is not None:
        clamp_check(ch, us, guard)
    pca.set_us(ch, us)
    pos[ch] = us


def angle_us(ch, deg):
    c = CH[ch]
    per_deg = (c["hi"] - c["lo"]) / c["deg"]
    return round(c["lo"] + deg * per_deg) if c["cam"] else round(c["center"] + deg * per_deg)


def us_angle(ch, us):
    if us is None:
        return None
    c = CH[ch]
    per_deg = (c["hi"] - c["lo"]) / c["deg"]
    return round(((us - c["lo"]) if c["cam"] else (us - c["center"])) / per_deg, 1)


def status():
    chans = []
    for i, c in enumerate(CH):
        chans.append(dict(ch=i, name=c["name"], label=c["label"], us=pos[i], deg=us_angle(i, pos[i]),
                          duty=None if pos[i] is None else round(100 * pos[i] / pca.period_us, 3),
                          lo=c["lo"], hi=c["hi"], center=c["center"], rest=c["rest"], limits=c["limits"],
                          deg_span=c["deg"], cam=c["cam"], sweeping=i in sweeps, ramping=i in ramps))
    return dict(type="status", period_us=round(pca.period_us, 2), interlock=interlock, busy=bool(queue),
                boot_off=boot_off(), nvm=NVM is not None,
                channels=chans, cam=dict(rest=CAM_REST, park=CAM_PARK, single=CAM_SINGLE, double=CAM_DOUBLE))


SEQS = {
    "park": "hammer: back to rest, then up onto lobe 1 (parked; tilt/roll may move)",
    "single": "hammer: park, then a single tap",
    "double": "hammer: park, then a double tap (one fast move over both lobes)",
    "rest": "hammer: down to rest (tilt and roll must be home)",
    "home": "park, then centre tilt, roll and pan",
    "tour": "park, then pan +-45, tilt +-30, roll +-30 and home (slow)",
    "center_all": "every channel to its centre pulse (horns detached!)",
}


def seq(name):
    m = lambda ch, us, rate=None: queue.append(("move", ch, us, rate))
    w = lambda s: queue.append(("wait", s))
    park = lambda: (m(1, CAM_REST, 600), w(0.3), m(1, CAM_PARK, 500), w(0.4))
    if name == "park":
        park()
    elif name in ("single", "double"):
        park()
        m(1, CAM_SINGLE if name == "single" else CAM_DOUBLE)
        w(0.6)
    elif name == "rest":
        m(0, CH[0]["center"], 300), m(2, CH[2]["center"], 300), w(0.3), m(1, CAM_REST, 600)
    elif name in ("home", "tour"):
        park()
        for ch in (0, 2, 3):
            m(ch, CH[ch]["center"], 300)
        if name == "tour":
            for ch, deg in ((3, 45), (3, -45), (3, 0), (0, 30), (0, -30), (0, 0), (2, 30), (2, -30), (2, 0)):
                m(ch, angle_us(ch, deg), 250)
                w(0.3)
    elif name == "center_all":
        for ch in range(8):
            m(ch, CH[ch]["center"])
    else:
        raise ValueError("unknown sequence; SEQS lists them")
    queue.append(("say", "sequence %s done" % name))


def chans(arg):
    if arg.upper() == "ALL":
        return list(range(8))
    ch = int(arg)
    if not 0 <= ch < 8:
        raise ValueError("channel must be 0-7")
    return [ch]


def stop(chs):
    global wait_until
    for ch in chs:
        sweeps.pop(ch, None)
        ramps.pop(ch, None)
    if len(chs) == 8:
        queue.clear()
        wait_until = 0.0


def handle(line):
    global interlock
    p = line.split()
    cmd = p[0].upper()
    if cmd == "PING":
        return reply(type="pong")
    if cmd in ("STATUS", "?"):
        return reply(**status())
    if cmd == "HELP":
        return reply(type="help", text=HELP)
    if cmd == "SEQS":
        return reply(type="seqs", seqs=SEQS)
    if cmd in ("US", "DUTY", "ANGLE"):
        ch, v = chans(p[1])[0], float(p[2])
        us = v if cmd == "US" else (v * pca.period_us / 100 if cmd == "DUTY" else angle_us(ch, v))
        stop([ch])
        put(ch, round(us, 1))
    elif cmd == "MOVE":
        ch, us, rate = chans(p[1])[0], float(p[2]), float(p[3])
        clamp_check(ch, us)
        sweeps.pop(ch, None)
        ramps[ch] = [us, max(1.0, rate)]
    elif cmd in ("CENTER", "REST", "OFF"):
        chs = chans(p[1] if len(p) > 1 else "ALL")
        stop(chs if len(chs) < 8 else list(range(8)))
        for ch in chs:
            put(ch, None if cmd == "OFF" else CH[ch]["center" if cmd == "CENTER" else "rest"], guard=False)
    elif cmd == "LIMITS":
        for ch in chans(p[1]):
            CH[ch]["limits"] = p[2].upper() == "ON"
    elif cmd == "SETLIMITS":
        ch, lo, hi = chans(p[1])[0], float(p[2]), float(p[3])
        if not ABS_MIN <= lo < hi <= ABS_MAX:
            raise ValueError("limits must satisfy %d <= min < max <= %d" % (ABS_MIN, ABS_MAX))
        CH[ch]["lo"], CH[ch]["hi"] = lo, hi
    elif cmd == "SWEEP":
        ch, a, b, rate = chans(p[1])[0], float(p[2]), float(p[3]), float(p[4])
        cycles = int(p[5]) if len(p) > 5 else 1
        clamp_check(ch, a), clamp_check(ch, b)
        ramps.pop(ch, None)
        put(ch, a) if pos[ch] is None else None
        sweeps[ch] = dict(start=a, end=b, rate=max(1.0, rate), legs=2 * cycles, done=0, toward=b)
        ramps[ch] = [a, max(1.0, rate)]  # first go to the start at the sweep rate
        sweeps[ch]["toward"] = a
    elif cmd == "STOP":
        stop(chans(p[1]) if len(p) > 1 else list(range(8)))
    elif cmd == "SEQ":
        stop(list(range(8)))
        seq(p[1].lower())
    elif cmd == "INTERLOCK":
        interlock = p[1].upper() == "ON"
    elif cmd == "BOOTOFF":
        if NVM is None:
            raise ValueError("no NVM on this board; outputs are always switched off at boot")
        NVM[0] = 0x00 if p[1].upper() == "ON" else 0xA5
    else:
        raise ValueError("unknown command; HELP lists them")
    reply(**status())


def tick(now, dt):
    global wait_until
    for ch in list(ramps):
        target, rate = ramps[ch]
        cur = pos[ch] if pos[ch] is not None else target
        step = rate * dt
        nxt = target if abs(target - cur) <= step else cur + step * (1 if target > cur else -1)
        try:
            put(ch, round(nxt, 1))
        except ValueError as e:
            ramps.pop(ch, None), sweeps.pop(ch, None)
            reply(type="error", error=str(e))
            continue
        if nxt == target:
            ramps.pop(ch, None)
            sw = sweeps.get(ch)
            if sw:
                if sw["toward"] == sw["start"] and sw["done"] == 0 and target == sw["start"]:
                    pass  # arrived at the start; the sweep proper begins now
                else:
                    sw["done"] += 1
                if sw["legs"] and sw["done"] >= sw["legs"]:
                    sweeps.pop(ch)
                    reply(type="info", text="sweep on %d done" % ch)
                else:
                    sw["toward"] = sw["end"] if sw["toward"] == sw["start"] else sw["start"]
                    ramps[ch] = [sw["toward"], sw["rate"]]
    if not queue:
        return
    step = queue[0]
    if step[0] == "hold":  # wait for a sequence ramp to finish
        if step[1] not in ramps:
            queue.pop(0)
        return
    if now < wait_until:
        return
    queue.pop(0)
    if step[0] == "move":
        _, ch, us, rate = step
        try:
            if rate is None:
                put(ch, us)
            else:
                clamp_check(ch, us)
                ramps[ch] = [us, rate]
                queue.insert(0, ("hold", ch))
        except ValueError as e:
            queue.clear()
            reply(type="error", error="sequence stopped: " + str(e))
    elif step[0] == "wait":
        wait_until = now + step[1]
    else:
        reply(type="info", text=step[1])
        reply(**status())


reply(type="hello", text="gimbal servo console ready", boot_off=boot_off())
reply(type="seqs", seqs=SEQS)
reply(**status())
buf = ""
last = time.monotonic()
while True:
    now = time.monotonic()
    try:
        tick(now, now - last)
    except Exception as e:
        ramps.clear(), sweeps.clear(), queue.clear()
        reply(type="error", error="motion stopped: %r" % e)
    last = now
    n = supervisor.runtime.serial_bytes_available
    if n:
        buf += sys.stdin.read(n)
        while "\n" in buf or "\r" in buf:
            i = min(x for x in (buf.find("\n"), buf.find("\r")) if x >= 0)
            line, buf = buf[:i].strip(), buf[i + 1:]
            if line:
                try:
                    handle(line)
                except Exception as e:  # never drop to the REPL: report and keep serving
                    reply(type="error", error=str(e) or "bad command", line=line)
    time.sleep(0.01)
