"""
    pid_lib.py -- motion control and odometry library.

    technical background:
    ---------------------
    this library handles the heavy lifting for robot movement. 
    it uses a combination of encoder feedback and gyro fusion 
    (via a simple kalman filter) to track position (x, y, h) 
    accurately in real-time.

    key features:
    -------------
    - s-curve & kinematic velocity profiles.
    - cross-track error (cte) steering for straight lines.
    - pure pursuit for smooth path following.
    - async task management for simultaneous movements.
"""

from hub import port
import hub as _hub
import motor, motor_pair
import runloop, time, micropython, gc
from math import cos, sin, atan2, pi, sqrt, degrees, radians, fabs, exp, asin
import config as C

#  ██████  ██████  ███    ██ ███████ ████████  █████  ███    ██ ████████ ███████ 
# ██      ██    ██ ████   ██ ██         ██    ██   ██ ████   ██    ██    ██      
# ██      ██    ██ ██ ██  ██ ███████    ██    ███████ ██ ██  ██    ██    ███████ 
# ██      ██    ██ ██  ██ ██      ██    ██    ██   ██ ██  ██ ██    ██         ██ 
#  ██████  ██████  ██   ████ ███████    ██    ██   ██ ██   ████    ██    ███████ 
#
# >>constants

_TWO_PI  = 2.0 * pi
_D2R     = pi / 180.0
_R2D     = 180.0 / pi
_HALF_WB = C.WHEEL_BASE_CM * 0.5

_GYRO_BIAS: float = 0.0
_TASKS = None # set in main

class EStopException(Exception): pass

# simple memory diagnostics
def mem_report():
    gc.collect()
    f = gc.mem_free()
    a = gc.mem_alloc()
    print(f"[mem] free: {f/1024:.1f}KB  used: {a/1024:.1f}KB  total: {(f+a)/1024:.1f}KB")

# ██    ██ ████████ ██ ██      ███████ 
# ██    ██    ██    ██ ██      ██      
# ██    ██    ██    ██ ██      ███████ 
# ██    ██    ██    ██ ██           ██ 
#  ██████     ██    ██ ███████ ███████ 
#
# >>utils

@micropython.native
def norm180(a: float) -> float:
    a %= 360.0
    return a - 360.0 if a > 180.0 else a

def _bat() -> float:
    v = _hub.battery.voltage()
    if v <= 6796: return 0.82
    if v >= C.BAT_FULL_MV: return 1.0
    return v / C.BAT_FULL_MV

def _clamp(v, lo=-1000, hi=1000):
    return lo if v < lo else (hi if v > hi else v)

@micropython.native
def _sig(x: float) -> float:
    if x >  20.0: return 1.0
    if x < -20.0: return 0.0
    return 1.0 / (1.0 + exp(-x))


#  ██████  ██    ██ ██████   ██████  
# ██       ██    ██ ██   ██ ██    ██ 
# ██   ███  ██  ██  ██████  ██    ██ 
# ██    ██    ██    ██   ██ ██    ██ 
#  ██████     ██    ██   ██  ██████  
#
# >>gyro calibration

async def calibrate_gyro(preheat_ms: int = 5000, samples: int = 300) -> float:
    global _GYRO_BIAS
    print(f"[gyro] preheat {preheat_ms}ms...")
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < preheat_ms:
        _hub.motion.angular_velocity(); await runloop.sleep_ms(10)
    buf = [0.0] * samples
    for i in range(samples):
        _, _, gz = _hub.motion.angular_velocity(); buf[i] = gz / 10.0; await runloop.sleep_ms(3)
    buf.sort()
    q1 = buf[samples >> 2]; q3 = buf[(samples * 3) >> 2]
    iqr = q3 - q1; lo = q1 - 1.5 * iqr; hi = q3 + 1.5 * iqr
    s = n = 0
    for v in buf:
        if lo <= v <= hi: s += v; n += 1
    _GYRO_BIAS = s / n if n else 0.0
    print(f"[gyro] bias={_GYRO_BIAS:.5f}  noise={buf[-1]-buf[0]:.3f}")
    return _GYRO_BIAS


# sensor snapshot

class _Snap:
    __slots__ = ('gz', 'eL', 'eR')
    def __init__(self): self.gz = 0.0; self.eL = self.eR = 0
    def read(self):
        # We use a simple derivative of yaw for gz if gyroscope() is unavailable, 
        # but try the direct motion API first, or fallback to 0.0 if not supported.
        try:
            from hub import motion
            self.gz = float(motion.angular_velocity()[2]) / 10.0
        except:
            self.gz = 0.0
        self.eL = motor.relative_position(C.PORT_L)
        self.eR = motor.relative_position(C.PORT_R)

_SNS = _Snap()


# fixed-rate loop

class _Loop:
    __slots__ = ('_p', '_n', 'dt')
    def __init__(self, hz=None):
        self._p = 1000 // (hz or C.HZ); self.dt = self._p * 0.001; self._n = 0
    def reset(self): self._n = time.ticks_ms()
    async def tick(self):
        self._n = time.ticks_add(self._n, self._p)
        r = time.ticks_diff(self._n, time.ticks_ms())
        if r > 0: await runloop.sleep_ms(r)


# backlash

class _BL:
    __slots__ = ('_bL', '_bR', '_dL', '_dR', '_cL', '_cR')
    def __init__(self):
        self._bL = C.BL_L; self._bR = C.BL_R
        self._dL = self._dR = self._cL = self._cR = 0
    def reset(self): self._dL = self._dR = self._cL = self._cR = 0
    def filt(self, dL, dR): return self._f(dL, 0), self._f(dR, 1)
    def _f(self, d, s):
        if d == 0: return 0
        bl = self._bL if s == 0 else self._bR
        nd = 1 if d > 0 else -1
        old = self._dL if s == 0 else self._dR
        cnt = self._cL if s == 0 else self._cR
        if nd != old and old != 0: cnt = bl
        if cnt > 0:
            ab = min(abs(d), cnt); cnt -= ab; out = (abs(d) - ab) * nd
        else: out = d
        if s == 0: self._dL = nd; self._cL = cnt
        else: self._dR = nd; self._cR = cnt
        return out


# tilt (removed for SPIKE 3 compatibility)

class _Tilt:
    __slots__ = ('_sc',)
    def __init__(self): self._sc = 1.0
    def cal(self): pass
    def fix(self, gz): return gz


#  ██████  ██████   ██████  
# ██    ██ ██   ██ ██    ██ 
# ██    ██ ██   ██ ██    ██ 
# ██    ██ ██   ██ ██    ██ 
#  ██████  ██████   ██████  
#
# >>odometry

class Odometry:
    __slots__ = ('x', 'y', 'h', '_eL', '_eR', '_tilt', '_bl',
                 '_kx', '_kP', '_vL', '_vR', '_spd')
    def __init__(self):
        self.x = self.y = self.h = 0.0
        self._eL = self._eR = 0
        self._tilt = _Tilt(); self._bl = _BL()
        self._kx = 0.0; self._kP = 1.0
        self._vL = self._vR = self._spd = 0.0
    def setup(self): self._tilt.cal(); self._bl.reset()
    def reset(self, x=0.0, y=0.0, h=0.0):
        self.x = x; self.y = y; self.h = h
        self._kx = h; self._kP = 1.0
        motor.reset_relative_position(C.PORT_L, 0)
        motor.reset_relative_position(C.PORT_R, 0)
        self._eL = self._eR = 0
        self._vL = self._vR = self._spd = 0.0
        self._bl.reset()
        print(f"[odo] reset x={x:.1f} y={y:.1f} h={h:.1f}")
    @micropython.native
    def update(self, moving=True):
        """
        math: sensor fusion odometry (simplified for SPIKE 3)
        ----------------------------
        1. backlash: filt(dL, dR) handles gear play compensation.
        2. velocity: low-pass filter (0.25 alpha) for speed stability.
        3. heading: direct from yaw_pitch_roll to avoid kalman complexity.
        4. position: standard dead reckoning (arc/chord approximation).
        """
        _SNS.read(); sns = _SNS; dt = 1.0 / C.HZ
        dLr = sns.eL - self._eL; dRr = sns.eR - self._eR
        self._eL = sns.eL; self._eR = sns.eR
        
        # apply backlash play compensation
        dLc, dRc = self._bl.filt(dLr, dRr)
        dL = dLc * C.CM_PER_COUNT; dR = dRc * C.CM_PER_COUNT
        
        # estimated velocity with noise filtering
        self._vL = 0.25 * (dLr * C.CM_PER_COUNT / dt) + 0.75 * self._vL
        self._vR = 0.25 * (dRr * C.CM_PER_COUNT / dt) + 0.75 * self._vR
        self._spd = (self._vL + self._vR) * 0.5
        
        # heading logic (simplified direct read)
        try:
            from hub import motion
            ypr = motion.yaw_pitch_roll()
            # Yaw is usually in decidegrees or degrees depending on firmware, assuming degrees here based on standard use
            self.h = float(ypr[0]) / 10.0
            gz = sns.gz
        except:
            self.h = self._kx
            gz = 0.0
            
        # update x,y coordinates
        ds = (dL + dR) * 0.5; hr = self.h * _D2R
        self.x += ds * cos(hr); self.y += ds * sin(hr)
        return self.x, self.y, self.h, gz
    def dist(self, tx, ty): return sqrt((tx-self.x)**2 + (ty-self.y)**2)
    def bear(self, tx, ty): return atan2(ty-self.y, tx-self.x) * _R2D
    def snap_x(self, v): self.x = v
    def snap_y(self, v): self.y = v
    def snap_h(self, v): self.h = v; self._kx = v
    @property
    def speed(self): return self._spd
    def __str__(self): return f"x={self.x:.2f} y={self.y:.2f} h={self.h:.1f}"


# ██████  ██████   ██████  ███████ ██ ██      ███████ ███████ 
# ██   ██ ██   ██ ██    ██ ██      ██ ██      ██      ██      
# ██████  ██████  ██    ██ █████   ██ ██      █████   ███████ 
# ██      ██   ██ ██    ██ ██      ██ ██      ██           ██ 
# ██      ██   ██  ██████  ██      ██ ███████ ███████ ███████ 
#
# >>speed profiles

def _sc(done, total, vmax, vmin=None):
    """
    math: s-curve (sigmoid) velocity profile
    ---------------------------------------
    formula: v = vmin + (vmax-vmin) / (1 + e^(-k * (progress - 0.5)))
    this creates a smooth acceleration and deceleration ramp.
    """
    vmin = vmin or C.VMIN
    if total <= 0: return vmin
    p = done / total
    t = min(_sig(C.SC_K_ACC*(p-0.15)), _sig(C.SC_K_DEC*(0.75-p)))
    return max(vmin, int(vmin + (vmax-vmin)*t))

def _kin(rem, vmax, vmin=None):
    vmin = vmin or C.VMIN
    vsq = 2.0 * C.DECEL_CPS2 * rem / C.DECEL_SAFETY
    return max(vmin, min(vmax, int(sqrt(max(0.0, vsq)))))


# ██   ██ ███████ ██      ██████  ███████ ██████  ███████ 
# ██   ██ ██      ██      ██   ██ ██      ██   ██ ██      
# ███████ █████   ██      ██████  █████   ██████  ███████ 
# ██   ██ ██      ██      ██      ██      ██   ██      ██ 
# ██   ██ ███████ ███████ ██      ███████ ██   ██ ███████ 
#
# >>hardware helpers

class DeadbandComp:
    __slots__ = ('dL', 'dR')
    def __init__(self): self.dL = C.DB_L; self.dR = C.DB_R
    def apply(self, L, R):
        if L > 0: L += self.dL
        elif L < 0: L -= self.dL
        if R > 0: R += self.dR
        elif R < 0: R -= self.dR
        return _clamp(L), _clamp(R)
    async def cal(self, p=None):
        p = p or C.PORT_L; motor.reset_relative_position(p, 0)
        for pw in range(20, 200, 2):
            motor.run(p, pw); await runloop.sleep_ms(60)
            if abs(motor.relative_position(p)) >= 3:
                motor.stop(p); print(f"[db] {p}={pw}"); return pw
            motor.reset_relative_position(p, 0)
        motor.stop(p); return 85

class _Stall:
    __slots__ = ('_cnt', '_lL', '_lR')
    def __init__(self): self._cnt = self._lL = self._lR = 0
    def reset(self):
        self._cnt = 0
        self._lL = motor.relative_position(C.PORT_L)
        self._lR = motor.relative_position(C.PORT_R)
    def check(self, pL, pR):
        cL = motor.relative_position(C.PORT_L); cR = motor.relative_position(C.PORT_R)
        p = abs(pL) > C.STALL_PWR or abs(pR) > C.STALL_PWR
        s = abs(cL-self._lL) < C.STALL_ENC and abs(cR-self._lR) < C.STALL_ENC
        self._lL = cL; self._lR = cR
        self._cnt = self._cnt + 1 if p and s else 0
        return self._cnt >= C.STALL_LOOPS

class _CTE:
    __slots__ = ('_I', '_ux', '_uy', '_sx', '_sy')
    def __init__(self): self._I = self._ux = self._uy = self._sx = self._sy = 0.0
    def seg(self, sx, sy, ex, ey):
        self._sx = sx; self._sy = sy; self._I = 0.0
        dx = ex-sx; dy = ey-sy; m = sqrt(dx*dx + dy*dy)
        if m > 1e-6: self._ux = dx/m; self._uy = dy/m
        else: self._ux = 1.0; self._uy = 0.0
    @micropython.native
    def calc(self, rx, ry, h, rate, dt):
        """
        math: cross-track error (cte) steering
        --------------------------------------
        1. vector: segment unit vector U = (ux, uy)
        2. normal: normal vector N = (-uy, ux)
        3. cte: dot product of (robot - start) and N
        4. steer: P(cte) + I(sum_cte) + D(gyro_rate) + P(heading_error)
        """
        cte = (rx-self._sx)*(-self._uy) + (ry-self._sy)*self._ux
        self._I = max(-60.0, min(60.0, self._I + cte*dt))
        he = norm180(atan2(self._uy, self._ux)*_R2D - h)
        # กลับเครื่องหมาย cte เนื่องจากถ้า cte > 0 (หุ่นอยู่ซ้าย) ต้องหักเลี้ยวขวา (corr ติดลบ)
        return -cte*C.CTE_KP - self._I*C.CTE_KI + he*C.CTE_KPH - rate*C.CTE_KDR

class _Micro:
    __slots__ = ()
    def creep(self, r):
        if r <= C.POS_TOL_CM: return 0
        ratio = max(0.0, min(1.0, (r-C.POS_TOL_CM)/(C.CREEP_CM-C.POS_TOL_CM)))
        return max(60, int(C.CREEP_PWR*ratio + 60))
    def done(self, r, v):
        return r <= C.POS_TOL_CM or (r <= C.CREEP_CM*0.3 and fabs(v) < C.CREEP_VEL_THR)

class _VelPID:
    __slots__ = ('_iL', '_iR', '_eL', '_eR', '_pL', '_pR')
    def __init__(self): self._iL = self._iR = self._eL = self._eR = self._pL = self._pR = 0.0
    def reset(self): self._iL = self._iR = self._eL = self._eR = 0.0; self._pL = self._pR = 0
    @micropython.native
    def step(self, tL, tR, aL, aR, dt):
        def _p(tgt, act, I, pe):
            e = tgt-act; I = max(-200, min(200, I+e*dt))
            return C.VEL_KP*e + C.VEL_KI*I + C.VEL_KD*(e-pe)/max(dt,1e-3), I, e
        oL,self._iL,self._eL = _p(tL,aL,self._iL,self._eL)
        oR,self._iR,self._eR = _p(tR,aR,self._iR,self._eR)
        self._pL = _clamp(int(self._pL+oL)); self._pR = _clamp(int(self._pR+oR))
        return int(self._pL), int(self._pR)

class _PP:
    __slots__ = ('_path', '_seg')
    def __init__(self): self._path = []; self._seg = 0
    def load(self, pts): self._path = [(float(p[0]),float(p[1])) for p in pts]; self._seg = 0
    def _ld(self, spd): return max(C.PP_LD_MIN_CM, min(C.PP_LD_MAX_CM, C.PP_LD_CM*spd/40.0))
    def step(self, rx, ry, hdeg, spd, vmx):
        """
        math: pure pursuit steering
        ---------------------------
        1. find target point on path at 'lookahead' distance.
        2. calculate curvature (kappa) = 2 * cross_track_error / lookahead²
        3. steering = curvature * wheelbase (simplified for differential drive)
        """
        if not self._path: return 0, 0
        Ld = self._ld(spd); best = None
        for i in range(self._seg, len(self._path)-1):
            x1,y1=self._path[i]; x2,y2=self._path[i+1]
            dx=x2-x1; dy=y2-y1; fx=x1-rx; fy=y1-ry
            a=dx*dx+dy*dy; b=2*(fx*dx+fy*dy); c=fx*fx+fy*fy-Ld*Ld
            disc=b*b-4*a*c
            if disc<0 or a<1e-9: continue
            sq=sqrt(disc)
            for t in [(-b+sq)/(2*a), (-b-sq)/(2*a)]:
                if 0.0<=t<=1.0: best=(x1+t*dx,y1+t*dy); self._seg=i; break
            if best: break
        if best is None: best = self._path[-1]
        hr = -hdeg*_D2R
        xl=(best[0]-rx)*cos(hr)-(best[1]-ry)*sin(hr)
        yl=(best[0]-rx)*sin(hr)+(best[1]-ry)*cos(hr)
        k = 2*yl/(Ld*Ld) if fabs(Ld)>1e-6 else 0.0
        vL=spd*(1-k*_HALF_WB); vR=spd*(1+k*_HALF_WB)
        sc=1.0
        if fabs(vL)>vmx: sc=min(sc,vmx/fabs(vL))
        if fabs(vR)>vmx: sc=min(sc,vmx/fabs(vR))
        return int(vL*sc), int(vR*sc)
    def done(self, rx, ry, tol=2.0):
        if not self._path: return True
        tx,ty=self._path[-1]; return sqrt((rx-tx)**2+(ry-ty)**2)<=tol


# ███████ ███████  ██████  ██████  ███████ 
# ██      ██      ██    ██ ██   ██ ██      
# ███████ █████   ██    ██ ██████  ███████ 
#      ██ ██      ██    ██ ██           ██ 
# ███████ ███████  ██████  ██      ███████ 
#
# >>system tools

class TaskManager:
    __slots__ = ('_jobs',)
    def __init__(self): self._jobs = []
    def start(self, coro):
        t = runloop.create_task(coro)
        self._jobs.append(t); return t
    def cancel_all(self):
        for t in self._jobs:
            try: t.cancel()
            except: pass
        self._jobs = []
        motor_pair.stop(motor_pair.PAIR_1)
        motor.stop(C.PORT_L); motor.stop(C.PORT_R)
        print("[task] all cancelled")


async def estop_loop(tm):
    from hub import button
    while True:
        if button.pressed(_hub.button.LEFT) or button.pressed(_hub.button.RIGHT):
            tm.cancel_all()
            print("\n!!! EMERGENCY STOP !!!\n")
            raise EStopException("E-Stop Pressed")
        await runloop.sleep_ms(100)

async def wall_align() -> bool:
    import config as C
    print(f"{C.CLR_GRY}[motion] wall aligning...{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
    dl=time.ticks_add(time.ticks_ms(),C.WA_MAX_MS)
    sL=sR=0; pL=pR=C.WA_SPEED
    lL=motor.relative_position(C.PORT_L); lR=motor.relative_position(C.PORT_R)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        await runloop.sleep_ms(8)
        cL=motor.relative_position(C.PORT_L); cR=motor.relative_position(C.PORT_R)
        dL=abs(cL-lL); dR=abs(cR-lR); lL=cL; lR=cR
        if dL<2 and dR>=2: sL+=1; pL=max(80,C.WA_SPEED-60); pR=min(1000,C.WA_SPEED+40)
        elif dR<2 and dL>=2: sR+=1; pR=max(80,C.WA_SPEED-60); pL=min(1000,C.WA_SPEED+40)
        else: sL=sL+1 if dL<2 else 0; sR=sR+1 if dR<2 else 0; pL=pR=C.WA_SPEED
        motor_pair.move_tank(motor_pair.PAIR_1,pL,pR)
        if sL>=C.WA_STALL and sR>=C.WA_STALL: break
    motor_pair.stop(motor_pair.PAIR_1); await runloop.sleep_ms(150)
    _hub.motion.reset_yaw(0); await runloop.sleep_ms(80)
    ok=sL>=C.WA_STALL and sR>=C.WA_STALL
    print(f"[wall_align] {'ok' if ok else 'timeout'}"); return ok


# ██████  ██████  ██ ██    ██ ███████ 
# ██   ██ ██   ██ ██ ██    ██ ██      
# ██   ██ ██████  ██ ██    ██ █████   
# ██   ██ ██   ██ ██  ██  ██  ██      
# ██████  ██   ██ ██   ████   ███████ 
#
# >>drive functions

async def straight(odo, dist_cm, vmax=None, heading=None,
                   db=None, max_ms=6000) -> bool:
    import config as C
    print(f"{C.CLR_GRY}[motion] straight {dist_cm:.1f}cm{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
    bf=_bat(); vmx=int((vmax or C.VMAX)*bf)
    hdg=heading if heading is not None else odo.h
    sx,sy=odo.x,odo.y; hr=hdg*_D2R
    ex,ey=sx+dist_cm*cos(hr),sy+dist_cm*sin(hr)
    m=max(sqrt((ex-sx)**2+(ey-sy)**2),1e-6); ux=(ex-sx)/m; uy=(ey-sy)/m
    lp=_Loop(); lp.reset(); dt=lp.dt
    cte=_CTE(); cte.seg(sx,sy,ex,ey)
    vpc=_VelPID(); vpc.reset()
    mic=_Micro(); stl=_Stall(); stl.reset()
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        x,y,h,rate=odo.update(moving=True)
        done=sqrt((x-sx)**2+(y-sy)**2); rem=max(0.0,dist_cm-done)
        if mic.done(rem,odo.speed): break
        spd=max(C.VMIN,min(_sc(done,dist_cm,vmx),_kin(rem,vmx)))
        cr=mic.creep(rem)
        if cr>0: spd=cr
        corr=cte.calc(x,y,h,rate,dt)
        tL=float(spd)-corr; tR=float(spd)+corr
        pL,pR=vpc.step(tL,tR,odo._vL,odo._vR,dt)
        pL=_clamp(pL); pR=_clamp(pR)
        if db: pL,pR=db.apply(pL,pR)
        if stl.check(pL,pR): motor_pair.stop(motor_pair.PAIR_1); print("[straight] stall"); return False
        motor_pair.move_tank(motor_pair.PAIR_1,pL,pR)
        await lp.tick()
    motor_pair.stop(motor_pair.PAIR_1)
    err=dist_cm-sqrt((odo.x-sx)**2+(odo.y-sy)**2)
    print(f"[straight] done err={err:+.2f}cm  {odo}"); return True


async def turn(odo, target_h, vmax=None, max_ms=3000) -> bool:
    import config as C
    print(f"{C.CLR_GRY}[motion] turn to {target_h:.1f}deg{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
    vmx=int((vmax or C.VTURN)*_bat())
    lp=_Loop(); lp.reset(); settle=0
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        _,_,h,rate=odo.update(moving=False)
        err=norm180(target_h-h)
        out=_clamp(int(C.TURN_KP*err-C.TURN_KD*rate),-vmx,vmx)
        # สลับเป็น -out, out เพื่อให้เมื่อ err > 0 (เป้าหมายอยู่ซ้าย) หุ่นเลี้ยวซ้าย
        motor_pair.move_tank(motor_pair.PAIR_1,-out,out)
        settle=settle+1 if abs(err)<C.TURN_ERR_TOL and abs(rate)<C.TURN_RATE_TOL else 0
        if settle>=C.TURN_SETTLE: break
        await lp.tick()
    motor_pair.stop(motor_pair.PAIR_1)
    _,_,h,_=odo.update(moving=False)
    print(f"[turn] done err={norm180(target_h-h):+.2f}deg  {odo}"); return True


async def pivot_turn(odo, target_h, side='auto', vmax=None, max_ms=3000) -> bool:
    """one wheel locked — tightest radius possible"""
    import config as C
    print(f"{C.CLR_GRY}[motion] pivot {side} to {target_h:.1f}deg{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
    vmx=int((vmax or C.VTURN)*_bat())
    err0=norm180(target_h-odo.h)
    if side=='auto': side='right' if err0>0 else 'left'
    lp=_Loop(); lp.reset(); settle=0
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        _,_,h,rate=odo.update(moving=False)
        err=norm180(target_h-h)
        pwr=_clamp(int(C.PIVOT_KP*abs(err)-C.PIVOT_KD*abs(rate)),0,vmx)
        # ใช้ motor_pair ขยับล้อเดียวเพื่อเลี้ยวให้ถูกทิศ
        if side=='right': motor_pair.move_tank(motor_pair.PAIR_1, 0, pwr) # ล้อขวาเดินหน้า = เลี้ยวซ้าย
        else: motor_pair.move_tank(motor_pair.PAIR_1, pwr, 0) # ล้อซ้ายเดินหน้า = เลี้ยวขวา
        settle=settle+1 if abs(err)<C.TURN_ERR_TOL and abs(rate)<C.TURN_RATE_TOL else 0
        if settle>=C.TURN_SETTLE: break
        await lp.tick()
    motor.stop(C.PORT_L); motor.stop(C.PORT_R)
    _,_,h,_=odo.update(moving=False)
    print(f"[pivot] done err={norm180(target_h-h):+.2f}deg  {odo}"); return True


async def swing_turn(odo, target_h, outer_speed=None,
                     inner_ratio=0.0, max_ms=3000) -> bool:
    """inner_ratio: 0=stop  -0.5=reverse  0.5=slow"""
    import config as C
    print(f"{C.CLR_GRY}[motion] swing to {target_h:.1f}deg{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
    ospd=int((outer_speed or C.VTURN)*_bat()); ispd=int(ospd*inner_ratio)
    lp=_Loop(); lp.reset(); settle=0
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        _,_,h,rate=odo.update(moving=False)
        err=norm180(target_h-h)
        f=_clamp(int(C.TURN_KP*abs(err)-C.TURN_KD*abs(rate)),0,ospd)/max(ospd,1)
        co=max(60,int(ospd*f)); ci=int(ispd*f)
        if err>0: pL,pR=ci,co
        else: pL,pR=co,ci
        motor_pair.move_tank(motor_pair.PAIR_1,_clamp(pL),_clamp(pR))
        settle=settle+1 if abs(err)<C.TURN_ERR_TOL and abs(rate)<C.TURN_RATE_TOL else 0
        if settle>=C.TURN_SETTLE: break
        await lp.tick()
    motor_pair.stop(motor_pair.PAIR_1)
    _,_,h,_=odo.update(moving=False)
    print(f"[swing] done err={norm180(target_h-h):+.2f}deg  {odo}"); return True


async def arc(odo, radius_cm, angle_deg, vmax=None, max_ms=5000) -> bool:
    """constant radius arc — smooth and geometric"""
    import config as C
    print(f"{C.CLR_GRY}[motion] arc r={radius_cm:.1f}cm{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
    bf=_bat(); vmx=int((vmax or C.VMAX)*bf)
    R=radius_cm if abs(radius_cm)>1e-3 else 1e-3
    W=C.WHEEL_BASE_CM
    rL=(R-W*0.5)/R; rR=(R+W*0.5)/R
    arc_cm=abs(radians(angle_deg)*R)
    target_h=odo.h+angle_deg
    lp=_Loop(); lp.reset(); sx,sy=odo.x,odo.y
    stl=_Stall(); stl.reset()
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        x,y,h,_=odo.update(moving=True)
        done=sqrt((x-sx)**2+(y-sy)**2)
        if abs(norm180(target_h-h))<0.5: break
        spd=max(C.VMIN,min(_sc(done,arc_cm,vmx),_kin(arc_cm-done,vmx)))
        pL=_clamp(int(spd*rL)); pR=_clamp(int(spd*rR))
        if stl.check(pL,pR): motor_pair.stop(motor_pair.PAIR_1); print("[arc] stall"); return False
        motor_pair.move_tank(motor_pair.PAIR_1,pL,pR)
        await lp.tick()
    motor_pair.stop(motor_pair.PAIR_1)
    _,_,h,_=odo.update(moving=False)
    print(f"[arc] done err={norm180(target_h-h):+.2f}deg  {odo}"); return True


async def goto_xy(odo, tx, ty, vmax=None, db=None, max_ms=6000) -> bool:
    import config as C
    print(f"{C.CLR_GRY}[motion] goto ({tx:.1f},{ty:.1f}){C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
    bf=_bat(); vmx=int((vmax or C.VMAX)*bf)
    lp=_Loop(); lp.reset(); dt=lp.dt
    brg=odo.bear(tx,ty)
    if abs(norm180(brg-odo.h))>2.0: await turn(odo,brg)
    cte=_CTE(); cte.seg(odo.x,odo.y,tx,ty)
    vpc=_VelPID(); vpc.reset()
    stl=_Stall(); stl.reset()
    mic=_Micro(); total=odo.dist(tx,ty)
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        x,y,h,rate=odo.update(moving=True)
        rem=odo.dist(tx,ty); done=total-rem
        if mic.done(rem,odo.speed): break
        spd=max(C.VMIN,min(_sc(done,total,vmx),_kin(rem,vmx)))
        cr=mic.creep(rem)
        if cr>0: spd=cr
        corr=cte.calc(x,y,h,rate,dt)
        tL=float(spd)-corr; tR=float(spd)+corr
        pL,pR=vpc.step(tL,tR,odo._vL,odo._vR,dt)
        pL=_clamp(pL); pR=_clamp(pR)
        if db: pL,pR=db.apply(pL,pR)
        if stl.check(pL,pR): motor_pair.stop(motor_pair.PAIR_1); print("[goto_xy] stall"); return False
        motor_pair.move_tank(motor_pair.PAIR_1,pL,pR)
        await lp.tick()
    motor_pair.stop(motor_pair.PAIR_1)
    print(f"[goto_xy] done err={odo.dist(tx,ty):.2f}cm  {odo}"); return True


async def track_line(odo, dist_cm, vmax=None, sensor_port=None,
                     edge='left', Kp=None, Kd=None, db=None, max_ms=8000) -> bool:
    """
    แทร็กเส้น 1 เซ็นเซอร์ (เกาะขอบ) พร้อมอัปเดตพิกัด (Odometry) และ S-Curve
    edge: 'left' (ขาวอยู่ซ้าย-ดำอยู่ขวา) หรือ 'right' (ดำอยู่ซ้าย-ขาวอยู่ขวา)
    """
    import config as C
    import sensor_lib as S
    print(f"{C.CLR_GRY}[motion] track_line {dist_cm:.1f}cm edge={edge}{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
    bf = _bat(); vmx = int((vmax or C.VMAX) * bf)
    sp = sensor_port or C.PORT_C1
    Kp = Kp or C.LF_KP_MED; Kd = Kd or C.LF_KD_MED
    
    lp = _Loop(); lp.reset(); dt = lp.dt
    vpc = _VelPID(); vpc.reset()
    mic = _Micro(); stl = _Stall(); stl.reset()
    
    sx, sy = odo.x, odo.y
    pe = 0.0
    dl = time.ticks_add(time.ticks_ms(), max_ms)
    edge_mult = 1.0 if edge == 'left' else -1.0
    le = S.LineEst(0.3)
    
    while time.ticks_diff(dl, time.ticks_ms()) > 0:
        x, y, h, rate = odo.update(moving=True)
        done = sqrt((x-sx)**2 + (y-sy)**2)
        rem = max(0.0, dist_cm - done)
        
        if mic.done(rem, odo.speed): break
        
        spd = max(C.VMIN, min(_sc(done, dist_cm, vmx), _kin(rem, vmx)))
        cr = mic.creep(rem)
        if cr > 0: spd = cr
            
        r = S.reflect(sp)
        if r < 0: r = S.CAL.mid(sp)
        
        err = le.single(r, sp) * edge_mult
        de = (err - pe) / dt; pe = err
        
        c = Kp * err + Kd * de
        tL = float(spd) - c
        tR = float(spd) + c
        
        pL, pR = vpc.step(tL, tR, odo._vL, odo._vR, dt)
        pL = _clamp(pL); pR = _clamp(pR)
        if db: pL, pR = db.apply(pL, pR)
        
        if stl.check(pL, pR): 
            motor_pair.stop(motor_pair.PAIR_1)
            print("[track_line] stall")
            return False
            
        motor_pair.move_tank(motor_pair.PAIR_1, pL, pR)
        await lp.tick()
        
    motor_pair.stop(motor_pair.PAIR_1)
    err_dist = dist_cm - sqrt((odo.x-sx)**2 + (odo.y-sy)**2)
    print(f"[track_line] done err={err_dist:+.2f}cm  {odo}"); return True


async def follow_path(odo, waypoints, default_vmax=None,
                      smooth=True, db=None) -> bool:
    print(f"[path] {len(waypoints)} pts  smooth={smooth}")
    dvmx=default_vmax or C.VMAX
    if smooth:
        motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
        bf=_bat(); pp=_PP()
        pts=[(float(w[0]),float(w[1])) for w in waypoints]; pp.load(pts)
        total=sum(sqrt((pts[i][0]-pts[i-1][0])**2+(pts[i][1]-pts[i-1][1])**2)
                  for i in range(1,len(pts)))
        lp=_Loop(); lp.reset(); dt=lp.dt; sx,sy=odo.x,odo.y
        dl=time.ticks_add(time.ticks_ms(),15000)
        while time.ticks_diff(dl,time.ticks_ms())>0:
            x,y,h,_=odo.update(moving=True)
            if pp.done(x,y): break
            done=sqrt((x-sx)**2+(y-sy)**2)
            spd=max(C.VMIN,_sc(done,total,int(dvmx*bf)))
            pL,pR=pp.step(x,y,h,float(spd),int(dvmx*bf))
            if db: pL,pR=db.apply(pL,pR)
            motor_pair.move_tank(motor_pair.PAIR_1,_clamp(pL),_clamp(pR))
            await lp.tick()
        motor_pair.stop(motor_pair.PAIR_1)
    else:
        for wp in waypoints:
            vmx=int(wp[2]) if len(wp)>2 else dvmx
            if not await goto_xy(odo,float(wp[0]),float(wp[1]),vmax=vmx,db=db): return False
            await runloop.sleep_ms(80)
    print(f"[path] done  {odo}"); return True


#  █████  ████████ ████████  █████   ██████ ██   ██ ███    ███ ███████ ███    ██ ████████ 
# ██   ██    ██       ██    ██   ██ ██      ██   ██ ████  ████ ██      ████   ██    ██    
# ███████    ██       ██    ███████ ██      ███████ ██ ████ ██ █████   ██ ██  ██    ██    
# ██   ██    ██       ██    ██   ██ ██      ██   ██ ██  ██  ██ ██      ██  ██ ██    ██    
# ██   ██    ██       ██    ██   ██  ██████ ██   ██ ██      ██ ███████ ██   ████    ██    
#
# >>attachments

async def motor_to_angle(p, target_deg: float, speed: int = 300,
                          tol: float = 2.0, max_ms: int = 3000) -> bool:
    """
    move attachment motor to absolute angle (degrees) from home
    uses position pid — accurate to ±1-2deg
    """
    print(f"[motor] port={p} -> {target_deg:.1f}deg  spd={speed}")
    KP=4.0; KD=0.8; prev_err=0.0; prev_t=time.ticks_ms()
    dl=time.ticks_add(time.ticks_ms(),max_ms); settle=0
    while time.ticks_diff(dl,time.ticks_ms())>0:
        pos=motor.relative_position(p)
        err=target_deg-pos
        now=time.ticks_ms(); dt=max(1,time.ticks_diff(now,prev_t))/1000.0; prev_t=now
        d=(err-prev_err)/dt; prev_err=err
        pwr=_clamp(int(KP*err-KD*d),-speed,speed)
        motor.run(p,pwr)
        settle=settle+1 if abs(err)<tol else 0
        if settle>=8: break
        await runloop.sleep_ms(8)
    motor.stop(p)
    pos=motor.relative_position(p)
    print(f"[motor] done pos={pos:.1f}deg  err={target_deg-pos:+.1f}")
    return True


async def motor_run_time(p, speed: int, duration_ms: int) -> None:
    """run attachment motor at speed for duration_ms then hold"""
    print(f"[motor] port={p} spd={speed} for {duration_ms}ms")
    motor.run(p, speed)
    await runloop.sleep_ms(duration_ms)
    motor.stop(p)
    print(f"[motor] done")


async def motor_run_until_stall(p, speed: int, max_ms: int = 3000,
                                 stall_ms: int = 200) -> bool:
    """
    run motor until resistance (claw close / arm hits limit)
    useful for: gripping objects, lowering arm to floor, homing
    """
    print(f"[motor] port={p} run until stall  spd={speed}")
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    prev=motor.relative_position(p); still_ms=0; still_t=time.ticks_ms()
    motor.run(p,speed)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        await runloop.sleep_ms(20)
        cur=motor.relative_position(p)
        if abs(cur-prev)<2:
            still_ms=time.ticks_diff(time.ticks_ms(),still_t)
            if still_ms>=stall_ms: break
        else: still_t=time.ticks_ms()
        prev=cur
    motor.stop(p)
    stalled=still_ms>=stall_ms
    print(f"[motor] {'stall detected' if stalled else 'timeout'}")
    return stalled


async def motor_home(p, speed: int = 150, max_ms: int = 3000) -> None:
    """run motor backward until stall then reset position to 0"""
    print(f"[motor] homing port={p}")
    await motor_run_until_stall(p,-abs(speed),max_ms)
    motor.reset_relative_position(p,0)
    print(f"[motor] home set to 0")


#  COMBINED MOTION  — drive + attachment at same time

async def straight_with_motor(odo, dist_cm: float, vmax: int,
                               arm_port, arm_target_deg: float,
                               arm_speed: int = 300,
                               heading: float = None,
                               db=None, max_ms: int = 6000) -> bool:
    """
    drive straight AND move attachment motor simultaneously
    arm starts moving immediately, drive finishes normally
    """
    print(f"[combo] straight {dist_cm:.1f}cm + arm -> {arm_target_deg:.0f}deg")
    motor_pair.pair(motor_pair.PAIR_1, C.PORT_L, C.PORT_R)
    bf=_bat(); vmx=int(vmax*bf)
    hdg=heading if heading is not None else odo.h
    sx,sy=odo.x,odo.y; hr=hdg*_D2R
    ex,ey=sx+dist_cm*cos(hr),sy+dist_cm*sin(hr)
    m=max(sqrt((ex-sx)**2+(ey-sy)**2),1e-6)
    lp=_Loop(); lp.reset(); dt=lp.dt
    cte=_CTE(); cte.seg(sx,sy,ex,ey)
    vpc=_VelPID(); vpc.reset()
    mic=_Micro(); stl=_Stall(); stl.reset()
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    # arm pid state
    arm_KP=4.0; arm_KD=0.8; arm_pe=0.0; arm_pt=time.ticks_ms()
    while time.ticks_diff(dl,time.ticks_ms())>0:
        x,y,h,rate=odo.update(moving=True)
        done=sqrt((x-sx)**2+(y-sy)**2); rem=max(0.0,dist_cm-done)
        if mic.done(rem,odo.speed): break
        spd=max(C.VMIN,min(_sc(done,dist_cm,vmx),_kin(rem,vmx)))
        cr=mic.creep(rem)
        if cr>0: spd=cr
        corr=cte.calc(x,y,h,rate,dt)
        tL=float(spd)-corr; tR=float(spd)+corr
        pL,pR=vpc.step(tL,tR,odo._vL,odo._vR,dt)
        pL=_clamp(pL); pR=_clamp(pR)
        if db: pL,pR=db.apply(pL,pR)
        motor_pair.move_tank(motor_pair.PAIR_1,pL,pR)
        # arm control
        arm_pos=motor.relative_position(arm_port)
        arm_err=arm_target_deg-arm_pos
        now=time.ticks_ms(); adt=max(1,time.ticks_diff(now,arm_pt))/1000.0; arm_pt=now
        arm_pwr=_clamp(int(arm_KP*arm_err-arm_KD*(arm_err-arm_pe)/adt),-arm_speed,arm_speed)
        arm_pe=arm_err; motor.run(arm_port,arm_pwr)
        await lp.tick()
    motor_pair.stop(motor_pair.PAIR_1); motor.stop(arm_port)
    print(f"[combo] done  {odo}"); return True


# competition check

async def comp_check() -> bool:
    print("\n=== comp check ===")
    warns=[]
    v=_hub.battery.voltage(); pct=(v-C.BAT_MIN_MV)/(C.BAT_FULL_MV-C.BAT_MIN_MV)*100
    print(f"[bat] {v}mv  {pct:.0f}%")
    if v<C.BAT_WARN_MV: warns.append(f"low bat {v}mv")
    print("[gyro] cal..."); await calibrate_gyro(5000,300)
    s=[]; [(s.append(_hub.motion.gyroscope()[0]),time.sleep_ms(5)) for _ in range(30)]
    n=max(s)-min(s); print(f"[gyro] noise={n:.2f}")
    if n>C.CHK_NOISE_MAX: warns.append(f"noise {n:.1f}")
    # straight
    print("[str] 50cm...")
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    motor.reset_relative_position(C.PORT_L,0); motor.reset_relative_position(C.PORT_R,0)
    I=h=0.0
    for _ in range(625):
        avg=(motor.relative_position(C.PORT_L)+motor.relative_position(C.PORT_R))/2
        if avg>=50/C.CM_PER_COUNT*0.95: break
        gz,_,_=_hub.motion.gyroscope(); h+=(gz-_GYRO_BIAS)*0.008; I+=h*0.008
        c=3.5*h+0.01*I
        motor_pair.move_tank(motor_pair.PAIR_1,int(400-c),int(400+c)); time.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1); await runloop.sleep_ms(400)
    actual=(motor.relative_position(C.PORT_L)+motor.relative_position(C.PORT_R))/2*C.CM_PER_COUNT
    e=50-actual; print(f"[str] act={actual:.2f}cm err={e:+.2f}cm")
    if abs(e)>C.CHK_STR_ERR_CM: warns.append(f"str err {e:+.2f}cm")
    # turn
    print("[turn] 360...")
    _hub.motion.yaw_pitch_roll(0); await runloop.sleep_ms(100); settle=0
    for _ in range(500):
        yaw=_hub.motion.yaw_pitch_roll()[0]/10.0; e2=norm180(360-yaw)
        motor_pair.move_tank(motor_pair.PAIR_1,max(-400,min(400,int(10*e2))),max(-400,min(400,-int(10*e2))))
        settle=settle+1 if abs(e2)<1.0 else 0
        if settle>12: break
        time.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1); await runloop.sleep_ms(300)
    te=norm180(_hub.motion.yaw_pitch_roll()[0]/10.0-360)
    print(f"[turn] err={te:+.1f}deg")
    if abs(te)>C.CHK_TURN_ERR: warns.append(f"turn {te:+.1f}deg")
    if not warns: print("=== all clear ===\n"); return True
    for w in warns: print(f"  warn: {w}")
    go=len(warns)<=1; print(f"=== {'caution' if go else 'abort'} ===\n"); return go
