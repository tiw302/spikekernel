"""
    sensor_lib.py -- sensor fusion and line following.

    technical background:
    ---------------------
    this module translates raw light reflection values into 
    mathematical positions. it supports single and dual sensor 
    centroid estimation, and handles "junction" detection 
    for complex wro mission paths.

    fusion:
    -------
    we combine color sensor readings with gyro heading to 
    maintain stability during high-speed line following, 
    reducing the "oscillations" common with simple pid.
"""

from hub import port
import hub as _hub
import motor, motor_pair
import runloop, time, micropython
from math import sqrt, fabs, pi
import config as C

#  ██████  ██████  ███    ██ ███████ ████████  █████  ███    ██ ████████ ███████ 
# ██      ██    ██ ████   ██ ██         ██    ██   ██ ████   ██    ██    ██      
# ██      ██    ██ ██ ██  ██ ███████    ██    ███████ ██ ██  ██    ██    ███████ 
# ██      ██    ██ ██  ██ ██      ██    ██    ██   ██ ██  ██ ██    ██         ██ 
#  ██████  ██████  ██   ████ ███████    ██    ██   ██ ██   ████    ██    ███████ 
#
# >>constants

NONE=0; BLACK=1; VIOLET=2; BLUE=3; CYAN=4
GREEN=5; YELLOW=6; RED=7; WHITE=8
_CN={0:"none",1:"black",2:"violet",3:"blue",4:"cyan",
     5:"green",6:"yellow",7:"red",8:"white"}
def cname(c): return _CN.get(c,"?")


# ██████   █████  ██     ██ 
# ██   ██ ██   ██ ██     ██ 
# ██████  ███████ ██  █  ██ 
# ██   ██ ██   ██ ██ ███ ██ 
# ██   ██ ██   ██  ███   ██ 
#
# >>raw reads

def reflect(p=None):
    try: d=_hub.port[p or C.PORT_C1].device.get(); return int(d[0]) if d else -1
    except: return -1

def color(p=None):
    try: d=_hub.port[p or C.PORT_C1].device.get(); return int(d[1]) if d and len(d)>1 else -1
    except: return -1

def rgb(p=None):
    try:
        d=_hub.port[p or C.PORT_C1].device.get()
        return (int(d[2]),int(d[3]),int(d[4]),int(d[5])) if d and len(d)>=6 else (0,0,0,0)
    except: return 0,0,0,0


#  ██████  █████  ██      
# ██      ██   ██ ██      
# ██      ███████ ██      
# ██      ██   ██ ██      
#  ██████ ██   ██ ███████ 
#
# >>calibration

class SensorCal:
    __slots__=('_w','_b','_m')
    def __init__(self):
        self._w={C.PORT_C1:C.REFLECT_WHITE,C.PORT_C2:C.REFLECT_WHITE}
        self._b={C.PORT_C1:C.REFLECT_BLACK,C.PORT_C2:C.REFLECT_BLACK}
        self._m={C.PORT_C1:(C.REFLECT_WHITE+C.REFLECT_BLACK)//2,
                 C.PORT_C2:(C.REFLECT_WHITE+C.REFLECT_BLACK)//2}
    def white(self,p=None,n=20):
        p=p or C.PORT_C1; s=k=0
        for _ in range(n): r=reflect(p); s+=r if r>=0 else 0; k+=1 if r>=0 else 0; time.sleep_ms(5)
        if k: self._w[p]=s//k; self._upd(p); print(f"[cal] white[{p}]={self._w[p]}")
    def black(self,p=None,n=20):
        p=p or C.PORT_C1; s=k=0
        for _ in range(n): r=reflect(p); s+=r if r>=0 else 0; k+=1 if r>=0 else 0; time.sleep_ms(5)
        if k: self._b[p]=s//k; self._upd(p); print(f"[cal] black[{p}]={self._b[p]}")
    def _upd(self,p): self._m[p]=(self._w[p]+self._b[p])//2
    def norm(self,raw,p=None):
        p=p or C.PORT_C1; span=max(1,self._w[p]-self._b[p])
        return (raw-self._m[p])/(span*0.5)
    def mid(self,p=None): return self._m[p or C.PORT_C1]
    async def sweep(self,p=None,spd=200,ms=600):
        p=p or C.PORT_C1; print(f"[cal] sweep p={p}")
        motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
        buf=[]; motor_pair.move_tank(motor_pair.PAIR_1,spd,spd)
        t0=time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(),t0)<ms:
            r=reflect(p)
            if r>=0: buf.append(r)
            time.sleep_ms(10)
        motor_pair.stop(motor_pair.PAIR_1)
        if len(buf)<5: return
        self._w[p]=max(buf); self._b[p]=min(buf); self._upd(p)
        print(f"[cal] w={self._w[p]} b={self._b[p]} mid={self._m[p]}")

CAL=SensorCal()


# ███████ ███████ ████████ ██ ███    ███  █████  ████████ ███████ 
# ██      ██         ██    ██ ████  ████ ██   ██    ██    ██      
# █████   ███████    ██    ██ ██ ████ ██ ███████    ██    █████   
# ██           ██    ██    ██ ██  ██  ██ ██   ██    ██    ██      
# ███████ ███████    ██    ██ ██      ██ ██   ██    ██    ███████ 
#
# >>line estimator

class LineEst:
    __slots__=('_a','_pos','_on')
    def __init__(self,a=0.3): self._a=a; self._pos=0.0; self._on=False
    @micropython.native
    def single(self,raw,p=None):
        p=p or C.PORT_C1; n=CAL.norm(raw,p)
        rp=-n; self._pos=self._a*rp+(1-self._a)*self._pos
        self._on=raw<=CAL.mid(p)+15; return self._pos
    @micropython.native
    def dual(self,rL,rR):
        """
        math: weighted dual-sensor centroid
        -----------------------------------
        1. intensity (n) = 1.0 - normalized_reflect (higher = darker)
        2. raw_pos = (right_intensity - left_intensity) / total_intensity
        3. filter: exponential moving average to reduce jitter
        range: [-1.0 (left), 1.0 (right)]
        """
        nL=max(0.0,1.0-CAL.norm(rL,C.PORT_C1))
        nR=max(0.0,1.0-CAL.norm(rR,C.PORT_C2))
        tot=nL+nR
        if tot>0.1: 
            rp=(nR-nL)/tot
            self._pos=self._a*rp+(1-self._a)*self._pos
        self._on=tot>0.3; return self._pos
    @property
    def pos(self): return self._pos
    @property
    def on_line(self): return self._on


def _gbias():
    try: import pid_lib; return pid_lib._GYRO_BIAS
    except: return 0.0

def _cnt(cm): return int(cm*C.COUNT_PER_CM)


# ██      ███████ 
# ██      ██      
# ██      █████   
# ██      ██      
# ███████ ██      
#
# >>line follow

async def lf_pd(dist_cm=50.0, vmax=500, Kp=None, Kd=None,
                p=None, max_ms=8000) -> bool:
    """line follow — single sensor pd"""
    import config as C
    print(f"{C.CLR_GRY}[sensor] lf_pd {dist_cm:.1f}cm{C.CLR_RST}")
    Kp=Kp or C.LF_KP_MED; Kd=Kd or C.LF_KD_MED; p=p or C.PORT_C1
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    le=LineEst(0.3); dl=time.ticks_add(time.ticks_ms(),max_ms)
    pe=0.0; pt=time.ticks_ms()
    motor.reset_relative_position(C.PORT_L,0); motor.reset_relative_position(C.PORT_R,0)
    tc=_cnt(dist_cm)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        if (motor.relative_position(C.PORT_L)+motor.relative_position(C.PORT_R))>>1>=tc: break
        r=reflect(p); r=CAL.mid(p) if r<0 else r
        err=le.single(r,p); now=time.ticks_ms()
        dt=max(1,time.ticks_diff(now,pt))/1000.0; pt=now
        de=(err-pe)/dt; pe=err; c=Kp*err+Kd*de
        motor_pair.move_tank(motor_pair.PAIR_1,max(-1000,min(1000,int(vmax-c))),max(-1000,min(1000,int(vmax+c))))
        await runloop.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1); print(f"[lf_pd] done"); return True


async def lf_gyro(dist_cm=50.0, vmax=500, Kp=None, Kd=None, Kg=None,
                  p=None, max_ms=8000) -> bool:
    """line follow — pd + gyro fusion, less zig-zag at high speed"""
    import config as C
    print(f"{C.CLR_GRY}[sensor] lf_gyro {dist_cm:.1f}cm{C.CLR_RST}")
    Kp=Kp or C.LF_KP_MED; Kd=Kd or C.LF_KD_MED; Kg=Kg or C.LF_KGYRO; p=p or C.PORT_C1
    bias=_gbias()
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    le=LineEst(0.25); dl=time.ticks_add(time.ticks_ms(),max_ms)
    pe=0.0; pt=time.ticks_ms(); hc=0.0; ht=time.ticks_ms()
    motor.reset_relative_position(C.PORT_L,0); motor.reset_relative_position(C.PORT_R,0)
    tc=_cnt(dist_cm)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        if (motor.relative_position(C.PORT_L)+motor.relative_position(C.PORT_R))>>1>=tc: break
        now=time.ticks_ms(); dth=max(1,time.ticks_diff(now,ht))/1000.0; ht=now
        gz,_,_=_hub.motion.gyroscope(); hc+=(gz-bias)*dth
        r=reflect(p); r=CAL.mid(p) if r<0 else r; lv=le.single(r,p)
        he=-hc
        while he>180: he-=360
        while he<-180: he+=360
        a=0.7 if le.on_line else 0.1
        err=a*lv+(1-a)*(he/45.0)
        dt2=max(1,time.ticks_diff(now,pt))/1000.0; pt=now
        de=(err-pe)/dt2; pe=err; c=Kp*err+Kd*de+Kg*he
        motor_pair.move_tank(motor_pair.PAIR_1,max(-1000,min(1000,int(vmax-c))),max(-1000,min(1000,int(vmax+c))))
        await runloop.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1); print(f"[lf_gyro] done"); return True


async def lf_dual(dist_cm=50.0, vmax=550, Kp=None, Kd=None,
                  max_ms=8000) -> bool:
    """line follow — dual sensor centroid, accurate on curves"""
    import config as C
    print(f"{C.CLR_GRY}[sensor] lf_dual {dist_cm:.1f}cm{C.CLR_RST}")
    Kp=Kp or C.LF_KP_HIGH; Kd=Kd or C.LF_KD_HIGH
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    le=LineEst(0.2); dl=time.ticks_add(time.ticks_ms(),max_ms)
    pe=0.0; pt=time.ticks_ms()
    motor.reset_relative_position(C.PORT_L,0); motor.reset_relative_position(C.PORT_R,0)
    tc=_cnt(dist_cm)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        if (motor.relative_position(C.PORT_L)+motor.relative_position(C.PORT_R))>>1>=tc: break
        rL=reflect(C.PORT_C1); rR=reflect(C.PORT_C2)
        rL=CAL.mid(C.PORT_C1) if rL<0 else rL; rR=CAL.mid(C.PORT_C2) if rR<0 else rR
        err=le.dual(rL,rR); now=time.ticks_ms()
        dt=max(1,time.ticks_diff(now,pt))/1000.0; pt=now
        de=(err-pe)/dt; pe=err; c=Kp*err+Kd*de
        motor_pair.move_tank(motor_pair.PAIR_1,max(-1000,min(1000,int(vmax-c))),max(-1000,min(1000,int(vmax+c))))
        await runloop.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1); print(f"[lf_dual] done"); return True


async def lf_dual_gyro(dist_cm=50.0, vmax=600, Kp=None, Kd=None,
                        Kg=None, max_ms=8000) -> bool:
    """line follow — dual sensor + gyro fusion, highest accuracy"""
    import config as C
    print(f"{C.CLR_GRY}[sensor] lf_dual_gyro {dist_cm:.1f}cm{C.CLR_RST}")
    Kp=Kp or C.LF_KP_HIGH; Kd=Kd or C.LF_KD_HIGH; Kg=Kg or C.LF_KGYRO
    bias=_gbias()
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    le=LineEst(0.2); dl=time.ticks_add(time.ticks_ms(),max_ms)
    pe=0.0; pt=time.ticks_ms(); hc=0.0; ht=time.ticks_ms()
    motor.reset_relative_position(C.PORT_L,0); motor.reset_relative_position(C.PORT_R,0)
    tc=_cnt(dist_cm)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        if (motor.relative_position(C.PORT_L)+motor.relative_position(C.PORT_R))>>1>=tc: break
        now=time.ticks_ms(); dth=max(1,time.ticks_diff(now,ht))/1000.0; ht=now
        gz,_,_=_hub.motion.gyroscope(); hc+=(gz-bias)*dth
        rL=reflect(C.PORT_C1); rR=reflect(C.PORT_C2)
        rL=CAL.mid(C.PORT_C1) if rL<0 else rL; rR=CAL.mid(C.PORT_C2) if rR<0 else rR
        lv=le.dual(rL,rR)
        he=-hc
        while he>180: he-=360
        while he<-180: he+=360
        a=0.75 if le.on_line else 0.05
        err=a*lv+(1-a)*(he/45.0)
        dt2=max(1,time.ticks_diff(now,pt))/1000.0; pt=now
        de=(err-pe)/dt2; pe=err; c=Kp*err+Kd*de+Kg*he
        motor_pair.move_tank(motor_pair.PAIR_1,max(-1000,min(1000,int(vmax-c))),max(-1000,min(1000,int(vmax+c))))
        await runloop.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1); print(f"[lf_dual_gyro] done"); return True


# ███    ███ ██ ███████ ███████ ██  ██████  ███    ██ 
# ████  ████ ██ ██      ██      ██ ██    ██ ████   ██ 
# ██ ████ ██ ██ ███████ ███████ ██ ██    ██ ██ ██  ██ 
# ██  ██  ██ ██      ██      ██ ██ ██    ██ ██  ██ ██ 
# ██      ██ ██ ███████ ███████ ██  ██████  ██   ████ 
#
# >>wro missions

async def lf_n_junctions(n: int, vmax: int = 400,
                          mode: str = "dual",
                          slow_at_junction: bool = True,
                          slow_vmax: int = 200,
                          slow_dist_cnt: int = 30,
                          max_ms: int = 20000) -> bool:
    """
    follow line and count junctions — stop after nth junction
    most common WRO mission pattern

    n: how many junctions to pass (stop AT nth)
    mode: "single" / "gyro" / "dual" / "dual_gyro"
    slow_at_junction: reduce speed near junction for accurate detection
    """
    import config as C
    print(f"{C.CLR_GRY}[sensor] follow to junction #{n}  mode={mode}{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    bias=_gbias()
    le=LineEst(0.2 if 'dual' in mode else 0.3)
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    pe=0.0; pt=time.ticks_ms(); hc=0.0; ht=time.ticks_ms()
    # junction state
    jcount=0; in_junc=False; junc_cnt=0
    JUNC_CNT_THR=C.JUNC_CONFIRM
    found=False

    while time.ticks_diff(dl,time.ticks_ms())>0:
        # read sensors
        rL=reflect(C.PORT_C1); rR=reflect(C.PORT_C2)
        rL=CAL.mid(C.PORT_C1) if rL<0 else rL
        rR=CAL.mid(C.PORT_C2) if rR<0 else rR

        # junction: both sensors dark
        tL=CAL.mid(C.PORT_C1)-C.JUNC_OFFSET
        tR=CAL.mid(C.PORT_C2)-C.JUNC_OFFSET
        both_dark=(rL<=tL and rR<=tR)

        if both_dark:
            junc_cnt+=1
            if junc_cnt==JUNC_CNT_THR and not in_junc:
                in_junc=True; jcount+=1
                print(f"[lf_n] junction {jcount}/{n}")
                if jcount>=n: found=True; break
        else:
            if in_junc and junc_cnt<JUNC_CNT_THR*3: pass  # still in junction
            else: in_junc=False; junc_cnt=0

        # speed: slow down when approaching junction threshold
        cur_v=slow_vmax if (junc_cnt>slow_dist_cnt//2 and slow_at_junction) else vmax

        # error calculation
        now=time.ticks_ms()
        if 'dual' in mode:
            lv=le.dual(rL,rR); err=lv
        else:
            lv=le.single(rL,C.PORT_C1); err=lv

        if 'gyro' in mode:
            dth=max(1,time.ticks_diff(now,ht))/1000.0; ht=now
            gz,_,_=_hub.motion.gyroscope(); hc+=(gz-bias)*dth
            he=-hc
            while he>180: he-=360
            while he<-180: he+=360
            a=0.7 if le.on_line else 0.1
            err=a*lv+(1-a)*(he/45.0)

        dt2=max(1,time.ticks_diff(now,pt))/1000.0; pt=now
        Kp=C.LF_KP_MED; Kd=C.LF_KD_MED
        de=(err-pe)/dt2; pe=err; c=Kp*err+Kd*de
        if 'gyro' in mode: c+=C.LF_KGYRO*(he if 'gyro' in mode else 0)
        motor_pair.move_tank(motor_pair.PAIR_1,
                             max(-1000,min(1000,int(cur_v-c))),
                             max(-1000,min(1000,int(cur_v+c))))
        await runloop.sleep_ms(8)

    motor_pair.stop(motor_pair.PAIR_1)
    print(f"[lf_n] {'reached junc '+str(n) if found else 'timeout'}  counted={jcount}")
    return found


async def lf_until_color(target_color: int, vmax: int = 400,
                          mode: str = "dual_gyro",
                          max_ms: int = 15000) -> bool:
    """follow line until specific color detected under sensor"""
    import config as C
    print(f"{C.CLR_GRY}[sensor] follow until color {cname(target_color)}{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    bias=_gbias()
    le=LineEst(0.2); dl=time.ticks_add(time.ticks_ms(),max_ms)
    pe=0.0; pt=time.ticks_ms(); hc=0.0; ht=time.ticks_ms(); found=False

    while time.ticks_diff(dl,time.ticks_ms())>0:
        # check color
        if color(C.PORT_C1)==target_color or color(C.PORT_C2)==target_color:
            found=True; break
        rL=reflect(C.PORT_C1); rR=reflect(C.PORT_C2)
        rL=CAL.mid(C.PORT_C1) if rL<0 else rL; rR=CAL.mid(C.PORT_C2) if rR<0 else rR
        lv=le.dual(rL,rR) if 'dual' in mode else le.single(rL,C.PORT_C1)
        now=time.ticks_ms()
        if 'gyro' in mode:
            dth=max(1,time.ticks_diff(now,ht))/1000.0; ht=now
            gz,_,_=_hub.motion.gyroscope(); hc+=(gz-bias)*dth
            he=-hc
            while he>180: he-=360
            while he<-180: he+=360
            a=0.7 if le.on_line else 0.1
            err=a*lv+(1-a)*(he/45.0)
        else: err=lv; he=0.0
        dt2=max(1,time.ticks_diff(now,pt))/1000.0; pt=now
        de=(err-pe)/dt2; pe=err; c=C.LF_KP_MED*err+C.LF_KD_MED*de+C.LF_KGYRO*he
        motor_pair.move_tank(motor_pair.PAIR_1,max(-1000,min(1000,int(vmax-c))),max(-1000,min(1000,int(vmax+c))))
        await runloop.sleep_ms(8)

    motor_pair.stop(motor_pair.PAIR_1)
    print(f"[lf_color] {'found '+cname(target_color) if found else 'timeout'}")
    return found


async def center_on_line(speed: int = 150, p: int = None,
                          max_ms: int = 2000) -> bool:
    """
    nudge robot until sensor is centered on line edge (reflect = mid)
    call after stopping at a junction to align precisely
    """
    import config as C
    print(f"{C.CLR_GRY}[sensor] center on line edge{C.CLR_RST}")
    p=p or C.PORT_C1; mid=CAL.mid(p)
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    dl=time.ticks_add(time.ticks_ms(),max_ms); settle=0
    while time.ticks_diff(dl,time.ticks_ms())>0:
        r=reflect(p)
        if r<0: await runloop.sleep_ms(8); continue
        err=mid-r   # positive = too far right (dark), negative = too far left (light)
        if abs(err)<3: settle+=1
        else: settle=0
        if settle>=10: break
        pwr=max(-speed,min(speed,int(err*1.5)))
        motor_pair.move_tank(motor_pair.PAIR_1,pwr,pwr)   # translate sideways
        await runloop.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1)
    print(f"[center] done  reflect={reflect(p)}  target={mid}")
    return True


async def detect_color_sequence(n: int = 3, p: int = None,
                                  vmax: int = 200,
                                  min_hold_ms: int = 100,
                                  max_ms: int = 10000) -> list:
    """
    drive slowly and record sequence of colors detected
    returns list of n colors detected in order
    use for: color sorting, reading mission instructions from field

    example: returns [RED, BLUE, GREEN] if robot passed those zones
    """
    print(f"[seq] detecting {n} colors  v={vmax}")
    p=p or C.PORT_C1
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    motor_pair.move_tank(motor_pair.PAIR_1,vmax,vmax)
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    seq=[]; last_c=NONE; hold_ms=0; hold_t=time.ticks_ms()

    while time.ticks_diff(dl,time.ticks_ms())>0 and len(seq)<n:
        c=color(p)
        if c not in (NONE,-1) and c!=last_c:
            if hold_ms==0: hold_t=time.ticks_ms()
            hold_ms=time.ticks_diff(time.ticks_ms(),hold_t)
            if hold_ms>=min_hold_ms:
                seq.append(c); print(f"[seq] detected {cname(c)}  ({len(seq)}/{n})")
                last_c=c; hold_ms=0
        else:
            if c!=last_c: hold_ms=0
        await runloop.sleep_ms(20)

    motor_pair.stop(motor_pair.PAIR_1)
    import config as C
    print(f"{C.CLR_GRY}[sensor] detected sequence: {[cname(c) for c in seq]}{C.CLR_RST}")
    return seq


async def align_to_wall_color(target_color: int,
                               approach_speed: int = 200,
                               push_ms: int = 400,
                               max_ms: int = 5000) -> bool:
    """
    drive until both sensors see target_color (colored wall/zone)
    then push for push_ms to align flat
    use for: parking in color zone, aligning before precision task
    """
    import config as C
    print(f"{C.CLR_GRY}[sensor] align to wall {cname(target_color)}{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    both_seen=False; confirm=0

    while time.ticks_diff(dl,time.ticks_ms())>0:
        c1=color(C.PORT_C1); c2=color(C.PORT_C2)
        both=(c1==target_color and c2==target_color)
        confirm=confirm+1 if both else 0
        if confirm>=5: both_seen=True; break
        motor_pair.move_tank(motor_pair.PAIR_1,approach_speed,approach_speed)
        await runloop.sleep_ms(8)

    if both_seen:
        motor_pair.move_tank(motor_pair.PAIR_1,approach_speed,approach_speed)
        await runloop.sleep_ms(push_ms)
    motor_pair.stop(motor_pair.PAIR_1)
    print(f"[align_col] {'aligned' if both_seen else 'timeout'}")
    return both_seen


async def until_color(target, vmax=400, mode="gyro",
                       heading=0.0, max_ms=10000) -> bool:
    """straight drive until color detected"""
    print(f"[until_color] target={cname(target)}  mode={mode}")
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    dl=time.ticks_add(time.ticks_ms(),max_ms)
    hc=heading; ht=time.ticks_ms(); I=pe=0.0; le=LineEst(0.3)
    while time.ticks_diff(dl,time.ticks_ms())>0:
        if color(C.PORT_C1)==target or color(C.PORT_C2)==target:
            motor_pair.stop(motor_pair.PAIR_1); print(f"[until_color] found"); return True
        now=time.ticks_ms(); dt=max(1,time.ticks_diff(now,ht))/1000.0; ht=now
        gz,_,_=_hub.motion.gyroscope(); hc+=gz*dt
        if mode=="line":
            r=reflect(C.PORT_C1); err=le.single(r,C.PORT_C1) if r>=0 else 0.0
            de=(err-pe)/dt; pe=err; c=35.0*err+8.0*de
        else:
            err=heading-hc; I=max(-60,min(60,I+err*dt))
            de=(err-pe)/dt; pe=err; c=3.5*err+0.01*I+2.0*de
        motor_pair.move_tank(motor_pair.PAIR_1,max(-1000,min(1000,int(vmax-c))),max(-1000,min(1000,int(vmax+c))))
        await runloop.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1); print(f"[until_color] timeout"); return False


async def until_line(vmax=400, heading=0.0, thr=None,
                      confirm_ms=80, max_ms=10000) -> bool:
    """straight drive until line detected"""
    import config as C
    print(f"{C.CLR_GRY}[sensor] until_line v={vmax}{C.CLR_RST}")
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    th=thr or CAL.mid(); dl=time.ticks_add(time.ticks_ms(),max_ms)
    hc=heading; ht=time.ticks_ms(); I=pe=0.0; lms=lst=0
    while time.ticks_diff(dl,time.ticks_ms())>0:
        r=reflect(C.PORT_C1); now=time.ticks_ms()
        dt=max(1,time.ticks_diff(now,ht))/1000.0; ht=now
        if r>=0 and r<=th:
            if lms==0: lst=now
            lms=time.ticks_diff(now,lst)
            if lms>=confirm_ms: break
        else: lms=0
        gz,_,_=_hub.motion.gyroscope(); hc+=gz*dt
        err=heading-hc; I=max(-60,min(60,I+err*dt))
        de=(err-pe)/dt; pe=err; c=3.5*err+0.01*I+2.0*de
        motor_pair.move_tank(motor_pair.PAIR_1,max(-1000,min(1000,int(vmax-c))),max(-1000,min(1000,int(vmax+c))))
        await runloop.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1)
    ok=lms>=confirm_ms; print(f"[until_line] {'found' if ok else 'timeout'}"); return ok


# ███████ ██   ██ ██████   █████  
# ██       ██   ██ ██   ██ ██   ██ 
# █████    ███████ ██████  ███████ 
# ██       ██   ██ ██   ██ ██   ██ 
# ███████  ██   ██ ██   ██ ██   ██ 
#
# >>extra tools

class JunctionDet:
    __slots__=('_cnt','count')
    def __init__(self): self._cnt=0; self.count=0
    def check(self):
        rL=reflect(C.PORT_C1); rR=reflect(C.PORT_C2)
        tL=CAL.mid(C.PORT_C1)-C.JUNC_OFFSET; tR=CAL.mid(C.PORT_C2)-C.JUNC_OFFSET
        if rL>=0 and rR>=0 and rL<=tL and rR<=tR:
            self._cnt+=1
            if self._cnt>=C.JUNC_CONFIRM: self._cnt=0; self.count+=1; return True
        else: self._cnt=0
        return False
    def reset(self): self._cnt=0; self.count=0


# landmark fix

class Landmark:
    __slots__=('_sv','_col','_ax','_hs','_tol','_done')
    def __init__(self,snap_cm,col,axis='y',heading_snap=None,tol_cm=None):
        self._sv=snap_cm; self._col=col; self._ax=axis
        self._hs=heading_snap; self._tol=tol_cm or C.LM_TOL_CM; self._done=False
    def poll(self,odo):
        if self._done: return False
        if color(C.PORT_C1)!=self._col: return False
        cur=odo.y if self._ax=='y' else odo.x
        if fabs(cur-self._sv)>self._tol: return False
        bf=str(odo)
        if self._ax=='y': odo.snap_y(self._sv)
        else: odo.snap_x(self._sv)
        if self._hs is not None: odo.snap_h(self._hs)
        self._done=True; print(f"[lm] fix {self._ax}={self._sv:.1f}  {bf}->{odo}"); return True
    def reset(self): self._done=False

class LandmarkMap:
    __slots__=('_lms',)
    def __init__(self,lms): self._lms=lms
    def poll(self,odo):
        for lm in self._lms: lm.poll(odo)
    def reset(self):
        for lm in self._lms: lm.reset()


# edge finder

async def find_line_center(spd=150,p=None)->float:
    p=p or C.PORT_C1; print(f"[edge] p={p}")
    motor_pair.pair(motor_pair.PAIR_1,C.PORT_L,C.PORT_R)
    thr=CAL.mid(p); lead=trail=None; on=False
    motor.reset_relative_position(C.PORT_L,0); motor.reset_relative_position(C.PORT_R,0)
    motor_pair.move_tank(motor_pair.PAIR_1,spd,spd)
    for _ in range(500):
        r=reflect(p); cnt=(motor.relative_position(C.PORT_L)+motor.relative_position(C.PORT_R))>>1
        if r>=0:
            if not on and r<thr-5: on=True; lead=cnt
            elif on and r>thr+5: trail=cnt; break
        time.sleep_ms(8)
    motor_pair.stop(motor_pair.PAIR_1)
    if lead is not None and trail is not None:
        cm=(lead+trail)*0.5*C.CM_PER_COUNT; w=(trail-lead)*C.CM_PER_COUNT
        print(f"[edge] centre={cm:.2f}cm  w={w:.2f}cm"); return cm
    print("[edge] not found"); return -1.0


# ██████  ██  █████   ██████  ███    ██  ██████  ███████ ████████ ██  ██████ ███████ 
# ██   ██ ██ ██   ██ ██       ████   ██ ██    ██ ██         ██    ██ ██      ██      
# ██   ██ ██ ███████ ██   ███ ██ ██  ██ ██    ██ ███████    ██    ██ ██      ███████ 
# ██   ██ ██ ██   ██ ██    ██ ██  ██ ██ ██    ██      ██    ██    ██ ██           ██ 
# ██████  ██ ██   ██  ██████  ██   ████  ██████  ███████    ██    ██  ██████ ███████ 
#
# >>diagnostics

def sensor_report():
    r1=reflect(C.PORT_C1); c1=color(C.PORT_C1)
    r2=reflect(C.PORT_C2); c2=color(C.PORT_C2)
    print(f"c1: r={r1} {cname(c1)}  |  c2: r={r2} {cname(c2)}")
    print(f"cal c1 w={CAL._w[C.PORT_C1]} b={CAL._b[C.PORT_C1]} mid={CAL._m[C.PORT_C1]}")
    print(f"cal c2 w={CAL._w[C.PORT_C2]} b={CAL._b[C.PORT_C2]} mid={CAL._m[C.PORT_C2]}")

async def live(ms=5000,iv=100):
    print(f"[live] {ms}ms..."); t0=time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(),t0)<ms:
        print(f"  c1 r={reflect(C.PORT_C1):3d} {cname(color(C.PORT_C1)):6s}"
              f"  c2 r={reflect(C.PORT_C2):3d} {cname(color(C.PORT_C2)):6s}")
        await runloop.sleep_ms(iv)
    print("[live] done")
