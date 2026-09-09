import asyncio, json, mimetypes, pathlib, urllib.parse, threading, time, sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from websockets.server import serve
from websockets.exceptions import ConnectionClosed

ROOT=pathlib.Path('/home/dual-arm-rubiks-cube-robot-v2/linux_web_debug')
SOURCE='/home/mycode1.3/software'
BRIDGE_PORT='/tmp/oldmotor'
BRIDGE_BAUD=1000000
CFG={'main.cfg','motion.cfg','testcase.txt'}
state={'brightness':128,'status':'Ready','last_error':''}
hardware_lock=threading.Lock()
serial_lock=threading.Lock()

def hw():
    sys.path.insert(0,SOURCE)
    import serial
    import cube_motion as legacy
    s=serial.Serial(BRIDGE_PORT,BRIDGE_BAUD,timeout=.3)
    return s,legacy

def positions():
    try:
        s,legacy=hw()
        with serial_lock:
            # Legacy IDs are finger, arm, finger, arm; report physical order.
            old=[legacy.cmd_get_pos(s,i) for i in range(1,5)]
            p=[old[1],old[0],old[3],old[2]]
        s.close(); return p
    except Exception: return [None]*4

def status_text():
    p=positions(); return f"{state['status']};angles:{','.join('--' if x is None else str(x) for x in p)};"

def motor_status_message():
    # Query right side first, then left, in physical motor order.
    motors=((1, 'right arm', 2), (2, 'right finger', 1),
            (3, 'left arm', 4), (4, 'left finger', 3))
    result=[]; s=None
    try:
        s,legacy=hw()
        with serial_lock:
            for physical_id,name,old_id in motors:
                try:
                    stat=legacy.cmd_stat(s, old_id)
                    if stat is None:
                        result.append({'id':physical_id,'name':name,'error':'no old-protocol reply'})
                    else:
                        result.append({'id':physical_id,'name':name,'position':stat[3],
                                       'moving':bool(stat[1]),'temperature':stat[2],
                                       'voltage_mv':stat[4]})
                except Exception as e:
                    result.append({'id':physical_id,'name':name,'error':str(e)})
    except Exception as e:
        result=[{'id':i,'name':n,'error':str(e)} for i,n,_ in motors]
    finally:
        if s is not None:
            s.close()
    return 'motor_status:'+json.dumps(result, ensure_ascii=False, separators=(',',':'))

def colors_message():
    return 'colors:'+','.join(['0','0','128']*54)+'U'*54

def run_motion_sequence(mc, seq):
    """Run original parser units intact; +N is a 2/3-token compound motion."""
    i = 0
    while i < len(seq):
        n = 1
        token = seq[i]
        if len(token) >= 2 and token[1] == '2' and i + 1 < len(seq):
            # Original cube_motion.motions() looks ahead for R+N/L+N
            # and optionally consumes the following R0/L0.
            if len(seq[i + 1]) >= 2 and seq[i + 1][1:] == '+N':
                n = 3 if i + 2 < len(seq) else 2
        state['status'] = f'STEP {i + 1}/{len(seq)} {" ".join(seq[i:i+n])}'
        mc.motions(seq[i:i+n])
        i += n

class HTTPHandler(BaseHTTPRequestHandler):
    def send_bytes(self,code,data,typ):
        self.send_response(code); self.send_header('Content-Type',typ); self.send_header('Content-Length',str(len(data))); self.send_header('Cache-Control','no-cache'); self.end_headers(); self.wfile.write(data)
    def do_GET(self):
        u=urllib.parse.urlsplit(self.path); q=urllib.parse.parse_qs(u.query)
        if u.path in ('/cfg/get','/cfg/download'):
            f=q.get('file',['main.cfg'])[0]
            if f not in CFG: self.send_error(403); return
            self.send_bytes(200,(ROOT/f).read_bytes(),'text/plain'); return
        p=ROOT/'index.html' if u.path in ('','/') else ROOT/u.path.lstrip('/')
        if p.is_file(): self.send_bytes(200,p.read_bytes(),mimetypes.guess_type(str(p))[0] or 'application/octet-stream')
        else: self.send_error(404)
    def do_POST(self):
        u=urllib.parse.urlsplit(self.path); q=urllib.parse.parse_qs(u.query)
        if u.path!='/cfg/save': self.send_error(404); return
        f=q.get('file',['main.cfg'])[0]
        if f not in CFG: self.send_error(403); return
        n=int(self.headers.get('Content-Length','0')); data=self.rfile.read(n)
        (ROOT/f).write_bytes(data); self.send_bytes(200,b'OK','text/plain')
    def log_message(self,*a): pass

def do_hw(cmd):
    global state
    s=None; legacy=None
    with hardware_lock:
        try:
            s,legacy=hw()
            with serial_lock:
                if cmd=='enable_all': result=legacy.cmd_enable(s,[1,2,3,4],True)
                elif cmd=='disable_all': result=legacy.cmd_enable(s,[1,2,3,4],False)
                elif cmd=='go_home': result=legacy.cmd_zero(s) is not None
                elif cmd in ('release_cube','clamp_cube'):
                    refs=legacy.cmd_zero(s)
                    mc=legacy.MotionCtrl(s,*refs)
                    mc.two_finger_init() if cmd == 'release_cube' else mc.two_finger_clamp()
                    result=True
                elif cmd.startswith('motions:'):
                    seq=cmd.split(':',1)[1].strip().split(); p=legacy.cmd_zero(s)
                    if any(x is None for x in p): raise RuntimeError('motor position read failed')
                    legacy.cmd_enable(s,[1,2,3,4],True); mc=legacy.MotionCtrl(s,*p)
                    run_motion_sequence(mc, seq)
                    result=True
                elif cmd.startswith('test_motions:'):
                    seq=cmd.split(':',1)[1].strip().split()
                    p=[legacy.cmd_get_pos(s,i) for i in range(1,5)]
                    if any(x is None for x in p): raise RuntimeError('motor position read failed')
                    legacy.cmd_enable(s,[1,2,3,4],True); mc=legacy.MotionCtrl(s,*p)
                    run_motion_sequence(mc, seq)
                    result=True
                elif cmd.startswith('scramble:'):
                    seq=cmd.split(':',1)[1].strip().split(); p=legacy.cmd_zero(s)
                    if any(x is None for x in p): raise RuntimeError('motor position read failed')
                    legacy.cmd_enable(s,[1,2,3,4],True); mc=legacy.MotionCtrl(s,*p)
                    run_motion_sequence(mc, seq)
                    result=True
                else: result=True
            s.close(); state['status']='OK' if result else 'Rejected'; state['last_error']=''
        except Exception as e:
            if s is not None and legacy is not None:
                try: legacy.cmd_enable(s,[1,2,3,4],False)
                except: pass
            state['status']='ERROR STOPPED/DISABLED'; state['last_error']=str(e); result=False
    return result

async def ws_handler(ws):
    await ws.send('brightness:'+str(state['brightness']))
    await ws.send('positions:(80,60),(160,20),(280,60),(80,180),(160,220),(270,200)')
    await ws.send('scramble_sequence:')
    async for msg in ws:
        if msg=='get_brightness': await ws.send('brightness:'+str(state['brightness']))
        elif msg=='get_pos': await ws.send('positions:(80,60),(160,20),(280,60),(80,180),(160,220),(270,200)')
        elif msg=='get_scramble': await ws.send('scramble_sequence:')
        elif msg=='get_colors': await ws.send('status:'+await asyncio.to_thread(status_text)+colors_message())
        elif msg=='get_motor_status': await ws.send(await asyncio.to_thread(motor_status_message))
        elif msg.startswith('brightness:'):
            state['brightness']=int(msg.split(':',1)[1]); await ws.send(msg)
        elif msg.startswith('update_pos:'): await ws.send('positions:'+msg.split(':',1)[1])
        elif msg in ('enable_all','disable_all','go_home','release_cube','clamp_cube') or msg.startswith('motions:') or msg.startswith('test_motions:') or msg.startswith('scramble:'):
            p=await asyncio.to_thread(positions)
            await ws.send('status:BUSY;angles:'+','.join(map(str,p))+';'+colors_message())
            ok=await asyncio.to_thread(do_hw,msg)
            final_status=await asyncio.to_thread(status_text)
            await ws.send('status:'+final_status+colors_message())
        elif msg in ('save','restore'):
            await ws.send('status:'+await asyncio.to_thread(status_text)+colors_message())

async def ws_main():
    async with serve(ws_handler,'0.0.0.0',8081): await asyncio.Future()

def http_main(): ThreadingHTTPServer(('0.0.0.0',8080),HTTPHandler).serve_forever()
threading.Thread(target=http_main,daemon=True).start()
asyncio.run(ws_main())
