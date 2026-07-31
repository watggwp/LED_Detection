#!/usr/bin/env python3
"""webapp.py — หน้าเว็บตรวจสีไฟ LED หลายจุด (ครอบ multiwatch.py)

รัน:  ./.venv/bin/python webapp.py            # เปิด http://localhost:8000
      ./.venv/bin/python webapp.py --cam 0 --port 8000

หน้าเว็บ: เลือกกล้อง (คลิกภาพ) → ล็อกแสง → กรอกจำนวนชิ้น × ดวงต่อชิ้น → Start
→ เห็นภาพสด + ตารางสถานะรายชิ้น (ดวงที่หาไม่เจอแสดงเป็น "ไม่พบ" ไม่ทำทั้งจอพัง)

ค่าเริ่มต้นเปิดเฉพาะเครื่องนี้ (localhost) — ถ้าต้องการให้เครื่องอื่นใน LAN เห็น
(เช่นมือถือหน้าไลน์ผลิต) รันด้วย --host 0.0.0.0 เอง โดยรับความเสี่ยงว่า
ภาพกล้อง+ปุ่มควบคุมจะเปิดให้ทั้งวงโดยไม่มีรหัสผ่าน

ข้ามแพลตฟอร์ม: ล็อกแสงบน mac ใช้ tools/uvc-util (นิ่งสุดกับ C200), บน Windows/Linux
ใช้ OpenCV property แบบ best-effort — ตั้งไม่ได้ก็ไม่ crash แค่แจ้งแล้วใช้ auto
"""

import argparse
import base64
import json
import math
import os
import subprocess
import sys
import threading
import time
from typing import Optional

import cv2
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel

from multiwatch import MAX_SPOTS, MultiWatch

app = FastAPI(title="LED colorwatch")
CAM_INDEX = 0
IDLE_RELEASE_SEC = 60.0   # ไม่มีคนดู stream + ไม่ได้ตรวจ เกินนี้ → ปล่อยกล้อง (ประหยัดแบต)
DEFAULT_EXPOSURE = 5      # auto-ล็อกแสงต่ำตอนเปิดกล้อง (ภาพเห็นแต่ดวงไฟบนพื้นดำ)
THRESH = 4.0              # เกณฑ์ NG (ห่างอ้างอิง/ห่างกันเอง เกินค่านี้) — ปรับด้วย --thresh
# NB: 4.0 เป็นค่ามาตรฐานเดิม; ลดลงได้ถ้าของจริงเพี้ยนแบบจางๆ (เช่นต่างกันแค่ ~3 หน่วย)
# แต่ยิ่งต่ำยิ่งเสี่ยงเด้ง OK/NG สลับจาก noise — จูนกับของจริง

# ล็อกแสงเฉพาะ mac ผ่าน uvc-util (OpenCV/AVFoundation สั่ง C200 ไม่ได้ — ทดสอบ 2026-06-12)
UVC_UTIL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "tools", "uvc-util")
C200_VID_PID = "0x291a:0x3369"
# VID/PID ที่ใช้ผูกกล้องบน Windows (เลือกกล้องด้วยรหัสนี้แทน index คงที่ — สลับพอร์ต USB
# แล้วยังเจอตัวเดิม) ค่าเริ่มต้น = Anker C200; เปลี่ยนรุ่นกล้องในอนาคตแก้ที่นี่ หรือ --vidpid
CAMERA_VIDPID = C200_VID_PID

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "calibration.json")


# ---------- config (จำกล้องที่เลือกไว้ ข้าม session) ----------

def load_config():
    try:
        with open(CONFIG_FILE) as f:
            return json.load(f)
    except Exception:
        return {}


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f)
    except OSError:
        pass


def load_reference():
    try:
        with open(CALIB_FILE) as f:
            return json.load(f).get("reference")
    except Exception:
        return None


def save_reference(ref):
    with open(CALIB_FILE, "w") as f:
        json.dump({"reference": ref}, f)


# ---------- ล็อกแสง (A3: แยกชั้นตามแพลตฟอร์ม) ----------

def _use_uvc():
    """mac + มี uvc-util → ใช้ทางนั้น (นิ่งสุดกับ C200) ไม่งั้นใช้ OpenCV property"""
    return sys.platform == "darwin" and os.path.exists(UVC_UTIL)


def _set_exposure_uvc(value):
    """ตั้ง exposure ผ่าน uvc-util (mac/C200): int = manual / None = auto"""
    def run(setting):
        return subprocess.run([UVC_UTIL, "-V", C200_VID_PID, "-s", setting],
                              capture_output=True, text=True, timeout=10)
    try:
        if value is None:
            r = run("auto-exposure-mode=8")
            if r.returncode != 0:
                r = run("auto-exposure-mode=2")
            run("white-balance-temp=default")
            return r.returncode == 0, "แสงอัตโนมัติ"
        value = max(1, min(1000, int(value)))
        r1 = run("auto-exposure-mode=1")
        r2 = run(f"exposure-time-abs={value}")
        run("white-balance-temp=4500")   # ล็อก WB คงที่ — จำเป็นกับโหมดเทียบขาวอ้างอิง
        ok = r1.returncode == 0 and r2.returncode == 0
        return ok, (f"ล็อกแสงที่ {value} + WB 4500K" if ok
                    else (r1.stderr or r2.stderr).strip()[-120:])
    except Exception as e:
        return False, str(e)


def _win_exposure(value):
    """แปลงค่า UI (1..1000, น้อย=มืด แบบฝั่ง mac) → ค่า exposure ของ OpenCV บน Windows/Linux
    ซึ่งเป็น log2 วินาทีและต้องติดลบ (เลขยิ่งลบ=ยิ่งมืด) — เลขบวกไม่มีผล (ภาพสว่างค้าง)
    map: value 1 → -13 (มืดสุด), value 1000 → -1 (สว่าง); default 5 → ~-10 (มืดพอตัดแสงฟุ้ง)"""
    return int(round(max(-13.0, min(-1.0, -13.0 + 1.2 * math.log2(max(1, value))))))


def _set_exposure_opencv(value, cap):
    """ตั้ง exposure ผ่าน OpenCV property (Windows/Linux) แบบ best-effort
    หน่วย/ค่าต่างตาม backend (DSHOW/MSMF/V4L2) — ตั้งไม่ได้ก็ไม่ crash
    NB: readback ของ CAP_PROP_* บน MSMF เชื่อไม่ได้ (ค้าง) จึงยืนยันผลด้วยความสว่างจริง"""
    if cap is None:
        return False, "ยังไม่ได้ต่อกล้อง — รอภาพขึ้นก่อนแล้วลองใหม่"
    try:
        if value is None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)   # 0.75 = auto บนหลาย backend
            cap.set(cv2.CAP_PROP_AUTO_WB, 1)
            return True, "🔄 กลับเป็นแสงอัตโนมัติแล้ว"
        value = max(1, min(1000, int(value)))
        win_exp = _win_exposure(value)
        # manual: MSMF ใช้ 0 / DSHOW,V4L2 ใช้ 0.25 — ยิงทั้งสองค่ากันพลาด
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        cap.set(cv2.CAP_PROP_EXPOSURE, win_exp)          # ส่งค่าติดลบ (ตามสเกล log2 ของ OS นี้)
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4500)
        # กล้องใช้เวลา 2-3 เฟรมกว่าแสงใหม่จะมีผล — อ่านทิ้งก่อนแล้วค่อยวัดความสว่างจริง
        # (set() คืน True หลอกได้บน MSMF จึงไม่เชื่อ ใช้ความสว่างภาพตัดสินแทน)
        bright = None
        for _ in range(6):
            ok2, fr = cap.read()
            if ok2 and fr is not None:
                bright = float(fr.mean())
        if bright is not None and bright < 130:
            return True, "✅ ล็อกแสงแล้ว — ภาพมืดลง เห็นแต่ดวงไฟ พร้อมตรวจ"
        return True, ("ภาพยังสว่างอยู่ — กดปุ่ม “มืดลง” อีกสัก 1-2 ครั้ง "
                      "หรือปิด/หรี่ไฟในห้องให้มืดลง")
    except Exception as e:
        return False, f"ตั้ง exposure ไม่ได้: {e}"


def set_exposure(value, cap=None):
    """ตั้ง exposure ข้ามแพลตฟอร์ม — คืน (ok, msg)
    NB: เส้นทาง OpenCV ต้องเรียกใน thread ที่ถือ cap เท่านั้น (กัน race กับ read())"""
    if _use_uvc():
        return _set_exposure_uvc(value)
    return _set_exposure_opencv(value, cap)


# ---------- เลือกกล้องด้วย VID/PID (Windows) ----------
# ผูกกล้องด้วย VID/PID แทน index คงที่ → สลับพอร์ต/รีบูตแล้วยังเจอตัวเดิม
# และเปลี่ยนรุ่นกล้องในอนาคตได้แค่แก้ค่า (ไม่กระทบ mac/Linux ซึ่งคืน None แล้วใช้ index เดิม)

def _parse_vidpid(s):
    """'0x291a:0x3369' หรือ '291A:3369' → ('291A', '3369'); รูปแบบผิด → (None, None)"""
    try:
        vid, pid = s.lower().replace("0x", "").split(":")
        return vid.strip().upper(), pid.strip().upper()
    except Exception:
        return None, None


def _win_camera_names():
    """ชื่อกล้องเรียงตาม index ของ OpenCV บน Windows (ผ่าน pygrabber) — [] ถ้าใช้ไม่ได้"""
    try:
        from pygrabber.dshow_graph import FilterGraph
        return FilterGraph().get_input_devices()
    except Exception:
        return []


def _win_names_for_vidpid(vid, pid):
    """ชื่อกล้องที่ VID/PID ตรง (Windows, ผ่าน WMI) — ใช้จับคู่กับชื่อจาก pygrabber"""
    try:
        q = ("Get-CimInstance Win32_PnPEntity | Where-Object "
             f"{{ $_.PNPDeviceID -match 'VID_{vid}.*PID_{pid}' }} | "
             "Select-Object -ExpandProperty Name")
        r = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", q],
            capture_output=True, text=True, timeout=10)
        return [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
    except Exception:
        return []


def find_index_by_vidpid(vidpid):
    """คืน OpenCV camera index ของกล้องที่ VID/PID ตรง (Windows, best-effort)
    หาไม่เจอ / ไม่ใช่ Windows / pygrabber ใช้ไม่ได้ → None (ผู้เรียก fallback ไป index เดิม)"""
    if sys.platform != "win32" or not vidpid:
        return None
    vid, pid = _parse_vidpid(vidpid)
    if not vid or not pid:
        return None
    names = _win_camera_names()
    targets = _win_names_for_vidpid(vid, pid)
    for i, n in enumerate(names):
        for t in targets:
            if n and t and (n.lower() in t.lower() or t.lower() in n.lower()):
                return i
    return None


# ---------- กล้อง ----------

def open_cap(index):
    """เปิดกล้องแบบไม่ sys.exit (ต่างจาก colorwatch.open_camera) — คืน None ถ้าเปิดไม่ได้"""
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap


def _thumb(frame, width=220):
    h, w = frame.shape[:2]
    small = cv2.resize(frame, (width, max(1, int(h * width / w))))
    ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return "data:image/jpeg;base64," + base64.b64encode(buf).decode() if ok else None


class Session:
    """สถานะกล้อง+การตรวจ — กล้องเปิดเมื่อมีคนดู/ตรวจ, ปล่อยเองตอน idle (D11)"""

    def __init__(self):
        self.lock = threading.Lock()
        self.thread = None
        self.stop_flag = False
        self.cap = None        # cap ปัจจุบัน (ให้ A3 OpenCV ตั้ง property ได้ ใน thread นี้)
        self.engine = None     # None = โหมดพรีวิว (โชว์ภาพเฉยๆ ไม่ตรวจ)
        self.pieces = 0
        self.per_piece = 0
        self.jpeg = None       # เฟรมล่าสุด (bytes)
        self.status = None     # สถานะล่าสุด (dict, มี cct)
        self.frame = None      # เฟรมดิบล่าสุด (ก่อน annotate) — ใช้ตอน calibrate
        self.reference = load_reference()  # b* ขาวอ้างอิง (คงไว้ข้าม start/stop)
        self.last_stream = time.time()     # เวลาเฟรมล่าสุดที่ stream ถูกดึง (D11)
        self.cam_error = None
        # คิวคำสั่งตั้ง exposure ฝั่ง OpenCV (ต้องทำใน capture thread ที่ถือ cap)
        self.exp_pending = False
        self.exp_value = None
        self.exp_event = threading.Event()
        self.exp_result = (False, "")

    def detecting(self):
        return self.engine is not None


SES = Session()
EXPOSURE_INIT = DEFAULT_EXPOSURE   # ตั้งจาก --exposure: ล็อกแสงทันทีที่กล้องเปิด


def capture_loop(ses):
    """thread ถือกล้อง — สลับพรีวิว/ตรวจตาม ses.engine, ปล่อยกล้องเองเมื่อ idle"""
    global CAM_INDEX
    cap = open_cap(CAM_INDEX)
    if cap is None:
        # index ที่จำไว้เปิดไม่ได้ (อาจสลับพอร์ต) — ลองหาใหม่จาก VID/PID (Windows)
        alt = find_index_by_vidpid(CAMERA_VIDPID)
        if alt is not None and alt != CAM_INDEX:
            cap = open_cap(alt)
            if cap is not None:
                CAM_INDEX = alt
    if cap is None:
        with ses.lock:
            ses.cam_error = (f"เปิดกล้อง index {CAM_INDEX} ไม่ได้ — "
                             f"กด 'เลือกกล้อง' แล้วคลิกตัวที่เห็นไฟ LED")
            ses.jpeg = None
        return
    with ses.lock:
        ses.cap = cap
        ses.cam_error = None
    if EXPOSURE_INIT is not None:
        set_exposure(EXPOSURE_INIT, cap)   # in-thread → ปลอดภัยทั้ง uvc และ OpenCV
    last_cct = 0.0
    try:
        while not ses.stop_flag:
            # คำสั่งตั้ง exposure ฝั่ง OpenCV (ทำในนี้เพราะเป็นเจ้าของ cap)
            if ses.exp_pending:
                ses.exp_result = _set_exposure_opencv(ses.exp_value, cap)
                ses.exp_pending = False
                ses.exp_event.set()

            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            now = time.time()
            with ses.lock:
                engine = ses.engine
                ses.frame = frame.copy()  # เก็บเฟรมดิบไว้ให้ calibrate
                idle = (now - ses.last_stream)
            # D11: ไม่ได้ตรวจ + ไม่มีคนดู stream นานเกิน → ปล่อยกล้อง
            if engine is None and idle > IDLE_RELEASE_SEC:
                break
            if engine is not None:
                want_cct = (now - last_cct) >= 1.0
                status = engine.update(frame, now, with_cct=want_cct)
                engine.annotate(frame, status)
            else:
                want_cct = False
                status = None
                cv2.putText(frame, "PREVIEW", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 200, 255), 2)
            ok2, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with ses.lock:
                if ok2:
                    ses.jpeg = buf.tobytes()
                if engine is not None and (want_cct or "error" in status):
                    ses.status = status
            if want_cct:
                last_cct = now
    finally:
        cap.release()
        with ses.lock:
            ses.cap = None


def ensure_camera():
    if SES.thread is None or not SES.thread.is_alive():
        SES.stop_flag = False
        SES.last_stream = time.time()
        SES.thread = threading.Thread(target=capture_loop, args=(SES,), daemon=True)
        SES.thread.start()


def stop_camera():
    """หยุด capture thread + ปล่อยกล้อง (รอ join) — ใช้ก่อนสแกน/เปลี่ยนกล้อง"""
    SES.stop_flag = True
    if SES.thread is not None:
        SES.thread.join(timeout=3.0)
    SES.thread = None


# ---------- โมเดล request ----------

class StartReq(BaseModel):
    pieces: int
    per_piece: int = 2


class ExposureReq(BaseModel):
    value: Optional[int] = None  # None = auto


class CamReq(BaseModel):
    index: int


# ---------- endpoints ----------

@app.post("/api/exposure")
def api_exposure(req: ExposureReq):
    ensure_camera()
    if _use_uvc():
        ok, msg = set_exposure(req.value)
        return {"ok": ok, "msg": msg}
    # OpenCV: ส่งให้ capture thread ตั้งให้ (กัน race กับ cap.read())
    SES.exp_value = req.value
    SES.exp_result = (False, "หมดเวลา — กล้องอาจยังไม่พร้อม ลองใหม่")
    SES.exp_event.clear()
    SES.exp_pending = True
    SES.exp_event.wait(timeout=2.5)
    ok, msg = SES.exp_result
    return {"ok": ok, "msg": msg}


@app.get("/api/cameras")
def api_cameras():
    """ไล่เปิด OpenCV index 0-5 อ่าน 1 เฟรม ส่ง thumbnail ให้ผู้ใช้คลิกเลือกตัวที่เห็น LED"""
    stop_camera()   # ปล่อยกล้องก่อน ไม่งั้นเปิดซ้ำ index เดิมไม่ได้
    cams = []
    try:
        for i in range(6):
            cap = cv2.VideoCapture(i)
            ok, fr = (cap.read() if cap.isOpened() else (False, None))
            cap.release()
            if ok and fr is not None:
                cams.append({"index": i, "thumb": _thumb(fr),
                             "current": i == CAM_INDEX})
    finally:
        ensure_camera()   # เปิดกล้องปัจจุบันคืน
    return {"cameras": cams, "current": CAM_INDEX}


@app.post("/api/select_camera")
def api_select_camera(req: CamReq):
    global CAM_INDEX
    stop_camera()
    CAM_INDEX = req.index
    cfg = load_config()
    cfg["cam_index"] = req.index
    save_config(cfg)
    with SES.lock:
        SES.engine = None   # กล้องเปลี่ยน — เริ่มตรวจใหม่ค่อยยึดสล็อตใหม่
        SES.status = None
        SES.jpeg = None
    ensure_camera()
    return {"ok": True, "index": req.index}


@app.post("/api/start")
def api_start(req: StartReq):
    total = req.pieces * req.per_piece
    if req.pieces < 1 or req.per_piece < 1:
        return JSONResponse({"error": "จำนวนต้องมากกว่า 0"}, status_code=400)
    if total < 2:
        return JSONResponse({"error": "ต้องมีอย่างน้อย 2 ดวงถึงเทียบกันได้"},
                            status_code=400)
    if total > MAX_SPOTS:
        return JSONResponse(
            {"error": f"รวม {total} ดวง เกินขีดจำกัด {MAX_SPOTS} ดวงต่อรอบ "
                      f"(แบ่งเทสทีละไม่เกิน {MAX_SPOTS // req.per_piece} ชิ้น)"},
            status_code=400)
    ensure_camera()
    with SES.lock:
        SES.pieces, SES.per_piece = req.pieces, req.per_piece
        SES.status = None
        SES.engine = MultiWatch(total, thresh=THRESH, reference=SES.reference)
    return {"ok": True, "total_spots": total,
            "mode": "absolute" if SES.reference is not None else "relative"}


@app.post("/api/calibrate")
def api_calibrate():
    """วางชิ้นดีครบตามจำนวน → เรียกตอนกำลังตรวจ — ตั้งขาวอ้างอิงและจำถาวร"""
    with SES.lock:
        engine = SES.engine
        frame = SES.frame.copy() if SES.frame is not None else None
    if engine is None:
        return JSONResponse({"error": "กดเริ่มตรวจก่อน แล้วค่อย calibrate "
                                      "(ต้องรู้จำนวนจุดก่อน)"}, status_code=400)
    if frame is None:
        return JSONResponse({"error": "ยังไม่มีภาพจากกล้อง"}, status_code=400)
    ref, msg = engine.calibrate(frame)
    if ref is None:
        return JSONResponse({"error": msg}, status_code=400)
    with SES.lock:
        SES.reference = ref
    save_reference(ref)
    return {"ok": True, "reference": round(ref, 2), "msg": msg}


@app.post("/api/calibrate/clear")
def api_calibrate_clear():
    """ล้างขาวอ้างอิง — กลับโหมดเทียบกันเอง"""
    with SES.lock:
        SES.reference = None
        if SES.engine is not None:
            SES.engine.reference = None
    try:
        os.remove(CALIB_FILE)
    except OSError:
        pass
    return {"ok": True}


@app.post("/api/stop")
def api_stop():
    with SES.lock:
        SES.engine = None  # กลับโหมดพรีวิว — กล้องยังเปิดอยู่ (จะปล่อยเองตอน idle)
        SES.status = None
    return {"ok": True}


@app.get("/api/status")
def api_status():
    with SES.lock:
        st = SES.status
        detecting = SES.detecting()
        cam_error = SES.cam_error
    if not detecting:
        return {"running": False, "preview": True, "cam_error": cam_error,
                "thresh": THRESH}
    out = {"running": True, "pieces": SES.pieces, "per_piece": SES.per_piece,
           "max_spots": MAX_SPOTS, "thresh": THRESH}
    if st is None:
        out["error"] = "กำลังเริ่มตรวจ..."
        return out
    if "error" in st:
        out["error"] = st["error"]
        return out
    # จัดกลุ่มจุดเป็นรายชิ้น: ชิ้นที่ j = จุด (j-1)*per+1 .. j*per (ซ้าย→ขวา)
    pieces = []
    for j in range(SES.pieces):
        spots = st["spots"][j * SES.per_piece:(j + 1) * SES.per_piece]
        sts = [s["state"] for s in spots]
        if "BAD" in sts:
            pstate = "BAD"
        elif "MISSING" in sts:
            pstate = "INCOMPLETE"   # มีดวงหาย — ยังตัดสินไม่ครบ
        else:
            pstate = "OK"
        pieces.append({"piece": j + 1, "state": pstate, "spots": spots})
    out["pieces_detail"] = pieces
    out["t"] = st["t"]
    out["mode"] = st.get("mode", "relative")
    out["ref"] = st.get("ref")
    out["n_missing"] = st.get("n_missing", 0)
    return out


@app.get("/stream")
def stream():
    ensure_camera()

    def gen():
        while True:
            with SES.lock:
                buf = SES.jpeg
                SES.last_stream = time.time()   # บอกว่ายังมีคนดูอยู่ (D11)
            if buf is None:
                time.sleep(0.2)
                continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n")
            time.sleep(0.1)  # ~10 fps พอสำหรับงานเฝ้าดู
    return StreamingResponse(gen(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


PAGE = """<!doctype html>
<html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ตรวจสีไฟ LED</title>
<style>
  body { font-family: -apple-system, 'Sukhumvit Set', sans-serif; margin: 0;
         background: #14171c; color: #e8eaed; }
  header { padding: 14px 22px; background: #1d2229; font-size: 20px; font-weight: 600; }
  main { display: flex; flex-wrap: wrap; gap: 18px; padding: 18px 22px; }
  .panel { background: #1d2229; border-radius: 12px; padding: 16px; }
  #controls label { margin-right: 6px; }
  input[type=number] { width: 64px; font-size: 16px; padding: 6px; border-radius: 6px;
         border: 1px solid #3a4250; background: #11141a; color: #e8eaed; }
  button { font-size: 16px; padding: 8px 22px; border-radius: 8px; border: 0;
         cursor: pointer; margin-left: 10px; }
  #btnStart { background: #2e7d32; color: #fff; }
  #btnStop  { background: #5a6270; color: #fff; }
  #video { max-width: 100%; border-radius: 10px; display: block; }
  table { border-collapse: collapse; min-width: 340px; }
  th, td { padding: 8px 14px; text-align: center; border-bottom: 1px solid #2c333d; }
  .ok  { color: #6fdc8c; font-weight: 700; }
  .bad { color: #ff6b6b; font-weight: 700; }
  .miss { color: #9aa3b2; font-weight: 700; }
  tr.badrow { background: #3a1d1d; }
  tr.missrow { background: #23272e; }
  .banner { font-size: 34px; font-weight: 800; text-align: center;
            padding: 16px; border-radius: 10px; margin-bottom: 14px;
            letter-spacing: 2px; }
  .bok { background: #1e4620; color: #7dff9b; }
  .bng { background: #5a1717; color: #ff8080; animation: blink 1s step-end infinite; }
  .binc { background: #463a1e; color: #ffd479; }
  @keyframes blink { 50% { background: #7a1f1f; } }
  .banner small { display: block; font-size: 15px; font-weight: 500;
                  letter-spacing: 0; margin-top: 4px; }
  #msg { margin-top: 10px; color: #ffb86b; min-height: 22px; }
  .hint { color: #8a93a3; font-size: 13px; margin-top: 6px; }
  #cams { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 10px; }
  .camcard { border: 2px solid #3a4250; border-radius: 8px; padding: 6px; cursor: pointer;
             text-align: center; background: #11141a; }
  .camcard.cur { border-color: #6fdc8c; }
  .camcard img { display: block; border-radius: 5px; width: 220px; max-width: 40vw; }
  .camcard span { font-size: 13px; color: #c4cad4; }
</style></head>
<body>
<header>🔍 ตรวจสีไฟ LED — ฝั่งไหนเพี้ยน (ฟ้า)</header>
<main>
  <div class="panel" style="flex:1 1 640px">
    <div id="controls">
      <div style="margin-bottom:10px">
        <button onclick="loadCams()" style="background:#3d5a80;color:#fff">📷 เลือกกล้อง</button>
        <span class="hint">กล้องสลับ index เอง? กดปุ่มนี้แล้วคลิกภาพที่เห็นไฟ LED</span>
        <div id="cams"></div>
      </div>
      <label>จำนวนชิ้น <input type="number" id="pieces" value="1" min="1" max="5"></label>
      <label>ดวงต่อชิ้น <input type="number" id="per" value="2" min="1" max="4"></label>
      <button id="btnStart" onclick="start()">▶ เริ่มตรวจ</button>
      <button id="btnStop" onclick="stop()">⏸ หยุด (กลับพรีวิว)</button>
      <div style="margin-top:10px">
        <span style="font-weight:600">💡 ปรับแสงภาพ:</span>
        <button onclick="darker()">🌙 มืดลง</button>
        <button onclick="brighter()">☀️ สว่างขึ้น</button>
        <button onclick="autoExp()">🔄 อัตโนมัติ</button>
        <span class="hint">ภาพฟุ้งขาว? กด “มืดลง” จนเห็นแต่ดวงไฟบนพื้นดำ</span>
      </div>
      <div style="margin-top:10px">
        <button onclick="calibrate()" style="background:#7b5dbe;color:#fff">🎯 Calibrate ขาว (วางชิ้นดีให้ครบก่อนกด)</button>
        <button onclick="clearCalib()">ล้างอ้างอิง</button>
        <span id="calinfo" class="hint"></span>
      </div>
      <div class="hint">ภาพควรเห็นแต่ดวงไฟบนพื้นดำ (โปรแกรมล็อกแสงต่ำให้อัตโนมัติแล้ว)
        ถ้ายังฟุ้งขาว กด "ล็อกแสง" ค่าต่ำๆ — จัดวางชิ้นงานเรียงแถวแนวนอนให้เข้าเฟรม
        ก่อนกดเริ่มตรวจ (รวมไม่เกิน <span id="maxs">10</span> ดวงต่อรอบ)
        ดวงที่หาไม่เจอจะขึ้น "ไม่พบ" — ดวงอื่นยังตรวจได้ปกติ</div>
      <div id="msg"></div>
    </div>
    <img id="video" src="/stream" alt="กำลังต่อกล้อง...">
  </div>
  <div class="panel" style="flex:0 1 420px">
    <h3 style="margin-top:4px">สถานะรายชิ้น (ซ้าย → ขวา)</h3>
    <div id="result">📷 โหมดพรีวิว — กดเริ่มตรวจเมื่อจัดวางเสร็จ</div>
  </div>
  <div class="panel" style="flex:1 1 100%">
    <h3 style="margin-top:4px">⏱ นาฬิกาจับเวลา (นับถอยหลัง)</h3>
    <label>นาที <input type="number" id="tmin" value="5" min="0" max="999"></label>
    <label>วินาที <input type="number" id="tsec" value="0" min="0" max="59"></label>
    <button onclick="timerStart()" style="background:#2e7d32;color:#fff">▶ เริ่มจับเวลา</button>
    <button onclick="timerStop()">รีเซ็ต</button>
    <span id="tdisp" style="font-size:42px;font-weight:800;margin-left:18px;font-variant-numeric:tabular-nums">--:--</span>
    <div id="talarm" class="banner bng" style="display:none;margin-top:10px">⏰ ครบเวลาแล้ว!</div>
  </div>
</main>
<script>
setInterval(poll, 1000);
async function loadCams() {
  document.getElementById('msg').textContent = 'กำลังสแกนกล้อง... (ภาพอาจกระตุกแป๊บ)';
  const j = await (await fetch('/api/cameras')).json();
  const box = document.getElementById('cams');
  box.innerHTML = '';
  if (!j.cameras.length) { box.innerHTML = '<span class="hint">ไม่พบกล้อง — เช็คสาย USB</span>'; }
  for (const c of j.cameras) {
    const d = document.createElement('div');
    d.className = 'camcard' + (c.current ? ' cur' : '');
    d.innerHTML = `<img src="${c.thumb}"><span>กล้อง #${c.index}${c.current?' (ใช้อยู่)':''}</span>`;
    d.onclick = () => selectCam(c.index);
    box.appendChild(d);
  }
  document.getElementById('msg').textContent = 'คลิกกล้องที่เห็นไฟ LED';
}
async function selectCam(i) {
  await fetch('/api/select_camera', {method:'POST',
    headers:{'Content-Type':'application/json'}, body: JSON.stringify({index: i})});
  document.getElementById('msg').textContent = 'เลือกกล้อง #' + i + ' แล้ว';
  document.getElementById('cams').innerHTML = '';
  // รีโหลดภาพสด (cache-bust)
  document.getElementById('video').src = '/stream?ts=' + Date.now();
}
async function start() {
  const pieces = +document.getElementById('pieces').value;
  const per = +document.getElementById('per').value;
  const r = await fetch('/api/start', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({pieces, per_piece: per})});
  const j = await r.json();
  document.getElementById('msg').textContent = j.error || '';
}
async function stop() {
  await fetch('/api/stop', {method:'POST'});
  document.getElementById('msg').textContent = '';
}
async function calibrate() {
  const r = await fetch('/api/calibrate', {method:'POST'});
  const j = await r.json();
  document.getElementById('msg').textContent = j.ok ? '✓ ' + j.msg : '✗ ' + j.error;
}
async function clearCalib() {
  await fetch('/api/calibrate/clear', {method:'POST'});
  document.getElementById('msg').textContent = 'ล้างอ้างอิงแล้ว — กลับโหมดเทียบกันเอง';
}
let expLevel = 5;            // ระดับแสงปัจจุบัน (น้อย = มืด) ตรงกับ DEFAULT_EXPOSURE ตอนเปิดกล้อง
async function applyExp(v) {
  const j = await (await fetch('/api/exposure', {method:'POST',
    headers:{'Content-Type':'application/json'}, body: JSON.stringify({value: v})})).json();
  document.getElementById('msg').textContent = j.msg || (j.ok ? 'เรียบร้อย' : 'ลองใหม่อีกครั้ง');
}
function darker()   { expLevel = Math.max(1, Math.round(expLevel / 2));   applyExp(expLevel); }
function brighter() { expLevel = Math.min(1000, Math.max(2, expLevel * 2)); applyExp(expLevel); }
async function autoExp() { expLevel = 5; await applyExp(null); }
function spotName(idx, per) {
  if (per === 2) return idx === 1 ? 'ซ้าย' : 'ขวา';
  return 'ดวง ' + idx;
}
async function poll() {
  const j = await (await fetch('/api/status')).json();
  const el = document.getElementById('result');
  if (!j.running) {
    el.innerHTML = j.cam_error
      ? '⚠️ ' + j.cam_error
      : '📷 โหมดพรีวิว — กดเริ่มตรวจเมื่อจัดวางเสร็จ';
    return;
  }
  if (j.error) { el.innerHTML = '⚠️ ' + j.error; return; }
  const ngPieces = j.pieces_detail.filter(p => p.state === 'BAD').map(p => p.piece);
  const incPieces = j.pieces_detail.filter(p => p.state === 'INCOMPLETE').map(p => p.piece);
  let h;
  if (ngPieces.length)
    h = `<div class="banner bng">❌ NG<small>ไม่ผ่าน: ชิ้นที่ ${ngPieces.join(', ')}</small></div>`;
  else if (incPieces.length)
    h = `<div class="banner binc">⏳ ไม่ครบ<small>หาดวงไม่เจอที่ชิ้น ${incPieces.join(', ')}</small></div>`;
  else
    h = `<div class="banner bok">✅ OK<small>ผ่านทุกชิ้น</small></div>`;
  h += j.mode === 'absolute'
    ? `<div class="hint">🎯 เทียบขาวอ้างอิง b*=${j.ref} (เพี้ยนฟ้า/เหลืองก็จับ)</div>`
    : `<div class="hint">⚠️ ยังไม่ calibrate — เทียบกันเองในกลุ่ม (แม่นน้อยกว่า)</div>`;
  h += '<table><tr><th>ชิ้นที่</th><th>ดวง</th><th>b*</th><th>K</th><th>ผล</th></tr>';
  for (const p of j.pieces_detail) {
    const pcls = p.state==='BAD' ? 'bad' : (p.state==='INCOMPLETE' ? 'miss' : 'ok');
    const plabel = p.state==='BAD' ? 'NG' : (p.state==='INCOMPLETE' ? 'ไม่ครบ' : 'OK');
    p.spots.forEach((s, k) => {
      let cls, row, verdict, bstr, kstr;
      if (s.state === 'MISSING') {
        cls = 'miss'; row = ' class="missrow"'; verdict = 'ไม่พบดวง';
        bstr = '–'; kstr = '–';
      } else {
        cls = s.state === 'BAD' ? 'bad' : 'ok'; row = s.state === 'BAD' ? ' class="badrow"' : '';
        verdict = s.state === 'BAD' ? 'NG ' + (s.dev < 0 ? '(ฟ้า)' : '(เหลือง)') : 'OK ผ่าน';
        bstr = s.b.toFixed(1); kstr = s.cct_k ?? '–';
      }
      h += `<tr${row}>` + (k === 0 ?
        `<td rowspan="${p.spots.length}" class="${pcls}">#${p.piece} ${plabel}</td>` : '');
      h += `<td>${spotName((s.i - 1) % j.per_piece + 1, j.per_piece)}</td>` +
           `<td>${bstr}</td><td>${kstr}</td>` +
           `<td class="${cls}">${verdict}</td></tr>`;
    });
  }
  el.innerHTML = h + '</table>';
}
// ---- นาฬิกานับถอยหลัง (ทำงานในเบราว์เซอร์ล้วนๆ) ----
let tEnd = null, tTick = null;
function fmt(s) { return String(Math.floor(s/60)).padStart(2,'0') + ':' + String(s%60).padStart(2,'0'); }
function beep() {
  const ctx = new (window.AudioContext || window.webkitAudioContext)();
  let t = ctx.currentTime;
  for (let i = 0; i < 6; i++) {
    const o = ctx.createOscillator(), g = ctx.createGain();
    o.connect(g); g.connect(ctx.destination);
    o.frequency.value = 880; g.gain.setValueAtTime(0.4, t);
    o.start(t); o.stop(t + 0.35); t += 0.55;
  }
}
function timerStart() {
  const m = +document.getElementById('tmin').value, s = +document.getElementById('tsec').value;
  const total = m*60 + s;
  if (total <= 0) return;
  document.getElementById('talarm').style.display = 'none';
  tEnd = Date.now() + total*1000;
  clearInterval(tTick);
  tTick = setInterval(() => {
    const left = Math.max(0, Math.round((tEnd - Date.now())/1000));
    const d = document.getElementById('tdisp');
    d.textContent = fmt(left);
    d.style.color = left <= 10 ? '#ff6b6b' : '';
    if (left <= 0) {
      clearInterval(tTick); tTick = null;
      document.getElementById('talarm').style.display = 'block';
      document.title = '⏰ ครบเวลา! — ตรวจสีไฟ LED';
      beep();
    }
  }, 250);
}
function timerStop() {
  clearInterval(tTick); tTick = null; tEnd = null;
  document.getElementById('tdisp').textContent = '--:--';
  document.getElementById('tdisp').style.color = '';
  document.getElementById('talarm').style.display = 'none';
  document.title = 'ตรวจสีไฟ LED';
}
</script>
</body></html>"""


@app.get("/", response_class=HTMLResponse)
def index():
    return PAGE


def main():
    global CAM_INDEX, EXPOSURE_INIT, THRESH, CAMERA_VIDPID
    p = argparse.ArgumentParser()
    p.add_argument("--cam", type=int, default=None,
                   help="camera index (ไม่ใส่ = หาจาก VID/PID ก่อน แล้วค่อยใช้ค่าที่จำไว้/0)")
    p.add_argument("--vidpid", default=None,
                   help="ผูกกล้องด้วย VID:PID เช่น 0x291a:0x3369 (Windows) แทน index คงที่; จำถาวร")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--exposure", type=int, default=DEFAULT_EXPOSURE,
                   help=f"ล็อก exposure ตั้งแต่เริ่ม (1-1000, default {DEFAULT_EXPOSURE}) "
                        f"ใส่ 0 = ปล่อย auto ของกล้อง")
    p.add_argument("--thresh", type=float, default=None,
                   help=f"เกณฑ์ NG b* (ไม่ใส่ = ใช้ค่าที่จำใน config.json หรือ {THRESH}); "
                        f"ใส่แล้วจำถาวร")
    args = p.parse_args()
    cfg = load_config()
    # VID/PID ที่จะใช้ผูกกล้อง: --vidpid > ค่าใน config > ค่าเริ่มต้น (C200)
    CAMERA_VIDPID = args.vidpid or cfg.get("cam_vidpid") or C200_VID_PID
    if args.vidpid:
        cfg["cam_vidpid"] = args.vidpid
        save_config(cfg)
    # เลือก index: --cam (ระบุตรงๆ) > หาจาก VID/PID (Windows) > ค่าที่จำไว้ > 0
    if args.cam is not None:
        CAM_INDEX = args.cam
    else:
        idx = find_index_by_vidpid(CAMERA_VIDPID)
        if idx is not None:
            CAM_INDEX = idx
            print(f"พบกล้อง VID/PID {CAMERA_VIDPID} ที่ index {idx}")
        else:
            CAM_INDEX = cfg.get("cam_index", 0)
    EXPOSURE_INIT = None if args.exposure == 0 else args.exposure
    if args.thresh is not None:        # ระบุมา → ใช้ + จำถาวร
        THRESH = args.thresh
        cfg["thresh"] = args.thresh
        save_config(cfg)
    else:                              # ไม่ระบุ → ใช้ค่าที่จำไว้ (หรือ default เดิม 4.0)
        THRESH = cfg.get("thresh", THRESH)
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
