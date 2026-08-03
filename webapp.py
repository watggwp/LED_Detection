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
import shutil
import subprocess
import sys
import threading
import time
import webbrowser
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import cv2
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel

from colorwatch import win_camera_names as _win_camera_names
from multiwatch import (DIRECTIONS, MAX_SPOTS, MultiWatch, resolve_direction)

app = FastAPI(title="LED colorwatch")
CAM_INDEX = 0
IDLE_RELEASE_SEC = 60.0   # ไม่มีคนดู stream + ไม่ได้ตรวจ เกินนี้ → ปล่อยกล้อง (ประหยัดแบต)

# Windows: บังคับใช้ DirectShow แทน MSMF (backend เริ่มต้นของ OpenCV บน Windows ซึ่งช้ามาก)
# วัดบนเครื่องทดสอบ (OpenCV 5.0.0, Python 3.8-32):
#   MSMF : VideoCapture()=4.04s + set 1280x720=7.13s + read แรก=0.34s  → ~11.5s ต่อการเปิด 1 ครั้ง
#   DSHOW: VideoCapture()=0.59s + set 1280x720=0.97s + read แรก=0.21s  → ~1.8s
# และสเกล exposure log2 ที่ _win_exposure() ใช้อยู่ก็เป็นสเกลของ DSHOW อยู่แล้ว (ดูคอมเมนต์ที่นั่น)
CAP_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
SCAN_MAX_INDEX = 6        # เพดาน index ตอนขอรายชื่อกล้องจาก OS ไม่ได้ (mac/Linux)
DEFAULT_EXPOSURE = 5      # auto-ล็อกแสงต่ำตอนเปิดกล้อง (ภาพเห็นแต่ดวงไฟบนพื้นดำ)
THRESH = 4.0              # เกณฑ์ NG (ห่างอ้างอิง/ห่างกันเอง เกินค่านี้) — ปรับด้วย --thresh
# ทิศทางที่ถือว่า NG: None = ตามพฤติกรรมเดิมของโปรแกรม (ดู effective_direction)
# ผู้ใช้เลือกเองในหน้าเว็บได้ แล้วค่าที่เลือกชนะและจำถาวร
DIRECTION = None
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


_WMI_CACHE = {}      # (vid, pid) -> (เวลา, [ชื่อ])
_WMI_TTL = 60.0      # ชื่อจาก WMI แทบไม่เปลี่ยน แต่เรียก powershell ทีนึงกิน 2-3s


def _win_names_for_vidpid(vid, pid):
    """ชื่อกล้องที่ VID/PID ตรง (Windows, ผ่าน WMI) — ใช้จับคู่กับชื่อจาก DirectShow
    cache ไว้ เพราะเรียก powershell ทีนึงกิน 2-3s และอยู่บนเส้นทาง 'เปิดกล้องไม่ได้'"""
    hit = _WMI_CACHE.get((vid, pid))
    if hit is not None and time.time() - hit[0] < _WMI_TTL:
        return hit[1]
    try:
        q = ("Get-CimInstance Win32_PnPEntity | Where-Object "
             f"{{ $_.PNPDeviceID -match 'VID_{vid}.*PID_{pid}' }} | "
             "Select-Object -ExpandProperty Name")
        r = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", q],
            capture_output=True, text=True, timeout=10)
        out = [ln.strip() for ln in r.stdout.splitlines() if ln.strip()]
    except Exception:
        out = []
    _WMI_CACHE[(vid, pid)] = (time.time(), out)
    return out


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


# ---------- Tailscale (เปิดให้เครื่องอื่นใน tailnet ดู) ----------

def tailscale_ip():
    """IPv4 ของเครื่องนี้ใน tailnet — None ถ้าไม่มี Tailscale / ยังไม่ได้ล็อกอิน

    ไม่ hardcode IP เพราะเปลี่ยนได้เวลา re-auth หรือย้ายเครื่อง"""
    candidates = [
        r"C:\Program Files\Tailscale\tailscale.exe",
        r"C:\Program Files (x86)\Tailscale\tailscale.exe",
        "/Applications/Tailscale.app/Contents/MacOS/Tailscale",
    ]
    exe = next((p for p in candidates if os.path.exists(p)), None) \
        or shutil.which("tailscale")
    if not exe:
        return None
    try:
        r = subprocess.run([exe, "ip", "-4"], capture_output=True, text=True,
                           timeout=10)
    except Exception:
        return None
    for line in r.stdout.splitlines():
        ip = line.strip()
        # Tailscale ใช้ช่วง CGNAT 100.64.0.0/10 — เช็คคร่าวๆ กันหยิบบรรทัดอื่นมา
        if ip.startswith("100.") and ip.count(".") == 3:
            return ip
    return None


# ---------- กล้อง ----------

def open_cap(index, should_stop=None):
    """เปิดกล้องแบบไม่ sys.exit (ต่างจาก colorwatch.open_camera) — คืน None ถ้าเปิดไม่ได้

    ต้องอ่านเฟรมได้จริงถึงจะนับว่าสำเร็จ: บาง index (เช่นกล้องที่ถูกโปรแกรมอื่นยึดอยู่)
    isOpened() คืน True แต่ read() ไม่เคยได้เฟรม — ของเดิมปล่อยผ่านแล้วไปวนติดใน
    capture_loop ทำให้จอดำค้างโดยไม่มีข้อความบอกสาเหตุ

    should_stop: callable ให้เลิกอุ่นเครื่องกลางคันได้ เวลามีคนสั่งปิดกล้องระหว่างนี้
    (VideoCapture() เองหยุดกลางคันไม่ได้ แต่ลูป warmup หยุดได้ ช่วยให้ stop_camera ไม่ต้องรอครบ)"""
    cap = None
    try:
        cap = cv2.VideoCapture(index, CAP_BACKEND)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        for _ in range(10):     # กล้องบางตัวต้องอุ่นเครื่อง 2-3 เฟรมก่อนส่งภาพจริง
            if should_stop is not None and should_stop():
                break
            ok, fr = cap.read()
            if ok and fr is not None:
                return cap
            time.sleep(0.1)
    except cv2.error as e:
        # DSHOW โยน C++ exception ได้ถ้าอุปกรณ์ถูกแย่ง/ถอดกลางคัน — อย่าให้ทั้ง thread ตาย
        print(f"[camera] เปิด index {index} ไม่สำเร็จ: {e}", file=sys.stderr)
    if cap is not None:
        try:
            cap.release()
        except Exception:
            pass
    return None


def probe_cap(index):
    """เปิดกล้องสั้นๆ ดูว่าใช้ได้จริงไหม + คืนเฟรมไว้ทำ thumbnail — (ok, frame)
    ไม่ตั้ง 1280x720 เพราะ thumbnail กว้างแค่ 220px และการ set กินเวลาเพิ่มโดยเปล่าประโยชน์"""
    cap = None
    try:
        cap = cv2.VideoCapture(index, CAP_BACKEND)
        if not cap.isOpened():
            return False, None
        for _ in range(3):
            ok, fr = cap.read()
            if ok and fr is not None:
                return True, fr
        return False, None
    except Exception as e:
        print(f"[camera] probe index {index} ล้มเหลว: {e}", file=sys.stderr)
        return False, None
    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass


def _apply_flip(frame, fx, fy):
    """กลับภาพตามที่ตั้งไว้ — ใช้ทั้งใน capture loop และตอนทำ thumbnail เลือกกล้อง
    เพื่อให้ทุกรูปในหน้าเลือกกล้องหันทางเดียวกับภาพสด (ไม่งั้นตัวที่กำลังใช้จะกลับด้านอยู่ตัวเดียว)"""
    if fx and fy:
        return cv2.flip(frame, -1)
    if fx:
        return cv2.flip(frame, 1)
    if fy:
        return cv2.flip(frame, 0)
    return frame


def _thumb(frame, width=220):
    h, w = frame.shape[:2]
    small = cv2.resize(frame, (width, max(1, int(h * width / w))))
    ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return "data:image/jpeg;base64," + base64.b64encode(buf).decode() if ok else None


class Session:
    """สถานะกล้อง+การตรวจ — กล้องเปิดเมื่อมีคนดู/ตรวจ, ปล่อยเองตอน idle (D11)"""

    def __init__(self):
        cfg = load_config()
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
        self.flip_x = cfg.get("flip_x", False)  # กลับภาพแนวนอน (ซ้าย-ขวา)
        self.flip_y = cfg.get("flip_y", False)  # กลับภาพแนวตั้ง (บน-ล่าง)
        # คิวคำสั่งตั้ง exposure ฝั่ง OpenCV (ต้องทำใน capture thread ที่ถือ cap)
        self.exp_pending = False
        self.exp_value = None
        self.exp_event = threading.Event()
        self.exp_result = (False, "")

    def detecting(self):
        return self.engine is not None


SES = Session()
EXPOSURE_INIT = DEFAULT_EXPOSURE   # ตั้งจาก --exposure: ล็อกแสงทันทีที่กล้องเปิด

# กันเปิด/ปิดกล้องชนกัน — ทุกจุดที่ "เปิด/ปิด/ไล่สแกน" กล้องต้องถือล็อกนี้
# ไม่งั้นสองเธรดแตะ DirectShow device ตัวเดียวกันพร้อมกันได้ เช่น /api/cameras กำลัง probe
# index 0 อยู่ แล้ว /stream (ซึ่งเรียก ensure_camera ทุกครั้ง) เปิด index 0 ขึ้นมาซ้อน
# → OpenCV โยน "Unknown C++ exception" ทิ้ง capture thread ตาย จอดำค้าง
# ใช้ RLock เพราะ api_cameras ถือล็อกอยู่แล้วยังต้องเรียก stop_camera()/ensure_camera() ต่อ
CAM_CTL = threading.RLock()
JOIN_TIMEOUT = 8.0     # เผื่อ VideoCapture() ที่ค้างอยู่ (interrupt กลางคันไม่ได้) ให้จบเอง


def capture_loop(ses):
    """thread ถือกล้อง — สลับพรีวิว/ตรวจตาม ses.engine, ปล่อยกล้องเองเมื่อ idle"""
    global CAM_INDEX
    stopping = lambda: ses.stop_flag        # noqa: E731 — สั้นกว่าและใช้ที่เดียว
    cap = open_cap(CAM_INDEX, stopping)
    if cap is None and not ses.stop_flag:
        # index ที่จำไว้เปิดไม่ได้ (อาจสลับพอร์ต) — ลองหาใหม่จาก VID/PID (Windows)
        alt = find_index_by_vidpid(CAMERA_VIDPID)
        if alt is not None and alt != CAM_INDEX:
            cap = open_cap(alt, stopping)
            if cap is not None:
                CAM_INDEX = alt
    # โดนสั่งหยุดระหว่างเปิด — ปล่อยทิ้งเงียบๆ อย่าไปเขียน cam_error ทับสถานะของกล้องตัวใหม่
    if cap is not None and ses.stop_flag:
        try:
            cap.release()
        except Exception:
            pass
        return
    if cap is None and ses.stop_flag:
        return
    if cap is None:
        names = _win_camera_names()
        who = (f" ({names[CAM_INDEX]})" if 0 <= CAM_INDEX < len(names) else "")
        with ses.lock:
            ses.cam_error = (f"เปิดกล้อง index {CAM_INDEX}{who} ไม่ได้ — "
                             f"อาจถูกโปรแกรมอื่นใช้อยู่ (Teams/Zoom/OBS) หรือถอดสายไปแล้ว "
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
            with ses.lock:
                fx, fy = ses.flip_x, ses.flip_y
            frame = _apply_flip(frame, fx, fy)

            now = time.time()
            with ses.lock:
                engine = ses.engine
                ses.frame = frame.copy()  # เก็บเฟรมดิบไว้ให้ calibrate (เป็นเฟรมที่กลับด้านแล้ว)
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
    except cv2.error as e:
        # กล้องถูกถอด/ถูกแย่งกลางคัน → OpenCV โยน C++ exception ออกมา ถ้าปล่อยหลุด
        # thread จะตายเงียบๆ แล้วหน้าเว็บค้างที่ภาพสุดท้ายโดยไม่มีใครบอกสาเหตุ
        with ses.lock:
            ses.cam_error = (f"กล้องหลุดกลางคัน ({e}) — เช็คสาย USB "
                             f"แล้วกด 'เลือกกล้อง' ใหม่")
            ses.jpeg = None
    finally:
        try:
            cap.release()
        except Exception:
            pass
        with ses.lock:
            if ses.cap == cap:
                ses.cap = None


def ensure_camera():
    with CAM_CTL:
        with SES.lock:
            t = SES.thread
            # thread ที่ยังไม่ตั้ง stop_flag = ตัวที่ใช้งานอยู่จริง → ไม่ต้องทำอะไร
            if t is not None and t.is_alive() and not SES.stop_flag:
                return
        # ถ้าตัวเก่ากำลังปิดตัวอยู่ ต้องรอให้มันปล่อยกล้องก่อน ไม่งั้นตัวใหม่จะเปิดซ้ำ index เดิมไม่ได้
        if t is not None and t.is_alive():
            t.join(timeout=JOIN_TIMEOUT)
        with SES.lock:
            SES.stop_flag = False
            SES.last_stream = time.time()
            SES.thread = threading.Thread(target=capture_loop, args=(SES,),
                                          daemon=True)
            SES.thread.start()


def wait_first_frame(timeout=8.0):
    """รอจนกล้องส่งเฟรมแรก (หรือแจ้ง error) — คืน True ถ้ามีภาพแล้ว
    ใช้ตอนสลับกล้อง เพื่อให้ตอบกลับหน้าเว็บหลังภาพพร้อมจริง ไม่ใช่ปล่อยให้ <img> ค้างขาว"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        with SES.lock:
            if SES.jpeg is not None:
                return True
            if SES.cam_error is not None:
                return False
            alive = SES.thread is not None and SES.thread.is_alive()
        if not alive:
            return False
        time.sleep(0.05)
    return False


def stop_camera():
    """หยุด capture thread + ปล่อยกล้อง (รอ join จนกว่าจะดับจริง) — ใช้ก่อนสแกน/เปลี่ยนกล้อง"""
    with CAM_CTL:
        with SES.lock:
            t = SES.thread
            cap = SES.cap
            SES.stop_flag = True

        if t is None or not t.is_alive():
            with SES.lock:
                SES.thread = None
            return

        # ปล่อย cap ทันทีเพื่อปลดล็อก cap.read() ที่ค้างอยู่ใน OpenCV thread
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

        t.join(timeout=JOIN_TIMEOUT)
        with SES.lock:
            if not t.is_alive():
                SES.thread = None


# ---------- โมเดล request ----------

class StartReq(BaseModel):
    pieces: int
    per_piece: int = 2


class ExposureReq(BaseModel):
    value: Optional[int] = None  # None = auto


class CamReq(BaseModel):
    index: int


class FlipReq(BaseModel):
    flip_x: Optional[bool] = None
    flip_y: Optional[bool] = None


class ThreshReq(BaseModel):
    value: float


class DirectionReq(BaseModel):
    value: str


class CalibrateReq(BaseModel):
    # ติ๊กออก = ข้ามด่านตรวจว่าชิ้นที่วางสีใกล้กันไหม แล้ว calibrate เลย
    check_spread: bool = True


# ---------- endpoints ----------

@app.post("/api/flip")
def api_flip(req: FlipReq):
    with SES.lock:
        if req.flip_x is not None:
            SES.flip_x = req.flip_x
        if req.flip_y is not None:
            SES.flip_y = req.flip_y
        cfg = load_config()
        cfg["flip_x"] = SES.flip_x
        cfg["flip_y"] = SES.flip_y
        save_config(cfg)
        fx, fy = SES.flip_x, SES.flip_y
    return {"ok": True, "flip_x": fx, "flip_y": fy}

def effective_direction(reference=None):
    """ทิศทางที่จะใช้ตัดสิน — ไว้โชว์ในหน้าเว็บตอนยังไม่ได้เริ่มตรวจ (ยังไม่มี engine)"""
    return resolve_direction(DIRECTION, reference)


DIR_LABEL = {"both": "ฟ้าและเหลือง", "blue": "ฟ้าเท่านั้น", "yellow": "เหลืองเท่านั้น"}


@app.post("/api/direction")
def api_direction(req: DirectionReq):
    """เลือกว่าจะจับการเบี่ยงสีทิศไหนเป็น NG — มีผลทันทีกับรอบที่กำลังตรวจอยู่"""
    global DIRECTION
    if req.value not in DIRECTIONS:
        return JSONResponse(
            {"error": f"ทิศทางต้องเป็นหนึ่งใน {', '.join(DIRECTIONS)}"},
            status_code=400)
    DIRECTION = req.value
    cfg = load_config()
    cfg["direction"] = DIRECTION
    save_config(cfg)
    with SES.lock:
        if SES.engine is not None:
            SES.engine.direction = DIRECTION
    return {"ok": True, "direction": DIRECTION, "label": DIR_LABEL[DIRECTION]}


THRESH_MIN, THRESH_MAX = 0.5, 20.0


@app.post("/api/thresh")
def api_thresh(req: ThreshReq):
    """ปรับเกณฑ์ NG ระหว่างใช้งาน — มีผลทันทีกับรอบที่กำลังตรวจอยู่ ไม่ต้องปิดเปิดโปรแกรม
    (เดิมตั้งได้จาก --thresh ตอนสตาร์ทอย่างเดียว ทำให้จูนหน้างานลำบาก)"""
    global THRESH
    v = req.value
    if not (THRESH_MIN <= v <= THRESH_MAX):
        return JSONResponse(
            {"error": f"เกณฑ์ต้องอยู่ระหว่าง {THRESH_MIN} ถึง {THRESH_MAX}"},
            status_code=400)
    THRESH = float(v)
    cfg = load_config()
    cfg["thresh"] = THRESH
    save_config(cfg)
    with SES.lock:
        if SES.engine is not None:
            # engine อ่าน self.thresh ใหม่ทุกเฟรม → เปลี่ยนสดได้ ไม่ต้องสร้าง engine ใหม่
            # (สร้างใหม่จะทำให้สล็อตที่ยึดไว้กับ hold timer รีเซ็ตหมดโดยไม่จำเป็น)
            SES.engine.thresh = THRESH
    return {"ok": True, "thresh": THRESH}


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
    """ส่ง thumbnail ของทุกกล้องให้ผู้ใช้คลิกเลือกตัวที่เห็นไฟ LED

    เร็วกว่าของเดิมสามทาง:
      1. ถาม DirectShow ว่ามีกล้องกี่ตัว (~0.3s) แล้วสแกนเท่าที่มีจริง — ไม่ไล่ index 0-5 เผื่อ
      2. กล้องที่กำลังเปิดอยู่ ยืมเฟรมสดมาทำ thumbnail เลย ไม่ต้องปิด/เปิดใหม่
         → ภาพสดไม่ดับระหว่างสแกน และไม่เสียเวลาเปิดกล้องคืนอีกรอบ
      3. index ที่เหลือ probe พร้อมกันแบบขนาน — เวลารวม = ตัวที่ช้าที่สุด ไม่ใช่ผลรวม
    """
    names = _win_camera_names()
    # ถือ CAM_CTL ตลอดการสแกน — ระหว่างนี้ห้ามใครเปิด/ปิดกล้อง (/stream เรียก ensure_camera
    # ทุกครั้งที่โหลด ถ้าปล่อยให้แทรกได้จะกลายเป็นสองเธรดแย่ง device เดียวกันแล้ว OpenCV พัง)
    with CAM_CTL:
        with SES.lock:
            alive = SES.thread is not None and SES.thread.is_alive()
            cur_frame = (SES.frame.copy()
                         if (alive and SES.frame is not None) else None)
            fx, fy = SES.flip_x, SES.flip_y
        cur_index = CAM_INDEX

        scan = set(range(len(names) if names else SCAN_MAX_INDEX))
        scan.add(cur_index)          # เผื่อ index ที่จำไว้อยู่นอกช่วงที่ OS รายงาน
        if cur_frame is not None:
            scan.discard(cur_index)  # ตัวนี้มีภาพสดอยู่แล้ว ไม่ต้องแตะ
        else:
            stop_camera()        # ยังไม่มีภาพ → ปล่อยกล้องก่อน ไม่งั้น probe index เดิมไม่ได้

        found = {}
        targets = sorted(scan)
        if targets:
            with ThreadPoolExecutor(max_workers=min(6, len(targets))) as ex:
                for i, res in zip(targets, ex.map(probe_cap, targets)):
                    found[i] = res
        if cur_frame is not None:
            found[cur_index] = (True, cur_frame)
        ensure_camera()   # ถ้ากล้องเดิมยังเปิดอยู่ ฟังก์ชันนี้ไม่ทำอะไร

    cams = []
    for i in sorted(found):
        ok, fr = found[i]
        if ok and fr is not None:
            # เฟรมที่ยืมจากภาพสด กลับด้านมาแล้ว ที่ probe มาใหม่ต้องกลับให้ตรงกัน
            if i != cur_index or cur_frame is None:
                fr = _apply_flip(fr, fx, fy)
            cams.append({"index": i, "thumb": _thumb(fr),
                         "name": names[i] if i < len(names) else "",
                         "current": i == cur_index})
    # กล้องที่ OS เห็นแต่เปิดไม่ได้ (โปรแกรมอื่นยึดอยู่ / ถอดสายค้าง) — บอกผู้ใช้ไปตรงๆ
    busy = [{"index": i, "name": names[i]} for i in sorted(found)
            if not found[i][0] and i < len(names)]
    return {"cameras": cams, "current": cur_index, "busy": busy}


@app.post("/api/select_camera")
def api_select_camera(req: CamReq):
    global CAM_INDEX
    # ปิด-สลับ-เปิด ต้องเป็นก้อนเดียว ไม่งั้น /stream ที่ยิงเข้ามาพอดีจะเปิดกล้อง "ตัวเก่า"
    # คืนมาคั่นกลาง แล้วเราไปเปิดตัวใหม่ทับ = สองเธรดถือกล้องพร้อมกัน
    with CAM_CTL:
        stop_camera()
        CAM_INDEX = req.index
        cfg = load_config()
        cfg["cam_index"] = req.index
        save_config(cfg)
        with SES.lock:
            SES.engine = None   # กล้องเปลี่ยน — เริ่มตรวจใหม่ค่อยยึดสล็อตใหม่
            SES.status = None
            SES.jpeg = None
            SES.frame = None
            SES.cam_error = None   # ล้าง error ตัวเก่า ไม่งั้น wait_first_frame เชื่อค่าเก่า
        ensure_camera()
    # รอให้ภาพแรกมาก่อนค่อยตอบ หน้าเว็บจะได้ไม่รีโหลด /stream ตอนกล้องยังไม่พร้อม
    ok = wait_first_frame()
    with SES.lock:
        err = SES.cam_error
    return {"ok": ok, "index": req.index, "error": err}


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
        SES.engine = MultiWatch(total, thresh=THRESH, reference=SES.reference,
                                direction=DIRECTION)
        direction = SES.engine.resolved_direction()
    return {"ok": True, "total_spots": total,
            "mode": "absolute" if SES.reference is not None else "relative",
            "direction": direction}


@app.post("/api/calibrate")
def api_calibrate(req: Optional[CalibrateReq] = None):
    """วางชิ้นดีครบตามจำนวน → เรียกตอนกำลังตรวจ — ตั้งขาวอ้างอิงและจำถาวร

    body ว่างได้ (ของเดิมเรียกแบบไม่ส่ง body) → ตรวจความห่างตามปกติ"""
    check_spread = True if req is None else req.check_spread
    with SES.lock:
        engine = SES.engine
        frame = SES.frame.copy() if SES.frame is not None else None
    if engine is None:
        return JSONResponse({"error": "กดเริ่มตรวจก่อน แล้วค่อย calibrate "
                                      "(ต้องรู้จำนวนจุดก่อน)"}, status_code=400)
    if frame is None:
        return JSONResponse({"error": "ยังไม่มีภาพจากกล้อง"}, status_code=400)
    ref, msg = engine.calibrate(frame, check_spread=check_spread)
    if ref is None:
        return JSONResponse({"error": msg}, status_code=400)
    with SES.lock:
        SES.reference = ref
    save_reference(ref)
    return {"ok": True, "reference": round(ref, 2), "msg": msg,
            "checked_spread": check_spread}


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
        fx, fy = SES.flip_x, SES.flip_y
        # ทิศทางที่ engine ใช้จริงอยู่ (ถ้ากำลังตรวจ) ไม่ใช่ค่าที่ "จะใช้รอบหน้า"
        eng = SES.engine
        direction = eng.resolved_direction() if eng is not None \
            else effective_direction(SES.reference)
    common = {"thresh": THRESH, "flip_x": fx, "flip_y": fy,
              "direction": direction, "direction_label": DIR_LABEL[direction],
              "direction_pinned": DIRECTION is not None}
    if not detecting:
        return dict(common, running=False, preview=True, cam_error=cam_error)
    out = dict(common, running=True, pieces=SES.pieces, per_piece=SES.per_piece,
               max_spots=MAX_SPOTS)
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
  .near { color: #ffb454; font-weight: 700; }   /* เฉียดเกณฑ์ — เตือนตอนจูน thresh */
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
  .camcard span { font-size: 13px; color: #c4cad4; display: block; max-width: 220px; }
  .camcard.busy { cursor: default; align-self: center; padding: 18px 12px; }
</style></head>
<body>
<header>🔍 ตรวจสีไฟ LED — ฝั่งไหนเพี้ยน (ฟ้า)</header>
<main>
  <div class="panel" style="flex:1 1 640px">
    <div id="controls">
      <div style="margin-bottom:10px">
        <button onclick="loadCams()" style="background:#3d5a80;color:#fff">📷 เลือกกล้อง</button>
        <button id="btnDoneCam" onclick="closeCamSelector()" style="background:#2e7d32;color:#fff;display:none">✅ เสร็จสิ้นการเลือกกล้อง</button>
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
        <span style="font-weight:600">🔄 กลับภาพกล้อง:</span>
        <button id="btnFlipX" onclick="toggleFlip('x')" style="background:#3a4250;color:#fff">↔ กลับซ้าย-ขวา (แกน X)</button>
        <button id="btnFlipY" onclick="toggleFlip('y')" style="background:#3a4250;color:#fff">↕ กลับบน-ล่าง (แกน Y)</button>
      </div>
      <div style="margin-top:10px">
        <button onclick="calibrate()" style="background:#7b5dbe;color:#fff">🎯 Calibrate ขาว (วางชิ้นดีให้ครบก่อนกด)</button>
        <button onclick="clearCalib()">ล้างอ้างอิง</button>
        <label style="margin-left:10px;cursor:pointer">
          <input type="checkbox" id="calspread" checked style="vertical-align:middle">
          ตรวจความห่างก่อน calibrate
        </label>
        <span id="calinfo" class="hint"></span>
      </div>
      <div class="hint">ติ๊กไว้ = ถ้าชิ้นที่วางสีต่างกันเกินเกณฑ์ จะไม่ยอม calibrate (กันเผลอมีชิ้นเสียปน)
        — ติ๊กออกเมื่อรู้อยู่แล้วว่าดวงแต่ละตำแหน่งสีต่างกันเองตามธรรมชาติ
        <span style="color:#ffb454">ระวัง: ติ๊กออกแล้วถ้ามีชิ้นเสียปนอยู่ ค่าอ้างอิงจะเพี้ยนโดยไม่มีใครเตือน</span></div>
      <div style="margin-top:10px">
        <span style="font-weight:600">🎯 จับการเพี้ยนทิศไหน:</span>
        <select id="dirsel" onchange="setDirection(this.value)"
                style="font-size:16px;padding:6px;border-radius:6px;border:1px solid #3a4250;background:#11141a;color:#e8eaed">
          <option value="both">ทั้งฟ้าและเหลือง</option>
          <option value="blue">ฟ้าเท่านั้น</option>
          <option value="yellow">เหลืองเท่านั้น</option>
        </select>
        <span class="hint" id="dirhint"></span>
      </div>
      <div style="margin-top:10px">
        <span style="font-weight:600">🎚 เกณฑ์ตัดสิน NG:</span>
        <input type="range" id="thslider" min="0.5" max="20" step="0.5"
               oninput="threshPreview(this.value)" onchange="setThresh(this.value)"
               style="vertical-align:middle;width:200px">
        <input type="number" id="thnum" min="0.5" max="20" step="0.5"
               onchange="setThresh(this.value)" style="width:70px">
        <span class="hint" id="thhint">ยิ่งน้อยยิ่งเข้มงวด — ปรับแล้วมีผลทันที</span>
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
  document.getElementById('msg').textContent = 'กำลังสแกนกล้อง...';
  const j = await (await fetch('/api/cameras')).json();
  const box = document.getElementById('cams');
  box.innerHTML = '';
  if (!j.cameras.length && !(j.busy || []).length) {
    box.innerHTML = '<span class="hint">ไม่พบกล้อง — เช็คสาย USB</span>';
    document.getElementById('btnDoneCam').style.display = 'none';
    return;
  }
  for (const c of j.cameras) {
    const d = document.createElement('div');
    d.id = 'camcard-' + c.index;
    d.className = 'camcard' + (c.current ? ' cur' : '');
    d.dataset.name = c.name || ('กล้อง #' + c.index);
    d.innerHTML = `<img src="${c.thumb}"><span>${d.dataset.name}${c.current?' (กำลังดูภาพสด)':''}</span>`;
    d.onclick = () => selectCam(c.index);
    box.appendChild(d);
  }
  // กล้องที่ Windows เห็นแต่เปิดไม่ได้ — บอกไปตรงๆ ดีกว่าหายไปเฉยๆ แล้วผู้ใช้งง
  for (const b of (j.busy || [])) {
    const d = document.createElement('div');
    d.className = 'camcard busy';
    d.style.opacity = '.55';
    d.innerHTML = `<span>⚠️ ${b.name}<br><small>เปิดไม่ได้ — อาจถูก Teams/Zoom/OBS ใช้อยู่ หรือสายหลุด</small></span>`;
    box.appendChild(d);
  }
  document.getElementById('btnDoneCam').style.display = 'inline-block';
  document.getElementById('msg').textContent = j.cameras.length
    ? 'คลิกเลือกกล้องเพื่อสลับดูภาพสดที่จอหลักได้ทันที — เมื่อพอใจแล้วกด "เสร็จสิ้น"'
    : 'พบกล้องแต่เปิดไม่ได้สักตัว — ปิดโปรแกรมที่ใช้กล้องอยู่ (Teams/Zoom/OBS) แล้วกด "เลือกกล้อง" ใหม่';
}
async function selectCam(i) {
  const card0 = document.getElementById('camcard-' + i);
  const nm = (card0 && card0.dataset.name) || ('กล้อง #' + i);
  document.getElementById('msg').textContent = 'กำลังสลับเป็น ' + nm + '...';
  const r = await (await fetch('/api/select_camera', {method:'POST',
    headers:{'Content-Type':'application/json'}, body: JSON.stringify({index: i})})).json();
  if (!r.ok) {
    document.getElementById('msg').textContent =
      '⚠️ สลับเป็น ' + nm + ' ไม่สำเร็จ — ' + (r.error || 'กล้องไม่ส่งภาพ');
    return;
  }

  const box = document.getElementById('cams');
  const cards = box.getElementsByClassName('camcard');
  for (let card of cards) {
    card.classList.remove('cur');
    const span = card.querySelector('span');
    if (span) span.textContent = span.textContent.replace(' (กำลังดูภาพสด)', '');
  }
  const selCard = document.getElementById('camcard-' + i);
  if (selCard) {
    selCard.classList.add('cur');
    const span = selCard.querySelector('span');
    if (span) span.textContent = `กล้อง #${i} (กำลังดูภาพสด)`;
  }
  document.getElementById('msg').textContent = '✅ สลับเป็นกล้อง #' + i + ' แล้ว (แสดงภาพสดที่จอหลักทันที)';

  // รีโหลดภาพสดหน้าหลัก (cache-bust)
  document.getElementById('video').src = '/stream?ts=' + Date.now();
}
function closeCamSelector() {
  document.getElementById('cams').innerHTML = '';
  document.getElementById('btnDoneCam').style.display = 'none';
  document.getElementById('msg').textContent = 'เลือกกล้องเรียบร้อยแล้ว';
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
  const chk = document.getElementById('calspread').checked;
  const r = await fetch('/api/calibrate', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({check_spread: chk})});
  const j = await r.json();
  // เขียนลง calinfo (ช่องข้างปุ่ม calibrate) ไม่ใช่ msg ที่ปุ่มอื่นใช้ร่วมกัน
  // → ย้อมสีเตือนได้โดยไม่ไปติดค้างกับข้อความของปุ่มอื่น
  const el = document.getElementById('calinfo');
  el.textContent = j.ok ? '✓ ' + j.msg : '✗ ' + j.error;
  el.style.color = !j.ok ? '#ff6b6b'
                 : (j.checked_spread ? '#6fdc8c' : '#ffb454');
  document.getElementById('msg').textContent = '';
}
async function clearCalib() {
  await fetch('/api/calibrate/clear', {method:'POST'});
  const el = document.getElementById('calinfo');
  el.textContent = 'ล้างอ้างอิงแล้ว — กลับโหมดเทียบกันเอง';
  el.style.color = '';
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

let flipX = null, flipY = null;
async function toggleFlip(axis) {
  let newX = flipX, newY = flipY;
  if (axis === 'x') newX = !flipX;
  if (axis === 'y') newY = !flipY;
  const r = await fetch('/api/flip', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({flip_x: newX, flip_y: newY})
  });
  const j = await r.json();
  flipX = j.flip_x; flipY = j.flip_y;
  updateFlipUI();
  document.getElementById('msg').textContent = `กลับภาพ: แกน X (ซ้าย-ขวา) = ${flipX ? 'เปิด' : 'ปิด'}, แกน Y (บน-ล่าง) = ${flipY ? 'เปิด' : 'ปิด'}`;
}
function updateFlipUI() {
  const bx = document.getElementById('btnFlipX');
  const by = document.getElementById('btnFlipY');
  if (bx) {
    bx.style.background = flipX ? '#2e7d32' : '#3a4250';
    bx.textContent = (flipX ? '✓ ' : '') + '↔ กลับซ้าย-ขวา (แกน X)';
  }
  if (by) {
    by.style.background = flipY ? '#2e7d32' : '#3a4250';
    by.textContent = (flipY ? '✓ ' : '') + '↕ กลับบน-ล่าง (แกน Y)';
  }
}

function spotName(idx, per) {
  if (per === 2) return idx === 1 ? 'ซ้าย' : 'ขวา';
  return 'ดวง ' + idx;
}

// ---- ทิศทางที่ถือว่า NG ----
let direction = null;
let dirBusy = false;
async function setDirection(v) {
  if (dirBusy) return;
  dirBusy = true;
  const hint = document.getElementById('dirhint');
  try {
    const r = await (await fetch('/api/direction', {method:'POST',
      headers:{'Content-Type':'application/json'}, body: JSON.stringify({value: v})})).json();
    if (r.error) { hint.textContent = '⚠️ ' + r.error; return; }
    direction = r.direction;
    hint.textContent = `✅ จับ${r.label} (จำถาวร)`;
  } catch (e) {
    hint.textContent = '⚠️ ตั้งทิศทางไม่สำเร็จ';
  } finally {
    dirBusy = false;
  }
}

// ---- เกณฑ์ตัดสิน NG (ปรับสดได้ ไม่ต้องปิดเปิดโปรแกรม) ----
let thresh = null;        // ค่าที่เซิร์ฟเวอร์ยืนยันแล้ว
let threshBusy = false;   // กันเลื่อน slider รัวๆ แล้วยิงซ้อนกัน
function threshPreview(v) {            // ระหว่างลาก: อัปเดตตัวเลขให้เห็นทันที
  document.getElementById('thnum').value = v;
}
async function setThresh(v) {
  v = parseFloat(v);
  if (!(v >= 0.5 && v <= 20)) return;
  if (threshBusy) return;
  threshBusy = true;
  const hint = document.getElementById('thhint');
  try {
    const r = await (await fetch('/api/thresh', {method:'POST',
      headers:{'Content-Type':'application/json'}, body: JSON.stringify({value: v})})).json();
    if (r.error) { hint.textContent = '⚠️ ' + r.error; return; }
    thresh = r.thresh;
    applyThreshUI();
    hint.textContent = `✅ ใช้เกณฑ์ ${r.thresh} แล้ว (จำถาวร)`;
  } catch (e) {
    hint.textContent = '⚠️ ตั้งเกณฑ์ไม่สำเร็จ';
  } finally {
    threshBusy = false;
  }
}
function applyThreshUI() {
  document.getElementById('thslider').value = thresh;
  document.getElementById('thnum').value = thresh;
}
async function poll() {
  const j = await (await fetch('/api/status')).json();
  if (j.flip_x !== undefined && j.flip_y !== undefined) {
    if (flipX !== j.flip_x || flipY !== j.flip_y) {
      flipX = j.flip_x; flipY = j.flip_y;
      updateFlipUI();
    }
  }
  // ดึงค่าเกณฑ์จริงจากเซิร์ฟเวอร์ครั้งแรก (และถ้ามีคนแก้จากอีกแท็บ) — แต่อย่าไปทับ
  // ตอนผู้ใช้กำลังลาก slider อยู่
  if (j.thresh !== undefined && !threshBusy && thresh !== j.thresh) {
    thresh = j.thresh;
    applyThreshUI();
  }
  if (j.direction !== undefined && !dirBusy && direction !== j.direction) {
    direction = j.direction;
    document.getElementById('dirsel').value = direction;
    // ยังไม่เคยเลือกเอง = ค่านี้มาจากพฤติกรรมเดิม บอกให้รู้ว่าทำไมเป็นค่านี้
    document.getElementById('dirhint').textContent = j.direction_pinned
      ? `จับ${j.direction_label}`
      : `จับ${j.direction_label} (ค่าเริ่มต้นตามโหมด — เลือกเองได้)`;
  }
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
  const rule = j.direction === 'blue' ? `ต่ำกว่า −${j.thresh}`
             : j.direction === 'yellow' ? `เกิน +${j.thresh}`
             : `เกิน ±${j.thresh}`;
  h += j.mode === 'absolute'
    ? `<div class="hint">🎯 เทียบขาวอ้างอิง b*=${j.ref} — NG เมื่อ "ห่าง" ${rule} (จับ${j.direction_label})</div>`
    : `<div class="hint">⚠️ ยังไม่ calibrate — เทียบกันเองในกลุ่ม NG เมื่อ "ห่าง" ${rule} (แม่นน้อยกว่า)</div>`;
  // relative + สองทาง ตอน 2 ดวง: dev ของสองดวงเป็นภาพสะท้อนกัน → เด้ง NG พร้อมกันเสมอ
  if (j.mode !== 'absolute' && j.direction === 'both' && j.pieces * j.per_piece === 2)
    h += `<div class="hint" style="color:#ffb454">⚠️ โหมดเทียบกันเอง + จับสองทาง ที่ 2 ดวง
          จะขึ้น NG พร้อมกันทั้งคู่เสมอ แยกไม่ออกว่าดวงไหนผิด — เลือก "ฟ้าเท่านั้น"
          หรือ calibrate ก่อน</div>`;
  h += '<table><tr><th>ชิ้นที่</th><th>ดวง</th><th>b*</th><th>ห่าง</th><th>K</th><th>ผล</th></tr>';
  for (const p of j.pieces_detail) {
    const pcls = p.state==='BAD' ? 'bad' : (p.state==='INCOMPLETE' ? 'miss' : 'ok');
    const plabel = p.state==='BAD' ? 'NG' : (p.state==='INCOMPLETE' ? 'ไม่ครบ' : 'OK');
    p.spots.forEach((s, k) => {
      let cls, row, verdict, bstr, kstr, dstr, dcls;
      if (s.state === 'MISSING') {
        cls = 'miss'; row = ' class="missrow"'; verdict = 'ไม่พบดวง';
        bstr = '–'; kstr = '–'; dstr = '–'; dcls = '';
      } else {
        cls = s.state === 'BAD' ? 'bad' : 'ok'; row = s.state === 'BAD' ? ' class="badrow"' : '';
        verdict = s.state === 'BAD' ? 'NG ' + (s.dev < 0 ? '(ฟ้า)' : '(เหลือง)') : 'OK ผ่าน';
        bstr = s.b.toFixed(1); kstr = s.cct_k ?? '–';
        // "ห่าง" = ระยะจากเกณฑ์ที่ใช้ตัดสิน — ตัวเลขที่ต้องดูตอนจูน thresh
        // ใกล้ ±thresh เมื่อไหร่ = เฉียดตกเกณฑ์ ทำเป็นสีส้มเตือน
        dstr = (s.dev >= 0 ? '+' : '') + s.dev.toFixed(1);
        dcls = Math.abs(s.dev) >= j.thresh ? 'bad'
             : (Math.abs(s.dev) >= j.thresh * 0.75 ? 'near' : '');
      }
      h += `<tr${row}>` + (k === 0 ?
        `<td rowspan="${p.spots.length}" class="${pcls}">#${p.piece} ${plabel}</td>` : '');
      h += `<td>${spotName((s.i - 1) % j.per_piece + 1, j.per_piece)}</td>` +
           `<td>${bstr}</td><td class="${dcls}">${dstr}</td><td>${kstr}</td>` +
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
    global CAM_INDEX, EXPOSURE_INIT, THRESH, CAMERA_VIDPID, DIRECTION
    p = argparse.ArgumentParser()
    p.add_argument("--cam", type=int, default=None,
                   help="camera index (ไม่ใส่ = หาจาก VID/PID ก่อน แล้วค่อยใช้ค่าที่จำไว้/0)")
    p.add_argument("--vidpid", default=None,
                   help="ผูกกล้องด้วย VID:PID เช่น 0x291a:0x3369 (Windows) แทน index คงที่; จำถาวร")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--tailscale", action="store_true",
                   help="ผูกกับ Tailscale IP ของเครื่องนี้ (หาให้อัตโนมัติ) — เครื่องอื่นที่"
                        "ล็อกอิน Tailscale บัญชีเดียวกันเปิดดูได้ แต่วง LAN/Wi-Fi มองไม่เห็น")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--exposure", type=int, default=DEFAULT_EXPOSURE,
                   help=f"ล็อก exposure ตั้งแต่เริ่ม (1-1000, default {DEFAULT_EXPOSURE}) "
                        f"ใส่ 0 = ปล่อย auto ของกล้อง")
    p.add_argument("--thresh", type=float, default=None,
                   help=f"เกณฑ์ NG b* (ไม่ใส่ = ใช้ค่าที่จำใน config.json หรือ {THRESH}); "
                        f"ใส่แล้วจำถาวร")
    p.add_argument("--direction", choices=DIRECTIONS, default=None,
                   help="ทิศที่ถือว่า NG: both=ฟ้า+เหลือง, blue=ฟ้าเท่านั้น, "
                        "yellow=เหลืองเท่านั้น (ไม่ใส่ = ใช้ค่าที่จำไว้); ใส่แล้วจำถาวร")
    p.add_argument("--flip-x", action="store_true",
                   help="กลับภาพแนวนอน (ซ้าย-ขวา) และจำถาวร")
    p.add_argument("--flip-y", action="store_true",
                   help="กลับภาพแนวตั้ง (บน-ล่าง) และจำถาวร")
    args = p.parse_args()
    host = args.host
    if args.tailscale:
        ip = tailscale_ip()
        if ip is None:
            sys.exit("หา Tailscale IP ไม่ได้ — เปิดแอป Tailscale แล้วล็อกอินก่อน\n"
                     "(ถ้าต้องการให้ทั้งวง LAN เห็นแทน ใช้ --host 0.0.0.0 "
                     "แล้วต้องเปิด Windows Firewall เองด้วย)")
        host = ip
        bar = "=" * 60
        print(f"\n{bar}\n  เปิดหน้าเว็บบนเครื่องอื่นด้วย URL นี้:\n\n"
              f"       http://{ip}:{args.port}\n\n"
              f"  เห็นได้เฉพาะเครื่องที่ล็อกอิน Tailscale บัญชีเดียวกัน\n"
              f"  (วง Wi-Fi / LAN มองไม่เห็น จึงไม่ต้องแก้ Windows Firewall)\n\n"
              f"  หมายเหตุ: โหมดนี้ localhost จะเข้าไม่ได้ ต้องใช้ URL ข้างบน\n{bar}\n",
              flush=True)
        # เปิดเบราว์เซอร์ให้เหมือน run_windows.bat — หน่วงไว้ให้ uvicorn ขึ้นก่อน
        # ไม่งั้นแท็บที่เปิดจะเจอ connection refused แล้วผู้ใช้ต้องรีเฟรชเอง
        threading.Timer(2.5, webbrowser.open,
                        args=(f"http://{ip}:{args.port}",)).start()
    cfg = load_config()
    if args.flip_x:
        SES.flip_x = True
        cfg["flip_x"] = True
        save_config(cfg)
    if args.flip_y:
        SES.flip_y = True
        cfg["flip_y"] = True
        save_config(cfg)
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
    if args.direction is not None:     # ระบุมา → ใช้ + จำถาวร
        DIRECTION = args.direction
        cfg["direction"] = DIRECTION
        save_config(cfg)
    else:                              # ไม่ระบุ → ค่าที่จำไว้ (None = ตามพฤติกรรมเดิม)
        saved = cfg.get("direction")
        DIRECTION = saved if saved in DIRECTIONS else None
    uvicorn.run(app, host=host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
