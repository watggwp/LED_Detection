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
IDLE_RELEASE_SEC = 60.0   # ไม่มีคนดู stream + ไม่ได้ตรวจ เกินนี้ → ปล่อยกล้อง (ประหยัดแบต)
MAX_CAMS = 2              # จำนวนกล้องสูงสุดที่รองรับ (แผงชุดที่ 1 / ชุดที่ 2)

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
            cfg = json.load(f)
        return cfg if isinstance(cfg, dict) else {}
    except Exception:
        return {}


def _write_json(path, data):
    """เขียนแบบ atomic — เขียนไฟล์ชั่วคราวก่อนแล้วค่อย replace ทับ
    กันเคสอีกโปรเซสอ่านไปเจอไฟล์ที่เขียนค้างครึ่งทางแล้ว parse ไม่ผ่าน"""
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, path)
        return True
    except OSError:
        try:
            os.remove(tmp)
        except OSError:
            pass
        return False


def update_config(**changes):
    """อ่าน-แก้-เขียน config อย่างปลอดภัย

    ถ้าไฟล์มีอยู่แต่อ่านไม่สำเร็จ จะ "ไม่เขียนทับ" เพราะไม่รู้ค่าเดิม — กันเคสที่เคยเกิดจริง:
    load_config() คืน {} เพราะอ่านชนกับการเขียนของอีกโปรเซส แล้ว save ทับจนค่าที่ตั้งไว้
    (กล้อง/flip/thresh/direction) หายเกลี้ยง เหลือแค่คีย์ที่เพิ่งตั้ง"""
    cfg = {}
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE) as f:
                cfg = json.load(f)
            if not isinstance(cfg, dict):
                cfg = {}
        except Exception as e:
            print(f"[config] อ่าน {CONFIG_FILE} ไม่ได้ ({e}) — "
                  f"ไม่เขียนทับเพื่อกันค่าที่ตั้งไว้หาย", file=sys.stderr)
            return False
    cfg.update(changes)
    return _write_json(CONFIG_FILE, cfg)


def save_config(cfg):
    _write_json(CONFIG_FILE, cfg)


def load_references():
    """b* ขาวอ้างอิงของแต่ละกล้อง — list ยาว MAX_CAMS (None = ยังไม่ calibrate)

    ต้องแยกต่อกล้อง ไม่ใช่ค่าเดียวร่วมกัน: กล้องคนละตัวให้ค่า b* ไม่เท่ากันแม้ส่องดวงเดียวกัน
    (วัดแล้วแค่เปลี่ยน backend ของกล้องตัวเดิม ค่ายังเลื่อน ~0.7 หน่วย คนละรุ่นจะห่างกว่านั้นมาก)
    รองรับไฟล์รูปแบบเก่า {"reference": x} → ยกให้เป็นของกล้องที่ 1"""
    refs = [None] * MAX_CAMS
    try:
        with open(CALIB_FILE) as f:
            data = json.load(f)
    except Exception:
        return refs
    if isinstance(data.get("references"), list):
        for i, v in enumerate(data["references"][:MAX_CAMS]):
            refs[i] = v
    elif data.get("reference") is not None:      # ไฟล์เก่าก่อนรองรับ 2 กล้อง
        refs[0] = data["reference"]
    return refs


def save_references(refs):
    _write_json(CALIB_FILE, {"references": list(refs)})


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
    """สถานะของ "หนึ่งกล้อง" — กล้อง + engine ตรวจของตัวเอง
    กล้องเปิดเมื่อมีคนดู/ตรวจ, ปล่อยเองตอน idle (D11)

    slot = ลำดับกล้อง (0 = แผงชุดแรก, 1 = แผงชุดที่สอง) ใช้เป็น key ของค่าที่จำไว้
    ทุกอย่างที่ผูกกับตัวกล้อง (index, flip, ค่าอ้างอิง) อยู่ในนี้ ไม่ใช่ global
    ส่วนที่เป็นนโยบายร่วม (thresh, direction, จำนวนแผง) ยังเป็น global ใช้ร่วมกัน"""

    def __init__(self, slot, cam_index=0, flip_x=False, flip_y=False,
                 reference=None):
        self.slot = slot
        self.cam_index = cam_index
        self.lock = threading.Lock()
        self.thread = None
        self.stop_flag = False
        self.cap = None        # cap ปัจจุบัน (ให้ A3 OpenCV ตั้ง property ได้ ใน thread นี้)
        self.engine = None     # None = โหมดพรีวิว (โชว์ภาพเฉยๆ ไม่ตรวจ)
        self.jpeg = None       # เฟรมล่าสุด (bytes)
        self.status = None     # สถานะล่าสุด (dict, มี cct)
        self.frame = None      # เฟรมดิบล่าสุด (ก่อน annotate) — ใช้ตอน calibrate
        self.reference = reference   # b* ขาวอ้างอิงของกล้องนี้ (คงไว้ข้าม start/stop)
        self.last_stream = time.time()     # เวลาเฟรมล่าสุดที่ stream ถูกดึง (D11)
        self.cam_error = None
        self.flip_x = flip_x   # กลับภาพแนวนอน (ซ้าย-ขวา)
        self.flip_y = flip_y   # กลับภาพแนวตั้ง (บน-ล่าง)
        # คิวคำสั่งตั้ง exposure ฝั่ง OpenCV (ต้องทำใน capture thread ที่ถือ cap)
        self.exp_pending = False
        self.exp_value = None
        self.exp_event = threading.Event()
        self.exp_result = (False, "")

    def detecting(self):
        return self.engine is not None


def _load_sessions():
    """สร้าง Session ตามที่จำไว้ใน config

    รูปแบบใหม่: "cams": [{"index":1,"flip_x":true,...}, {...}]
    รูปแบบเก่า (กล้องเดียว): "cam_index"/"flip_x"/"flip_y" ที่ระดับบนสุด — แปลงให้อัตโนมัติ
    เพื่อให้เครื่องที่อัปเดตมาแล้วยังใช้ค่าเดิมได้ทันทีโดยไม่ต้องตั้งใหม่"""
    cfg = load_config()
    refs = load_references()
    cams = cfg.get("cams")
    if not isinstance(cams, list) or not cams:
        cams = [{"index": int(cfg.get("cam_index", 0)),
                 "flip_x": bool(cfg.get("flip_x", False)),
                 "flip_y": bool(cfg.get("flip_y", False))}]
    out = []
    for slot, c in enumerate(cams[:MAX_CAMS]):
        out.append(Session(slot,
                           cam_index=int(c.get("index", 0)),
                           flip_x=bool(c.get("flip_x", False)),
                           flip_y=bool(c.get("flip_y", False)),
                           reference=refs[slot] if slot < len(refs) else None))
    return out


def save_sessions_config():
    """เขียนรายการกล้อง (index + flip ของแต่ละตัว) ลง config"""
    update_config(
        cams=[{"index": s.cam_index, "flip_x": s.flip_x, "flip_y": s.flip_y}
              for s in SESSIONS],
        # ค่าเก่าระดับบนสุด: คงไว้ให้ตรงกับกล้องตัวแรก เผื่อ downgrade กลับเวอร์ชันเดิม
        cam_index=SESSIONS[0].cam_index,
        flip_x=SESSIONS[0].flip_x, flip_y=SESSIONS[0].flip_y)


SESSIONS = _load_sessions()
_cfg0 = load_config()
# จำนวนแผง × ดวงต่อแผง "ต่อกล้องหนึ่งตัว" — ใช้ร่วมกันทุกกล้อง
# กล้องที่ 1 ได้แผง 1..PIECES, กล้องที่ 2 ได้แผง PIECES+1..PIECES*2
PIECES = int(_cfg0.get("pieces", 1))
PER_PIECE = int(_cfg0.get("per_piece", 2))
del _cfg0
EXPOSURE_INIT = DEFAULT_EXPOSURE   # ตั้งจาก --exposure: ล็อกแสงทันทีที่กล้องเปิด


def get_ses(cam):
    """Session ของกล้องลำดับ cam — None ถ้าไม่มีกล้องนั้น"""
    return SESSIONS[cam] if 0 <= cam < len(SESSIONS) else None

# กันเปิด/ปิดกล้องชนกัน — ทุกจุดที่ "เปิด/ปิด/ไล่สแกน" กล้องต้องถือล็อกนี้
# ไม่งั้นสองเธรดแตะ DirectShow device ตัวเดียวกันพร้อมกันได้ เช่น /api/cameras กำลัง probe
# index 0 อยู่ แล้ว /stream (ซึ่งเรียก ensure_camera ทุกครั้ง) เปิด index 0 ขึ้นมาซ้อน
# → OpenCV โยน "Unknown C++ exception" ทิ้ง capture thread ตาย จอดำค้าง
# ใช้ RLock เพราะ api_cameras ถือล็อกอยู่แล้วยังต้องเรียก stop_camera()/ensure_camera() ต่อ
CAM_CTL = threading.RLock()
JOIN_TIMEOUT = 8.0     # เผื่อ VideoCapture() ที่ค้างอยู่ (interrupt กลางคันไม่ได้) ให้จบเอง


def capture_loop(ses):
    """thread ถือกล้อง "หนึ่งตัว" — สลับพรีวิว/ตรวจตาม ses.engine, ปล่อยกล้องเองเมื่อ idle
    แต่ละกล้องมี thread ของตัวเอง ตัวหนึ่งพังไม่ลามไปอีกตัว"""
    stopping = lambda: ses.stop_flag        # noqa: E731 — สั้นกว่าและใช้ที่เดียว
    cap = open_cap(ses.cam_index, stopping)
    if cap is None and not ses.stop_flag and ses.slot == 0:
        # index ที่จำไว้เปิดไม่ได้ (อาจสลับพอร์ต) — ลองหาใหม่จาก VID/PID (Windows)
        # ทำเฉพาะกล้องตัวแรก เพราะ VID/PID ผูกกับรุ่นเดียว ถ้ามีสองตัวรุ่นเดียวกันจะแยกไม่ออก
        # แล้วจะไปแย่ง index ของอีกตัวมาใช้
        taken = {s.cam_index for s in SESSIONS if s is not ses}
        alt = find_index_by_vidpid(CAMERA_VIDPID)
        if alt is not None and alt != ses.cam_index and alt not in taken:
            cap = open_cap(alt, stopping)
            if cap is not None:
                ses.cam_index = alt
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
        idx = ses.cam_index
        who = (f" ({names[idx]})" if 0 <= idx < len(names) else "")
        with ses.lock:
            ses.cam_error = (f"เปิดกล้อง index {idx}{who} ไม่ได้ — "
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


def ensure_camera(ses):
    with CAM_CTL:
        with ses.lock:
            t = ses.thread
            # thread ที่ยังไม่ตั้ง stop_flag = ตัวที่ใช้งานอยู่จริง → ไม่ต้องทำอะไร
            if t is not None and t.is_alive() and not ses.stop_flag:
                return
        # ถ้าตัวเก่ากำลังปิดตัวอยู่ ต้องรอให้มันปล่อยกล้องก่อน ไม่งั้นตัวใหม่จะเปิดซ้ำ index เดิมไม่ได้
        if t is not None and t.is_alive():
            t.join(timeout=JOIN_TIMEOUT)
        with ses.lock:
            ses.stop_flag = False
            ses.last_stream = time.time()
            ses.thread = threading.Thread(target=capture_loop, args=(ses,),
                                          daemon=True)
            ses.thread.start()


def ensure_all_cameras():
    for s in SESSIONS:
        ensure_camera(s)


def wait_first_frame(ses, timeout=8.0):
    """รอจนกล้องส่งเฟรมแรก (หรือแจ้ง error) — คืน True ถ้ามีภาพแล้ว
    ใช้ตอนสลับกล้อง เพื่อให้ตอบกลับหน้าเว็บหลังภาพพร้อมจริง ไม่ใช่ปล่อยให้ <img> ค้างขาว"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        with ses.lock:
            if ses.jpeg is not None:
                return True
            if ses.cam_error is not None:
                return False
            alive = ses.thread is not None and ses.thread.is_alive()
        if not alive:
            return False
        time.sleep(0.05)
    return False


def stop_camera(ses):
    """หยุด capture thread + ปล่อยกล้อง (รอ join จนกว่าจะดับจริง) — ใช้ก่อนสแกน/เปลี่ยนกล้อง"""
    with CAM_CTL:
        with ses.lock:
            t = ses.thread
            cap = ses.cap
            ses.stop_flag = True

        if t is None or not t.is_alive():
            with ses.lock:
                ses.thread = None
            return

        # ปล่อย cap ทันทีเพื่อปลดล็อก cap.read() ที่ค้างอยู่ใน OpenCV thread
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

        t.join(timeout=JOIN_TIMEOUT)
        with ses.lock:
            if not t.is_alive():
                ses.thread = None


def stop_all_cameras():
    for s in SESSIONS:
        stop_camera(s)


# ---------- โมเดล request ----------

class StartReq(BaseModel):
    pieces: int
    per_piece: int = 2


class ExposureReq(BaseModel):
    value: Optional[int] = None  # None = auto
    cam: int = 0                 # กล้องลำดับไหน (0 = ตัวแรก)


class CamReq(BaseModel):
    index: int
    cam: int = 0


class FlipReq(BaseModel):
    flip_x: Optional[bool] = None
    flip_y: Optional[bool] = None
    cam: int = 0


class CamCountReq(BaseModel):
    count: int                   # จำนวนกล้องที่ใช้ (1 หรือ 2)


class ThreshReq(BaseModel):
    value: float


class DirectionReq(BaseModel):
    value: str


class CalibrateReq(BaseModel):
    # ติ๊กออก = ข้ามด่านตรวจว่าชิ้นที่วางสีใกล้กันไหม แล้ว calibrate เลย
    check_spread: bool = True
    cam: int = 0


# ---------- endpoints ----------

def _bad_cam(cam):
    return JSONResponse({"error": f"ไม่มีกล้องลำดับที่ {cam + 1}"}, status_code=400)


@app.post("/api/flip")
def api_flip(req: FlipReq):
    ses = get_ses(req.cam)
    if ses is None:
        return _bad_cam(req.cam)
    with ses.lock:
        if req.flip_x is not None:
            ses.flip_x = req.flip_x
        if req.flip_y is not None:
            ses.flip_y = req.flip_y
        fx, fy = ses.flip_x, ses.flip_y
    save_sessions_config()
    return {"ok": True, "flip_x": fx, "flip_y": fy, "cam": req.cam}

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
    update_config(direction=DIRECTION)
    for s in SESSIONS:
        with s.lock:
            if s.engine is not None:
                s.engine.direction = DIRECTION
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
    update_config(thresh=THRESH)
    for s in SESSIONS:
        with s.lock:
            if s.engine is not None:
                # engine อ่าน self.thresh ใหม่ทุกเฟรม → เปลี่ยนสดได้ ไม่ต้องสร้าง engine ใหม่
                # (สร้างใหม่จะทำให้สล็อตที่ยึดไว้กับ hold timer รีเซ็ตหมดโดยไม่จำเป็น)
                s.engine.thresh = THRESH
    return {"ok": True, "thresh": THRESH}


@app.post("/api/exposure")
def api_exposure(req: ExposureReq):
    ses = get_ses(req.cam)
    if ses is None:
        return _bad_cam(req.cam)
    ensure_camera(ses)
    if _use_uvc():
        ok, msg = set_exposure(req.value)
        return {"ok": ok, "msg": msg}
    # OpenCV: ส่งให้ capture thread ตั้งให้ (กัน race กับ cap.read())
    ses.exp_value = req.value
    ses.exp_result = (False, "หมดเวลา — กล้องอาจยังไม่พร้อม ลองใหม่")
    ses.exp_event.clear()
    ses.exp_pending = True
    ses.exp_event.wait(timeout=2.5)
    ok, msg = ses.exp_result
    return {"ok": ok, "msg": msg, "cam": req.cam}


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
    # index ที่แต่ละกล้องยึดอยู่ → บอกหน้าเว็บว่าตัวไหนถูกใช้เป็นกล้องที่ 1 / ที่ 2 แล้ว
    used = {s.cam_index: s.slot for s in SESSIONS}
    # ถือ CAM_CTL ตลอดการสแกน — ระหว่างนี้ห้ามใครเปิด/ปิดกล้อง (/stream เรียก ensure_camera
    # ทุกครั้งที่โหลด ถ้าปล่อยให้แทรกได้จะกลายเป็นสองเธรดแย่ง device เดียวกันแล้ว OpenCV พัง)
    with CAM_CTL:
        live = {}      # index -> (frame, fx, fy) ของกล้องที่กำลังเปิดและมีภาพอยู่แล้ว
        for s in SESSIONS:
            with s.lock:
                alive = s.thread is not None and s.thread.is_alive()
                has_frame = alive and s.frame is not None
                if has_frame:
                    live[s.cam_index] = (s.frame.copy(), s.flip_x, s.flip_y)
            if not has_frame:
                # ยังไม่มีภาพ → ปล่อยกล้องตัวนี้ก่อน ไม่งั้น probe index เดิมไม่ได้
                stop_camera(s)

        scan = set(range(len(names) if names else SCAN_MAX_INDEX))
        scan.update(used)            # เผื่อ index ที่จำไว้อยู่นอกช่วงที่ OS รายงาน
        scan -= set(live)            # ตัวที่มีภาพสดอยู่แล้ว ไม่ต้องแตะ

        found = {}
        targets = sorted(scan)
        if targets:
            with ThreadPoolExecutor(max_workers=min(6, len(targets))) as ex:
                for i, res in zip(targets, ex.map(probe_cap, targets)):
                    found[i] = res
        for i, (fr, _, _) in live.items():
            found[i] = (True, fr)
        ensure_all_cameras()   # กล้องที่ยังเปิดอยู่ ฟังก์ชันนี้ไม่ทำอะไร

    # flip ของกล้องตัวแรกใช้เป็นค่าตั้งต้นให้ thumbnail ที่ยัง probe มาดิบๆ
    fx0, fy0 = SESSIONS[0].flip_x, SESSIONS[0].flip_y
    cams = []
    for i in sorted(found):
        ok, fr = found[i]
        if ok and fr is not None:
            # เฟรมที่ยืมจากภาพสด กลับด้านมาแล้ว ที่ probe มาใหม่ต้องกลับให้ตรงกัน
            if i not in live:
                fr = _apply_flip(fr, fx0, fy0)
            cams.append({"index": i, "thumb": _thumb(fr),
                         "name": names[i] if i < len(names) else "",
                         "used_by": used.get(i, -1)})   # -1 = ยังไม่ถูกใช้
    # กล้องที่ OS เห็นแต่เปิดไม่ได้ (โปรแกรมอื่นยึดอยู่ / ถอดสายค้าง) — บอกผู้ใช้ไปตรงๆ
    busy = [{"index": i, "name": names[i]} for i in sorted(found)
            if not found[i][0] and i < len(names)]
    return {"cameras": cams, "busy": busy,
            "current": [s.cam_index for s in SESSIONS],
            "n_cams": len(SESSIONS)}


@app.post("/api/select_camera")
def api_select_camera(req: CamReq):
    ses = get_ses(req.cam)
    if ses is None:
        return _bad_cam(req.cam)
    # กล้องสองตัวใช้ index เดียวกันไม่ได้ — DirectShow เปิดซ้ำตัวเดิมไม่ได้อยู่แล้ว
    for s in SESSIONS:
        if s is not ses and s.cam_index == req.index:
            return JSONResponse(
                {"error": f"index {req.index} ถูกใช้เป็นกล้องที่ {s.slot + 1} อยู่แล้ว "
                          f"— เลือกตัวอื่น"}, status_code=400)
    # ปิด-สลับ-เปิด ต้องเป็นก้อนเดียว ไม่งั้น /stream ที่ยิงเข้ามาพอดีจะเปิดกล้อง "ตัวเก่า"
    # คืนมาคั่นกลาง แล้วเราไปเปิดตัวใหม่ทับ = สองเธรดถือกล้องพร้อมกัน
    with CAM_CTL:
        stop_camera(ses)
        ses.cam_index = req.index
        save_sessions_config()
        with ses.lock:
            ses.engine = None   # กล้องเปลี่ยน — เริ่มตรวจใหม่ค่อยยึดสล็อตใหม่
            ses.status = None
            ses.jpeg = None
            ses.frame = None
            ses.cam_error = None   # ล้าง error ตัวเก่า ไม่งั้น wait_first_frame เชื่อค่าเก่า
        ensure_camera(ses)
    # รอให้ภาพแรกมาก่อนค่อยตอบ หน้าเว็บจะได้ไม่รีโหลด /stream ตอนกล้องยังไม่พร้อม
    ok = wait_first_frame(ses)
    with ses.lock:
        err = ses.cam_error
    return {"ok": ok, "index": req.index, "cam": req.cam, "error": err}


@app.post("/api/cam_count")
def api_cam_count(req: CamCountReq):
    """เพิ่ม/ลดจำนวนกล้องที่ใช้ (1 หรือ 2) — ต่อกล้องตัวที่สองแล้วค่อยกดเพิ่ม"""
    global SESSIONS
    n = req.count
    if not 1 <= n <= MAX_CAMS:
        return JSONResponse({"error": f"จำนวนกล้องต้องอยู่ระหว่าง 1 ถึง {MAX_CAMS}"},
                            status_code=400)
    with CAM_CTL:
        if n < len(SESSIONS):
            for s in SESSIONS[n:]:
                stop_camera(s)
            SESSIONS = SESSIONS[:n]
        while len(SESSIONS) < n:
            slot = len(SESSIONS)
            used = {s.cam_index for s in SESSIONS}
            # เดา index ที่ยังว่างให้ก่อน ผู้ใช้ค่อยกด "เลือกกล้อง" เปลี่ยนทีหลังได้
            free = next((i for i in range(SCAN_MAX_INDEX) if i not in used), 0)
            SESSIONS.append(Session(slot, cam_index=free))
        save_sessions_config()
        refs = load_references()
        for s in SESSIONS:      # ค่าอ้างอิงของแต่ละกล้องอ่านกลับมาตาม slot
            s.reference = refs[s.slot] if s.slot < len(refs) else None
        ensure_all_cameras()
    return {"ok": True, "n_cams": len(SESSIONS),
            "cams": [s.cam_index for s in SESSIONS]}


@app.post("/api/start")
def api_start(req: StartReq):
    """เริ่มตรวจทุกกล้องพร้อมกัน — แต่ละกล้องรับผิดชอบแผงคนละชุด
    กล้อง 1 = แผง 1..pieces, กล้อง 2 = แผง pieces+1..pieces*2"""
    global PIECES, PER_PIECE
    total = req.pieces * req.per_piece
    if req.pieces < 1 or req.per_piece < 1:
        return JSONResponse({"error": "จำนวนต้องมากกว่า 0"}, status_code=400)
    if total < 2:
        return JSONResponse({"error": "ต้องมีอย่างน้อย 2 ดวงถึงเทียบกันได้ (ต่อกล้อง)"},
                            status_code=400)
    if total > MAX_SPOTS:
        return JSONResponse(
            {"error": f"กล้องละ {total} ดวง เกินขีดจำกัด {MAX_SPOTS} ดวงต่อกล้อง "
                      f"(แบ่งเทสทีละไม่เกิน {MAX_SPOTS // req.per_piece} ชิ้นต่อกล้อง)"},
            status_code=400)
    ensure_all_cameras()
    # จำจำนวนที่ใช้ล่าสุด — เปิดโปรแกรมครั้งหน้าฟอร์มจะเติมให้เอง ไม่ต้องกรอกซ้ำ
    PIECES, PER_PIECE = req.pieces, req.per_piece
    update_config(pieces=req.pieces, per_piece=req.per_piece)
    modes = []
    for s in SESSIONS:
        with s.lock:
            s.status = None
            s.engine = MultiWatch(total, thresh=THRESH, reference=s.reference,
                                  direction=DIRECTION)
            modes.append("absolute" if s.reference is not None else "relative")
            direction = s.engine.resolved_direction()
    return {"ok": True, "total_spots": total, "n_cams": len(SESSIONS),
            "mode": "absolute" if all(m == "absolute" for m in modes) else "relative",
            "modes": modes, "direction": direction}


@app.post("/api/calibrate")
def api_calibrate(req: Optional[CalibrateReq] = None):
    """วางชิ้นดีครบตามจำนวน → เรียกตอนกำลังตรวจ — ตั้งขาวอ้างอิงและจำถาวร

    body ว่างได้ (ของเดิมเรียกแบบไม่ส่ง body) → ตรวจความห่างตามปกติ"""
    check_spread = True if req is None else req.check_spread
    cam = 0 if req is None else req.cam
    ses = get_ses(cam)
    if ses is None:
        return _bad_cam(cam)
    with ses.lock:
        engine = ses.engine
        frame = ses.frame.copy() if ses.frame is not None else None
    if engine is None:
        return JSONResponse({"error": "กดเริ่มตรวจก่อน แล้วค่อย calibrate "
                                      "(ต้องรู้จำนวนจุดก่อน)"}, status_code=400)
    if frame is None:
        return JSONResponse({"error": f"กล้องที่ {cam + 1} ยังไม่มีภาพ"},
                            status_code=400)
    ref, msg = engine.calibrate(frame, check_spread=check_spread)
    if ref is None:
        return JSONResponse({"error": msg}, status_code=400)
    with ses.lock:
        ses.reference = ref
    # เขียนทับเฉพาะช่องของกล้องนี้ ไม่แตะค่าอ้างอิงของอีกกล้อง
    refs = load_references()
    refs[ses.slot] = ref
    save_references(refs)
    return {"ok": True, "reference": round(ref, 2), "msg": msg, "cam": cam,
            "checked_spread": check_spread}


@app.post("/api/calibrate/clear")
def api_calibrate_clear(req: Optional[CalibrateReq] = None):
    """ล้างขาวอ้างอิงของกล้องที่ระบุ — กลับโหมดเทียบกันเองเฉพาะกล้องนั้น"""
    cam = 0 if req is None else req.cam
    ses = get_ses(cam)
    if ses is None:
        return _bad_cam(cam)
    with ses.lock:
        ses.reference = None
        if ses.engine is not None:
            ses.engine.reference = None
    refs = load_references()
    refs[ses.slot] = None
    if all(r is None for r in refs):
        try:
            os.remove(CALIB_FILE)     # ไม่เหลือค่าอ้างอิงเลย → ลบไฟล์ทิ้งเหมือนเดิม
        except OSError:
            pass
    else:
        save_references(refs)
    return {"ok": True, "cam": cam}


@app.post("/api/stop")
def api_stop():
    for s in SESSIONS:
        with s.lock:
            s.engine = None  # กลับโหมดพรีวิว — กล้องยังเปิดอยู่ (จะปล่อยเองตอน idle)
            s.status = None
    return {"ok": True}


def _pieces_from(st, per_piece, n_pieces, offset, cam):
    """แปลง spots ของกล้องหนึ่งตัวเป็นรายการแผง พร้อมเลื่อนเลขแผงตามกล้อง
    (กล้องที่ 2 เริ่มนับต่อจากกล้องที่ 1 เช่น 6-10 เมื่อกล้องละ 5 แผง)"""
    out = []
    for j in range(n_pieces):
        spots = st["spots"][j * per_piece:(j + 1) * per_piece]
        sts = [s["state"] for s in spots]
        if "BAD" in sts:
            pstate = "BAD"
        elif "MISSING" in sts:
            pstate = "INCOMPLETE"   # มีดวงหาย — ยังตัดสินไม่ครบ
        else:
            pstate = "OK"
        out.append({"piece": offset + j + 1, "state": pstate, "spots": spots,
                    "cam": cam})
    return out


@app.get("/api/status")
def api_status():
    """สถานะรวมทุกกล้อง — ตารางผลเป็นชุดเดียวเรียงแผง 1..N ต่อเนื่องข้ามกล้อง

    กล้องตัวไหนพัง ตัวที่เหลือยังตรวจต่อได้ แผงของกล้องที่พังจะขึ้นเป็น 'ไม่พบ'
    พร้อม cam_error ของตัวนั้นบอกสาเหตุ"""
    cams_info, pieces_all, errors = [], [], []
    any_running = False
    direction = effective_direction(SESSIONS[0].reference)
    for s in SESSIONS:
        with s.lock:
            st, eng = s.status, s.engine
            detecting = eng is not None
            info = {"cam": s.slot, "index": s.cam_index,
                    "flip_x": s.flip_x, "flip_y": s.flip_y,
                    "cam_error": s.cam_error,
                    "ref": round(s.reference, 2) if s.reference is not None else None,
                    "mode": "absolute" if s.reference is not None else "relative",
                    "running": detecting}
            if eng is not None:
                direction = eng.resolved_direction()
        cams_info.append(info)
        if s.cam_error:
            errors.append(f"กล้องที่ {s.slot + 1}: {s.cam_error}")
        if not detecting:
            continue
        any_running = True
        offset = s.slot * PIECES
        if st is None:
            info["error"] = "กำลังเริ่มตรวจ..."
        elif "error" in st:
            info["error"] = st["error"]
        else:
            pieces_all.extend(_pieces_from(st, PER_PIECE, PIECES, offset, s.slot))
            info["t"] = st["t"]
            info["n_missing"] = st.get("n_missing", 0)

    common = {"saved_pieces": PIECES, "saved_per_piece": PER_PIECE,
              "thresh": THRESH, "n_cams": len(SESSIONS), "cams": cams_info,
              "max_spots": MAX_SPOTS,
              "direction": direction, "direction_label": DIR_LABEL[direction],
              "direction_pinned": DIRECTION is not None,
              # ค่าเดิมที่หน้าเว็บ/สคริปต์เก่าอ่าน — ยึดกล้องตัวแรกไว้เพื่อความเข้ากันได้
              "flip_x": SESSIONS[0].flip_x, "flip_y": SESSIONS[0].flip_y,
              "cam_error": SESSIONS[0].cam_error}
    if not any_running:
        return dict(common, running=False, preview=True)
    out = dict(common, running=True, pieces=PIECES, per_piece=PER_PIECE)
    if pieces_all:
        out["pieces_detail"] = sorted(pieces_all, key=lambda p: p["piece"])
        out["mode"] = ("absolute"
                       if all(c["mode"] == "absolute" for c in cams_info)
                       else "relative")
        out["ref"] = cams_info[0]["ref"]
    else:
        out["error"] = next((c["error"] for c in cams_info if c.get("error")),
                            "กำลังเริ่มตรวจ...")
    if errors:
        out["cam_errors"] = errors
    return out


@app.get("/stream")
def stream(cam: int = 0):
    ses = get_ses(cam)
    if ses is None:
        return JSONResponse({"error": f"ไม่มีกล้องลำดับที่ {cam + 1}"},
                            status_code=404)
    ensure_camera(ses)

    def gen():
        sent = False
        while True:
            with ses.lock:
                buf = ses.jpeg
                err = ses.cam_error
                ses.last_stream = time.time()   # บอกว่ายังมีคนดูอยู่ (D11)
            if buf is None:
                # กล้องเปิดไม่ได้และยังไม่เคยส่งภาพเลย → จบ stream ไปเลย
                # ไม่งั้น <img> ฝั่งเบราว์เซอร์จะค้างรอจนหมดเวลา แล้วผู้ใช้ไม่รู้ว่าเกิดอะไร
                # (หน้าเว็บมี cam_error จาก /api/status บอกสาเหตุอยู่แล้ว)
                if err is not None and not sent:
                    return
                time.sleep(0.2)
                continue
            sent = True
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
  #videos { display: flex; flex-wrap: wrap; gap: 12px; }
  .vwrap { flex: 1 1 320px; min-width: 280px; }
  .vwrap img { width: 100%; border-radius: 10px; display: block; background: #0b0d11; }
  .vlabel { font-size: 14px; font-weight: 600; color: #9fb4d0; margin-bottom: 4px; }
  .vwrap.off .vlabel { color: #ff6b6b; }
  .verr { color: #ff6b6b; font-size: 13px; margin-top: 4px; min-height: 18px; }
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
        <span style="font-weight:600">📺 ใช้กี่กล้อง:</span>
        <select id="ncam" onchange="setCamCount(this.value)"
                style="font-size:16px;padding:6px;border-radius:6px;border:1px solid #3a4250;background:#11141a;color:#e8eaed">
          <option value="1">1 กล้อง</option>
          <option value="2">2 กล้อง (แบ่งแผงคนละชุด)</option>
        </select>
        <span class="hint" id="ncamhint"></span>
      </div>
      <div id="camtabs" style="margin-bottom:10px"></div>
      <div style="margin-bottom:10px">
        <button onclick="loadCams()" style="background:#3d5a80;color:#fff">📷 เลือกกล้อง</button>
        <button id="btnDoneCam" onclick="closeCamSelector()" style="background:#2e7d32;color:#fff;display:none">✅ เสร็จสิ้นการเลือกกล้อง</button>
        <span class="hint" id="pickhint">กล้องสลับ index เอง? กดปุ่มนี้แล้วคลิกภาพที่เห็นไฟ LED</span>
        <div id="cams"></div>
      </div>
      <label>จำนวนชิ้น <input type="number" id="pieces" value="1" min="1" max="5"></label>
      <label>ดวงต่อชิ้น <input type="number" id="per" value="2" min="1" max="4"></label>
      <span class="hint" id="savedinfo"></span>
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
    <div id="videos">
      <div class="vwrap" id="vwrap0">
        <div class="vlabel" id="vlabel0">กล้องที่ 1</div>
        <img id="video0" src="/stream?cam=0" alt="กำลังต่อกล้อง...">
        <div class="verr" id="verr0"></div>
      </div>
      <div class="vwrap" id="vwrap1" style="display:none">
        <div class="vlabel" id="vlabel1">กล้องที่ 2</div>
        <img id="video1" alt="กำลังต่อกล้อง...">
        <div class="verr" id="verr1"></div>
      </div>
    </div>
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

// ---- หลายกล้อง ----
// activeCam = กล้องที่ปุ่มตั้งค่า (เลือกกล้อง/แสง/กลับภาพ/calibrate) จะไปมีผลด้วย
let nCams = 1, activeCam = 0, camsInfo = [];

async function setCamCount(n) {
  n = +n;
  const r = await (await fetch('/api/cam_count', {method:'POST',
    headers:{'Content-Type':'application/json'}, body: JSON.stringify({count: n})})).json();
  const hint = document.getElementById('ncamhint');
  if (r.error) { hint.textContent = '⚠️ ' + r.error; return; }
  hint.textContent = n > 1
    ? 'กล้องที่ 2 รับแผงชุดถัดไป — อย่าลืมกด "เลือกกล้อง" ตั้ง index ให้ตัวที่ 2 ด้วย'
    : '';
  if (activeCam >= n) activeCam = 0;
  applyCamCount(n);
  // สตรีมของกล้องที่เพิ่งเพิ่ม ต้องสั่งโหลดเอง (cache-bust กันภาพเก่าค้าง)
  for (let i = 0; i < n; i++) reloadStream(i);
}

function applyCamCount(n) {
  nCams = n;
  document.getElementById('ncam').value = n;
  document.getElementById('vwrap1').style.display = n > 1 ? '' : 'none';
  renderCamTabs();
}

function reloadStream(i) {
  document.getElementById('video' + i).src = '/stream?cam=' + i + '&ts=' + Date.now();
}

function renderCamTabs() {
  const box = document.getElementById('camtabs');
  if (nCams < 2) { box.innerHTML = ''; return; }   // กล้องเดียวไม่ต้องมีแท็บให้รก
  let h = '<span style="font-weight:600">⚙️ ตั้งค่ากล้อง:</span>';
  for (let i = 0; i < nCams; i++) {
    const on = i === activeCam;
    h += `<button onclick="setActiveCam(${i})" style="background:${on ? '#3d5a80' : '#2a303a'};color:#fff">`
       + `${on ? '● ' : ''}กล้องที่ ${i + 1}</button>`;
  }
  h += '<span class="hint">ปุ่มเลือกกล้อง / ปรับแสง / กลับภาพ / calibrate จะมีผลกับกล้องที่เลือกไว้</span>';
  box.innerHTML = h;
}

function setActiveCam(i) {
  activeCam = i;
  renderCamTabs();
  updateFlipUI();
  document.getElementById('calinfo').textContent = '';
  document.getElementById('msg').textContent = 'ตอนนี้กำลังตั้งค่ากล้องที่ ' + (i + 1);
}

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
    const mine = c.used_by === activeCam;
    d.className = 'camcard' + (mine ? ' cur' : '');
    d.dataset.name = c.name || ('กล้อง #' + c.index);
    // used_by บอกว่า index นี้ถูกกล้องตัวไหนยึดไว้ (-1 = ว่าง)
    const tag = mine ? ' (ใช้อยู่)'
              : (c.used_by >= 0 ? ` (เป็นกล้องที่ ${c.used_by + 1})` : '');
    d.innerHTML = `<img src="${c.thumb}"><span>${d.dataset.name}${tag}</span>`;
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
  document.getElementById('msg').textContent = !j.cameras.length
    ? 'พบกล้องแต่เปิดไม่ได้สักตัว — ปิดโปรแกรมที่ใช้กล้องอยู่ (Teams/Zoom/OBS) แล้วกด "เลือกกล้อง" ใหม่'
    : (nCams > 1 ? `คลิกเลือกกล้องให้ "กล้องที่ ${activeCam + 1}" — เสร็จแล้วกด "เสร็จสิ้น"`
                 : 'คลิกเลือกกล้องเพื่อสลับดูภาพสดได้ทันที — เมื่อพอใจแล้วกด "เสร็จสิ้น"');
}
async function selectCam(i) {
  const card0 = document.getElementById('camcard-' + i);
  const nm = (card0 && card0.dataset.name) || ('กล้อง #' + i);
  const who = nCams > 1 ? `กล้องที่ ${activeCam + 1} = ` : '';
  document.getElementById('msg').textContent = `กำลังตั้ง ${who}${nm}...`;
  const r = await (await fetch('/api/select_camera', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({index: i, cam: activeCam})})).json();
  if (!r.ok) {
    document.getElementById('msg').textContent =
      `⚠️ ตั้ง ${who}${nm} ไม่สำเร็จ — ` + (r.error || 'กล้องไม่ส่งภาพ');
    return;
  }
  const box = document.getElementById('cams');
  for (let card of box.getElementsByClassName('camcard')) {
    card.classList.remove('cur');
    const span = card.querySelector('span');
    if (span) span.textContent = span.textContent.replace(/ \\(ใช้อยู่\\)$/, '');
  }
  const selCard = document.getElementById('camcard-' + i);
  if (selCard) {
    selCard.classList.add('cur');
    const span = selCard.querySelector('span');
    if (span) span.textContent = nm + ' (ใช้อยู่)';
  }
  document.getElementById('msg').textContent = `✅ ตั้ง ${who}${nm} แล้ว`;
  reloadStream(activeCam);   // รีโหลดภาพสดของกล้องตัวนั้น (cache-bust)
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
  if (j.ok) document.getElementById('savedinfo').textContent =
    `(จำค่านี้แล้ว: ${pieces} ชิ้น × ${per} ดวง = ${pieces * per} ดวง)`;
}
async function stop() {
  await fetch('/api/stop', {method:'POST'});
  document.getElementById('msg').textContent = '';
}
function camTag() { return nCams > 1 ? `[กล้องที่ ${activeCam + 1}] ` : ''; }

async function calibrate() {
  const chk = document.getElementById('calspread').checked;
  const r = await fetch('/api/calibrate', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({check_spread: chk, cam: activeCam})});
  const j = await r.json();
  // เขียนลง calinfo (ช่องข้างปุ่ม calibrate) ไม่ใช่ msg ที่ปุ่มอื่นใช้ร่วมกัน
  // → ย้อมสีเตือนได้โดยไม่ไปติดค้างกับข้อความของปุ่มอื่น
  const el = document.getElementById('calinfo');
  el.textContent = camTag() + (j.ok ? '✓ ' + j.msg : '✗ ' + j.error);
  el.style.color = !j.ok ? '#ff6b6b'
                 : (j.checked_spread ? '#6fdc8c' : '#ffb454');
  document.getElementById('msg').textContent = '';
}
async function clearCalib() {
  await fetch('/api/calibrate/clear', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({cam: activeCam})});
  const el = document.getElementById('calinfo');
  el.textContent = camTag() + 'ล้างอ้างอิงแล้ว — กลับโหมดเทียบกันเอง';
  el.style.color = '';
}
let expLevel = 5;            // ระดับแสงปัจจุบัน (น้อย = มืด) ตรงกับ DEFAULT_EXPOSURE ตอนเปิดกล้อง
async function applyExp(v) {
  const j = await (await fetch('/api/exposure', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({value: v, cam: activeCam})})).json();
  document.getElementById('msg').textContent =
    camTag() + (j.msg || (j.ok ? 'เรียบร้อย' : 'ลองใหม่อีกครั้ง'));
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
    body: JSON.stringify({flip_x: newX, flip_y: newY, cam: activeCam})
  });
  const j = await r.json();
  flipX = j.flip_x; flipY = j.flip_y;
  updateFlipUI();
  document.getElementById('msg').textContent = camTag() +
    `กลับภาพ: แกน X (ซ้าย-ขวา) = ${flipX ? 'เปิด' : 'ปิด'}, แกน Y (บน-ล่าง) = ${flipY ? 'เปิด' : 'ปิด'}`;
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

let formFilled = false;   // เติมฟอร์มจากค่าที่จำไว้แล้วหรือยัง (ทำครั้งเดียว)

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
function renderCamStatus(j) {
  camsInfo = j.cams || [];
  if (j.n_cams && j.n_cams !== nCams) {
    if (activeCam >= j.n_cams) activeCam = 0;
    applyCamCount(j.n_cams);
    for (let i = 0; i < j.n_cams; i++) reloadStream(i);
  }
  const per = j.saved_per_piece || 2, pc = j.saved_pieces || 0;
  for (const c of camsInfo) {
    const lab = document.getElementById('vlabel' + c.cam);
    const err = document.getElementById('verr' + c.cam);
    const wrap = document.getElementById('vwrap' + c.cam);
    if (!lab) continue;
    // ป้ายบอกว่ากล้องนี้รับผิดชอบแผงหมายเลขไหน — ตรงกับตารางผลด้านขวา
    const from = c.cam * pc + 1, to = (c.cam + 1) * pc;
    const range = pc ? ` — แผง ${from}-${to}` : '';
    const mode = c.ref !== null ? ` · อ้างอิง b*=${c.ref}` : ' · ยังไม่ calibrate';
    lab.textContent = `กล้องที่ ${c.cam + 1}${range}${mode}`;
    err.textContent = c.cam_error ? '⚠️ ' + c.cam_error : '';
    if (wrap) wrap.className = 'vwrap' + (c.cam_error ? ' off' : '');
  }
}

async function poll() {
  const j = await (await fetch('/api/status')).json();
  renderCamStatus(j);
  // flip เป็นค่าประจำกล้อง — เอาของกล้องที่กำลังตั้งค่าอยู่มาแสดงบนปุ่ม
  const me = (j.cams || []).find(c => c.cam === activeCam);
  if (me && (flipX !== me.flip_x || flipY !== me.flip_y)) {
    flipX = me.flip_x; flipY = me.flip_y;
    updateFlipUI();
  }
  // ดึงค่าเกณฑ์จริงจากเซิร์ฟเวอร์ครั้งแรก (และถ้ามีคนแก้จากอีกแท็บ) — แต่อย่าไปทับ
  // ตอนผู้ใช้กำลังลาก slider อยู่
  if (j.thresh !== undefined && !threshBusy && thresh !== j.thresh) {
    thresh = j.thresh;
    applyThreshUI();
  }
  // เติมจำนวนแผง/ดวงที่ใช้ล่าสุดกลับเข้าฟอร์ม — ทำครั้งเดียวตอนเปิดหน้า
  // ห้ามเติมซ้ำทุกรอบ poll ไม่งั้นจะทับเลขที่ผู้ใช้กำลังพิมพ์อยู่
  if (!formFilled && j.saved_pieces) {
    formFilled = true;
    document.getElementById('pieces').value = j.saved_pieces;
    document.getElementById('per').value = j.saved_per_piece;
    document.getElementById('savedinfo').textContent =
      `(จำค่าล่าสุด: ${j.saved_pieces} ชิ้น × ${j.saved_per_piece} ดวง = ${j.saved_pieces * j.saved_per_piece} ดวง)`;
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
  const camWarn = (j.cam_errors || []).length
    ? `<div class="hint" style="color:#ffb454">⚠️ ${j.cam_errors.join('<br>⚠️ ')}</div>` : '';
  if (!j.running) {
    el.innerHTML = camWarn ||
      (j.cam_error ? '⚠️ ' + j.cam_error
                   : '📷 โหมดพรีวิว — กดเริ่มตรวจเมื่อจัดวางเสร็จ');
    return;
  }
  if (j.error) { el.innerHTML = camWarn + '⚠️ ' + j.error; return; }
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
  h += camWarn;
  if (nCams > 1) {
    // แต่ละกล้องมีค่าอ้างอิงของตัวเอง — สรุปให้เห็นว่าตัวไหนอยู่โหมดไหน
    const per = camsInfo.map(c => `กล้อง ${c.cam + 1}: `
      + (c.ref !== null ? `อ้างอิง b*=${c.ref}` : 'เทียบกันเอง')).join(' · ');
    h += `<div class="hint">🎯 NG เมื่อ "ห่าง" ${rule} (จับ${j.direction_label}) — ${per}</div>`;
  } else {
    h += j.mode === 'absolute'
      ? `<div class="hint">🎯 เทียบขาวอ้างอิง b*=${j.ref} — NG เมื่อ "ห่าง" ${rule} (จับ${j.direction_label})</div>`
      : `<div class="hint">⚠️ ยังไม่ calibrate — เทียบกันเองในกลุ่ม NG เมื่อ "ห่าง" ${rule} (แม่นน้อยกว่า)</div>`;
  }
  // relative + สองทาง ตอน 2 ดวง: dev ของสองดวงเป็นภาพสะท้อนกัน → เด้ง NG พร้อมกันเสมอ
  if (j.mode !== 'absolute' && j.direction === 'both' && j.pieces * j.per_piece === 2)
    h += `<div class="hint" style="color:#ffb454">⚠️ โหมดเทียบกันเอง + จับสองทาง ที่ 2 ดวง
          จะขึ้น NG พร้อมกันทั้งคู่เสมอ แยกไม่ออกว่าดวงไหนผิด — เลือก "ฟ้าเท่านั้น"
          หรือ calibrate ก่อน</div>`;
  h += '<table><tr>' + (nCams > 1 ? '<th>กล้อง</th>' : '')
     + '<th>ชิ้นที่</th><th>ดวง</th><th>b*</th><th>ห่าง</th><th>K</th><th>ผล</th></tr>';
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
      h += `<tr${row}>`;
      if (k === 0 && nCams > 1)
        h += `<td rowspan="${p.spots.length}" class="hint">${(p.cam ?? 0) + 1}</td>`;
      if (k === 0)
        h += `<td rowspan="${p.spots.length}" class="${pcls}">#${p.piece} ${plabel}</td>`;
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
    global EXPOSURE_INIT, THRESH, CAMERA_VIDPID, DIRECTION
    p = argparse.ArgumentParser()
    p.add_argument("--cam", type=int, default=None,
                   help="camera index ของกล้องตัวแรก "
                        "(ไม่ใส่ = หาจาก VID/PID ก่อน แล้วค่อยใช้ค่าที่จำไว้/0)")
    p.add_argument("--cams", type=int, default=None, choices=range(1, MAX_CAMS + 1),
                   help=f"ใช้กล้องกี่ตัว (1-{MAX_CAMS}) กล้องที่ 2 รับแผงชุดถัดไป; จำถาวร")
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
        SESSIONS[0].flip_x = True
    if args.flip_y:
        SESSIONS[0].flip_y = True
    if args.flip_x or args.flip_y:
        save_sessions_config()
        cfg = load_config()
    # VID/PID ที่จะใช้ผูกกล้อง: --vidpid > ค่าใน config > ค่าเริ่มต้น (C200)
    CAMERA_VIDPID = args.vidpid or cfg.get("cam_vidpid") or C200_VID_PID
    if args.vidpid:
        update_config(cam_vidpid=args.vidpid)
    # จำนวนกล้อง: --cams ชนะค่าที่จำไว้ (ต่อกล้องตัวที่สองแล้วสั่ง --cams 2 ครั้งเดียวพอ)
    if args.cams is not None and args.cams != len(SESSIONS):
        while len(SESSIONS) > args.cams:
            SESSIONS.pop()
        while len(SESSIONS) < args.cams:
            slot = len(SESSIONS)
            used = {s.cam_index for s in SESSIONS}
            free = next((i for i in range(SCAN_MAX_INDEX) if i not in used), 0)
            SESSIONS.append(Session(slot, cam_index=free))
        refs = load_references()
        for s in SESSIONS:
            s.reference = refs[s.slot] if s.slot < len(refs) else None
        save_sessions_config()
    # เลือก index ของกล้องตัวแรก: --cam (ระบุตรงๆ) > หาจาก VID/PID (Windows) > ค่าที่จำไว้
    if args.cam is not None:
        SESSIONS[0].cam_index = args.cam
        save_sessions_config()
    elif len(SESSIONS) == 1:
        # หาด้วย VID/PID เฉพาะตอนใช้กล้องเดียว — ถ้ามีสองตัวรุ่นเดียวกันจะแยกไม่ออก
        idx = find_index_by_vidpid(CAMERA_VIDPID)
        if idx is not None:
            SESSIONS[0].cam_index = idx
            print(f"พบกล้อง VID/PID {CAMERA_VIDPID} ที่ index {idx}")
            save_sessions_config()
    EXPOSURE_INIT = None if args.exposure == 0 else args.exposure
    if args.thresh is not None:        # ระบุมา → ใช้ + จำถาวร
        THRESH = args.thresh
        update_config(thresh=args.thresh)
    else:                              # ไม่ระบุ → ใช้ค่าที่จำไว้ (หรือ default เดิม 4.0)
        THRESH = cfg.get("thresh", THRESH)
    if args.direction is not None:     # ระบุมา → ใช้ + จำถาวร
        DIRECTION = args.direction
        update_config(direction=args.direction)
    else:                              # ไม่ระบุ → ค่าที่จำไว้ (None = ตามพฤติกรรมเดิม)
        saved = cfg.get("direction")
        DIRECTION = saved if saved in DIRECTIONS else None
    uvicorn.run(app, host=host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
