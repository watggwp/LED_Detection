# 🔍 ตรวจสีไฟ LED (LED colorwatch)

[![Based On](https://img.shields.io/badge/Based%20On-pitchakorn--pkt%2Fled--colorwatch-181717?style=for-the-badge&logo=github&logoColor=white)](https://github.com/pitchakorn-pkt/led-colorwatch)
[![Python](https://img.shields.io/badge/PYTHON-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![FastAPI](https://img.shields.io/badge/FASTAPI-009688?style=for-the-badge&logo=fastapi&logoColor=white)](https://fastapi.tiangolo.com/)
[![OpenCV](https://img.shields.io/badge/OPENCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org/)
[![NumPy](https://img.shields.io/badge/NUMPY-013243?style=for-the-badge&logo=numpy&logoColor=white)](https://numpy.org/)

> 📌 **พัฒนาต่อยอดมาจาก:** [pitchakorn-pkt/led-colorwatch](https://github.com/pitchakorn-pkt/led-colorwatch)  
> รองรับ **Windows / macOS / Linux** • ใช้กล้อง USB (เทสกับ Anker PowerConf C200)

โปรแกรมตรวจว่าไฟ LED ดวงไหน **"เพี้ยน (ออกฟ้า/เหลือง)"** ผ่านกล้อง — ดูผลบนหน้าเว็บ  
เขียว = ผ่าน / แดง = ไม่ผ่าน (NG) บอกชิ้น+ทิศที่เพี้ยน **ไม่ต้องพิมพ์คำสั่งตอนใช้งาน คลิกบนเว็บอย่างเดียว**

---

## 🟦 ติดตั้งบน Windows (ทำครั้งเดียว)

**1. ติดตั้ง Python** — โหลดจาก [python.org/downloads](https://www.python.org/downloads/) (3.10 ขึ้นไป)
> ⚠️ ตอนติดตั้ง **ติ๊กถูก "Add Python to PATH"** ด้วย (สำคัญมาก)

**2. โหลดโปรแกรมนี้** — กดปุ่มเขียว **Code ▾ → Download ZIP** ด้านบนของหน้านี้ แล้วแตกไฟล์

**3. เปิด Command Prompt** ในโฟลเดอร์ที่แตกไว้ (คลิกช่อง address ของ File Explorer พิมพ์ `cmd` กด Enter) แล้วพิมพ์:
```bat
py -m venv .venv
.venv\Scripts\pip install -r requirements.txt
```
รอจนติดตั้งเสร็จ (ครั้งแรกใช้เวลาสักครู่)

**▶️ เปิดโปรแกรม:** ดับเบิลคลิกไฟล์ **`run_windows.bat`** — เบราว์เซอร์จะเปิดหน้าโปรแกรมให้เอง
(หรือพิมพ์ `.venv\Scripts\python webapp.py` ใน Command Prompt)

---

## 🍎 ติดตั้งบน macOS / Linux (ทำครั้งเดียว)
```bash
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt
```
**▶️ เปิดโปรแกรม:**
```bash
./.venv/bin/python webapp.py
```
แล้วเปิดเบราว์เซอร์ไปที่ **http://localhost:8000**

---

## 📋 วิธีใช้งาน (บนหน้าเว็บ — ง่ายมาก)

| ขั้น | ทำอะไร |
|---|---|
| 1️⃣ | กด **📷 เลือกกล้อง** → คลิกภาพที่**เห็นไฟ LED** (ไม่ต้องรู้ว่ากล้องตัวไหน) |
| 2️⃣ | ดูภาพ — ควรเห็นแต่ดวงไฟบนพื้นมืด ถ้าสว่างฟุ้ง กด **ล็อกแสง** |
| 3️⃣ | วางชิ้นงานเรียงแถวแนวนอนให้เข้าเฟรม → กรอกจำนวนชิ้น/ดวง → **▶ เริ่มตรวจ** |
| 4️⃣ | อ่านผลขวามือ: **✅ OK = ผ่าน / ❌ NG = เพี้ยน** (บอกชิ้น + ฟ้า/เหลือง) |

### 🎯 โหมดแม่นยำ (Calibrate ขาว) — แนะนำ
จับได้ทั้งฟ้า**และ**เหลือง แม่นกว่า — ทำตามลำดับนี้:
1. กด **ล็อกแสง** ก่อน แล้ว **อย่าเปลี่ยนค่าแสงอีก**
2. วาง **ชิ้นดีล้วนๆ** ให้ครบทุกจุด → กด **▶ เริ่มตรวจ**
3. กด **🎯 Calibrate ขาว** (ระบบจำ "ขาวที่ถูกต้อง" ไว้ถาวร)
4. ค่อยสลับชิ้นที่จะเทสเข้ามา → ชิ้นที่สีเพี้ยนเกินเกณฑ์จะขึ้น NG ภายใน ~5 วินาที

> ดวงที่หากล้องไม่เจอจะขึ้น **"ไม่พบ"** (เทาๆ) — ดวงอื่นยังตรวจได้ปกติ ไม่พังทั้งจอ

---

## 🎚️ ปรับความไว (ค่า thresh) — จุดเดียวที่มักต้องจูน

`thresh` = "ต่างจากขาวเกินเท่าไหร่ถึงนับว่าเพี้ยน" — **เลขน้อย = ไวขึ้น (จับฟ้าจางได้) / เลขมาก = เข้มงวดน้อยลง**

- ค่าเริ่มต้นในโค้ด = 4.0 / ที่จูนใช้งานจริงแล้วดี = **2.5**
- เปลี่ยนแล้ว**จำถาวร**: เปิดด้วยคำสั่งนี้ครั้งเดียว แล้วครั้งต่อไปไม่ต้องใส่อีก
  - Windows: `.venv\Scripts\python webapp.py --thresh 2.5`
  - mac/Linux: `./.venv/bin/python webapp.py --thresh 2.5`

**จูนยังไง:** ถ้าชิ้นเสียจริง**หลุด** (ขึ้น OK) → ลดเลขลง / ถ้าชิ้นดี**โดนฟ้องมั่ว** (ขึ้น NG) → เพิ่มเลขขึ้น
ลองทีละ 0.5 จนแยกดี-เสียได้ชัด

---

## 🛠️ แก้ปัญหาที่เจอบ่อย

| อาการ | วิธีแก้ |
|---|---|
| ภาพสว่างฟุ้งขาว แยกดวงไม่ออก | กด **ล็อกแสง** (ค่าต่ำ ~5) — **บน Windows ถ้าล็อกไม่ได้ ให้จัดห้องให้มืดลง** ให้เห็นแต่ดวงไฟ |
| หากล้องไม่เจอ / index สลับ | กด **📷 เลือกกล้อง** แล้วคลิกตัวที่เห็น LED (ระบบจำให้) |
| ชิ้นเสียหลุด / ชิ้นดีโดนฟ้อง | ปรับ `--thresh` (ดูหัวข้อด้านบน) |
| อยากให้มือถือ/เครื่องอื่นใน Wi‑Fi เปิดดู | เปิดด้วย `--host 0.0.0.0` แล้วเข้าจากมือถือที่ `http://<ไอพีเครื่องนี้>:8000` |
| ปิดโปรแกรม | กด **`Ctrl+C`** ในหน้าต่าง Terminal/Command Prompt (กล้องจะถูกปล่อย) |

> **หมายเหตุเรื่องล็อกแสงตาม OS:** macOS ใช้ `tools/uvc-util` (มากับโปรแกรม) เสถียรสุดกับ Anker C200 •
> Windows/Linux ล็อกแสงผ่าน OpenCV แบบ best-effort — กล้องบางรุ่นตั้งไม่ได้ ให้ใช้ auto + จัดห้องมืดแทน

---

## 📄 ไฟล์สำคัญ
- `webapp.py` — โปรแกรมหลัก (หน้าเว็บ)
- `multiwatch.py` / `colorwatch.py` — เครื่องมือตรวจ (engine)
- `README_USE.md` — คู่มือฉบับละเอียด
- `requirements.txt` — รายการ library ที่ต้องติดตั้ง
- `run_windows.bat` — ดับเบิลคลิกเปิดบน Windows

*สร้างค่าที่จำไว้ (`config.json`, `calibration.json`) จะถูกสร้างตอนใช้งานจริง — เป็นค่าเฉพาะเครื่องนั้นๆ*
