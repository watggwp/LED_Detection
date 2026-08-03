#!/usr/bin/env python3
"""multiwatch.py — ตรวจสีไฟ LED N จุดพร้อมกัน (ขยายจาก colorwatch.py, วางแถวแนวนอน)

หลักการ: วัด b* ทุกจุดในเฟรมเดียวกัน แล้วเทียบแต่ละจุดกับ median ของทุกจุด
จุดที่ b* ต่ำกว่า median เกิน thresh ค้างครบ hold วินาที = เพี้ยน (ฟ้า)
median = เสียงข้างมาก → ตรวจได้แม้เพี้ยนพร้อมกันหลายดวง (ตราบใดที่ไม่เกินครึ่ง)

กรณี N=2 ใช้สูตรเดิมของ colorwatch (เทียบกันตรงๆ b_i − b_อีกฝั่ง) เพื่อให้เกณฑ์
thresh 4.0 มีความหมายเท่าเดิม — เทียบ median ตอน N=2 จะไวขึ้น 2 เท่าโดยไม่ตั้งใจ

โหมด "ยึดสล็อต" (A1): พอเจอครบ N ครั้งแรก (หรือตอน calibrate) จะจำตำแหน่ง x ของ
แต่ละช่องไว้ เฟรมถัดไปจับ blob ที่เจอเข้าช่องที่ใกล้สุด → ทนต่อการที่บางดวงหาย
(วัดดวงที่เหลือได้ ไม่ error ทั้งจอ) และ map ดวง↔ช่อง↔ชิ้น ถูกเสมอ

ใช้งาน (CLI ทดสอบ):
    python multiwatch.py run --cam 0 --spots 10 --json
    python multiwatch.py run --cam 0 --spots 4 --show
"""

import argparse
import json
import statistics
import sys
import time

import cv2
import numpy as np

from colorwatch import (MIN_BLOB_AREA, HoldTimer, estimate_cct,
                        grow, measure_bstar, open_camera)

MAX_SPOTS = 10  # จำกัดต่อรอบ — แถวแนวนอนบนเฟรม 1280px เกินนี้แต่ละจุดเล็กจนวัดไม่นิ่ง

# ทิศทางการเบี่ยงสีที่ถือว่า NG (b* ติดลบ = ฟ้า, บวก = เหลือง)
#   both   = เบี่ยงทางไหนก็ NG
#   blue   = NG เฉพาะที่ฟ้าเกิน (เหลืองปล่อยผ่าน) — ใช้เมื่ออาการเสียของงานมีทางเดียว
#   yellow = NG เฉพาะที่เหลืองเกิน
DIR_BOTH, DIR_BLUE, DIR_YELLOW = "both", "blue", "yellow"
DIRECTIONS = (DIR_BOTH, DIR_BLUE, DIR_YELLOW)


def resolve_direction(direction, reference):
    """แปลง direction=None เป็นพฤติกรรมเดิมของโปรแกรม (ก่อนมีตัวเลือกนี้)

      มี reference (calibrate แล้ว) → จับสองทาง (ฟ้า+เหลือง)
      ไม่มี reference (เทียบกันเอง) → จับฟ้าทางเดียว

    คง default นี้ไว้เพื่อไม่ให้ผู้เรียกเดิม (เช่น CLI ของไฟล์นี้ ที่ไม่ส่ง reference
    และไม่ส่ง direction) เปลี่ยนพฤติกรรม — ผู้ที่อยากคุมเองส่ง direction มา แล้วค่านั้นชนะ
    กติกานี้เขียนที่นี่ที่เดียว ทั้ง engine และ webapp เรียกใช้ร่วมกัน"""
    if direction is not None:
        return direction
    return DIR_BOTH if reference is not None else DIR_BLUE

# เกณฑ์ detection แบบ relative (สัดส่วนของ gray.max())
# 0.6 (ผ่อนกว่า 0.85 เดิม) ใช้ได้ปลอดภัยเมื่อ "ล็อกแสงต่ำ → พื้นหลังดำ" (ดู A3 ใน webapp)
# พื้นดำทำให้ดวงที่หรี่กว่ายังโผล่เป็น blob โดยไม่มีแสงห้องมารวมแผงเป็นก้อนเดียว
DETECT_REL = 0.6
DETECT_FLOOR = 40

# ถ้าทุกช่อง MISSING ต่อเนื่องเกินนี้ = สล็อตที่ยึดไว้ผิด/กล้องขยับ → เคลียร์ แล้ว re-detect เอง
# (กันเคสเฟรมแรกล็อกเงาสะท้อนเป็นดวง หรือแผง/กล้องขยับ แล้วค้าง MISSING จนคนงงหน้างาน)
SLOT_RECOVER_SEC = 3.0


def detect_blobs(gray, rel=DETECT_REL, floor=DETECT_FLOOR):
    """คืน list ROI (x,y,w,h) ของ blob สว่าง 'ทุก' อันที่ผ่านเกณฑ์ เรียงซ้าย→ขวา
    (ไม่จำกัดจำนวน — ใช้สำหรับจับ blob เข้าสล็อตในแต่ละเฟรม)"""
    thr = max(int(gray.max() * rel), floor)
    _, mask = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    blobs = [tuple(int(v) for v in stats[i, :4]) for i in range(1, n)
             if stats[i, cv2.CC_STAT_AREA] >= MIN_BLOB_AREA]
    blobs.sort(key=lambda r: r[0])
    return blobs


def find_spots(gray, expected, rel=DETECT_REL, floor=DETECT_FLOOR):
    """หา blob ที่ 'น่าจะเป็นดวงไฟ' ไม่เกิน expected อัน คืน list ROI เรียงซ้าย→ขวา
    (เลือก expected อันที่ใหญ่สุด แล้วตัดคู่ที่ซ้อนกัน) — ใช้ตอนยึดสล็อต/calibrate"""
    blobs = detect_blobs(gray, rel, floor)
    blobs = sorted(blobs, key=lambda r: r[2] * r[3], reverse=True)[:expected]
    blobs.sort(key=lambda r: r[0])
    # sanity แบบเดียวกับ colorwatch: จุดข้างกัน centroid ต้องห่างแนวนอนจริง
    # ถ้าคู่ไหนซ้อน = จับก้อนรวม/แสงสะท้อน → เก็บอันที่ใหญ่กว่า
    cleaned = []
    for roi in blobs:
        if cleaned:
            (xp, _, wp, hp), (xc, _, wc, hc) = cleaned[-1], roi
            if (xc + wc / 2) - (xp + wp / 2) < max(wp, wc) * 0.6:
                if wc * hc > wp * hp:
                    cleaned[-1] = roi
                continue
        cleaned.append(roi)
    return cleaned


class MultiWatch:
    """engine ตรวจ N จุด — ป้อนเฟรมผ่าน update() ได้สถานะกลับ ไม่ผูกกับกล้อง/UI"""

    def __init__(self, n_spots, thresh=4.0, hold=5.0, smooth=15, reference=None,
                 detect_rel=DETECT_REL, direction=None):
        if not 2 <= n_spots <= MAX_SPOTS:
            raise ValueError(f"n_spots ต้องอยู่ระหว่าง 2 ถึง {MAX_SPOTS}")
        if direction is not None and direction not in DIRECTIONS:
            raise ValueError(f"direction ต้องเป็นหนึ่งใน {sorted(DIRECTIONS)}")
        self.n = n_spots
        self.thresh = thresh
        # ทิศทางที่ถือว่า NG — เปลี่ยนสดได้ (อ่านใหม่ทุกเฟรมใน update())
        # None = ตามพฤติกรรมเดิมของโปรแกรม ดู resolved_direction()
        self.direction = direction
        self.smooth = smooth
        self.detect_rel = detect_rel
        # สล็อต = ตำแหน่งช่องที่ยึดไว้หลังเจอครบ N ครั้งแรก/ตอน calibrate (len==n, ซ้าย→ขวา)
        self.slots = None
        self.slot_tol = None   # ระยะ x สูงสุดที่ blob เข้าสล็อตได้ (ครึ่งหนึ่งของช่องที่ชิดสุด)
        self.all_missing_since = None  # เวลาเริ่มที่ทุกช่อง MISSING (ใช้ตัดสิน auto re-detect)
        self.dev_hist = [[] for _ in range(n_spots)]
        self.timers = [HoldTimer(hold) for _ in range(n_spots)]
        # absolute mode: b* ขาวอ้างอิง (จาก calibrate ชิ้นดี ภายใต้แสง+WB ที่ล็อกแล้ว)
        # → NG เมื่อห่างอ้างอิงเกิน thresh ทั้งสองทิศ (ฟ้า "และ" เหลือง)
        # None = relative mode เดิม (เทียบกันเองในเฟรม — ใช้เมื่อยังไม่ calibrate)
        self.reference = reference

    # ---- สล็อต (A1) ----

    def _lock_slots(self, rois):
        """บันทึกตำแหน่งช่องจากเฟรมที่เจอครบ n — ใช้ map blob→ช่องในเฟรมถัดไป"""
        self.slots = [tuple(r) for r in rois]
        centers = sorted(x + w / 2 for (x, _, w, _) in self.slots)
        gaps = [centers[i + 1] - centers[i] for i in range(len(centers) - 1)]
        # tol = ครึ่งหนึ่งของช่องว่างที่แคบสุด → blob จะไม่ถูกแย่งไปเข้าช่องผิด
        self.slot_tol = 0.5 * min(gaps) if gaps else 1e9
        self.all_missing_since = None

    def _assign(self, blobs):
        """จับ blob เข้าสล็อตที่ใกล้สุด (blob ใหญ่ได้สิทธิ์เลือกก่อน)
        คืน list ยาว n: ROI ของ blob ที่เข้าช่องนั้น หรือ None ถ้าช่องนั้นไม่มี blob"""
        assigned = [None] * self.n
        for blob in sorted(blobs, key=lambda r: r[2] * r[3], reverse=True):
            bc = blob[0] + blob[2] / 2
            best, best_d = None, None
            for i, slot in enumerate(self.slots):
                if assigned[i] is not None:
                    continue
                sc = slot[0] + slot[2] / 2
                d = abs(bc - sc)
                if d <= self.slot_tol and (best_d is None or d < best_d):
                    best, best_d = i, d
            if best is not None:
                assigned[best] = blob
        return assigned

    def calibrate(self, frame, now=None):
        """วางชิ้นดีให้ครบ n จุดแล้วเรียก — ตั้ง reference = median b* ของชิ้นดีทั้งหมด
        และยึดสล็อตจากเฟรมนี้ คืน (ref, msg) — ref เป็น None ถ้าวัดไม่ได้"""
        now = time.time() if now is None else now
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rois = find_spots(gray, self.n, self.detect_rel)
        if len(rois) != self.n:
            return None, f"เจอ {len(rois)}/{self.n} จุด — จัดวางให้ครบก่อน calibrate"
        bs = [measure_bstar(frame, r)[0] for r in rois]
        if any(b is None for b in bs):
            return None, "pixel ไม่พอ วัดสีไม่ได้ — เช็คแสง"
        # กันพลาด: ชิ้นดีล้วนๆ ต้องสีใกล้กัน ถ้า spread เกินเกณฑ์ = มีชิ้นเพี้ยนปน
        # (เคยเกิดจริง 2026-06-12: calibrate ทั้งที่มีชิ้นฟ้าอยู่ → ref กลายเป็นค่ากลาง
        #  ระหว่างดี-เสีย แล้วทุกชิ้นผ่านหมด)
        spread = max(bs) - min(bs)
        if spread > self.thresh:
            return None, (f"สีชิ้นที่วางต่างกันเกินไป (ห่างสุด {spread:.1f} เกินเกณฑ์ "
                          f"{self.thresh:.0f}) — น่าจะมีชิ้นเพี้ยนปนอยู่ "
                          f"เอาออกให้เหลือชิ้นดีล้วนๆ แล้ว calibrate ใหม่")
        self.reference = float(statistics.median(bs))
        self._lock_slots(rois)
        self._reset_history()
        for t in self.timers:
            t.since = None
        return self.reference, f"อ้างอิงขาว b* = {self.reference:+.2f} (จาก {self.n} จุด)"

    def resolved_direction(self):
        """ทิศทางที่ใช้ตัดสินจริงในเฟรมถัดไป (แปลง None ให้แล้ว)"""
        return resolve_direction(self.direction, self.reference)

    def _is_bad(self, sdev):
        """เบี่ยงเท่านี้ถือว่า NG ไหม — ตัดสินจาก thresh + ทิศทางที่ใช้จริง

        NB: เทียบกันเอง + จับสองทาง ตอน n=2 ใช้ไม่ได้ผล เพราะ dev ของสองดวงเป็นภาพ
        สะท้อนกัน (b_ซ้าย−b_ขวา กับ b_ขวา−b_ซ้าย) → เกินเกณฑ์พร้อมกันทั้งคู่เสมอ
        แยกไม่ออกว่าดวงไหนผิด ฝั่ง webapp จึงเตือนไม่ให้ใช้คู่นี้"""
        d = self.resolved_direction()
        if d == DIR_BLUE:
            return sdev < -self.thresh
        if d == DIR_YELLOW:
            return sdev > self.thresh
        return abs(sdev) > self.thresh

    def _reset_history(self):
        """ล้างเฉพาะ rolling window — hold timer ต้องเดินต่อ ไม่งั้น re-detect
        จะทำสถานะ BAD หลุดเป็น OK เป็นจังหวะ"""
        for h in self.dev_hist:
            h.clear()

    def update(self, frame, now=None, with_cct=False):
        """ประมวลผลหนึ่งเฟรม คืน dict สถานะ
        - ก่อนยึดสล็อต (ยังไม่เคยเจอครบ): มี 'error' บอกให้จัดวางให้ครบ 1 ครั้ง
        - หลังยึดสล็อต: คืน 'spots' ครบ n เสมอ แต่ละช่อง state = OK/BAD/MISSING"""
        now = time.time() if now is None else now
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.slots is None:
            found = find_spots(gray, self.n, self.detect_rel)
            if len(found) == self.n:
                self._lock_slots(found)
            else:
                return {"t": round(now, 1),
                        "error": f"เจอ {len(found)}/{self.n} ดวง — จัดวางให้ครบ 1 ครั้งก่อน",
                        "n_found": len(found), "need": self.n}

        blobs = detect_blobs(gray, self.detect_rel)
        assigned = self._assign(blobs)

        bs = [None] * self.n
        ns = [0] * self.n
        for i, roi in enumerate(assigned):
            if roi is not None:
                bs[i], ns[i] = measure_bstar(frame, roi)

        present = [b for b in bs if b is not None]
        med = statistics.median(present) if present else None

        spots = []
        n_missing = 0
        for i in range(self.n):
            b = bs[i]
            if b is None:
                # ช่องนี้ไม่มี blob (ดวงหาย/หรี่เกิน) — อย่านับ hold ต่อ, ล้าง window
                self.timers[i].since = None
                self.dev_hist[i].clear()
                spots.append({"i": i + 1, "state": "MISSING"})
                n_missing += 1
                continue
            if self.reference is not None:
                dev = b - self.reference
            elif self.n == 2:
                other = bs[1 - i]
                # อีกช่องหาย → เทียบกันเองไม่ได้ ถือว่ายังโอเค (ไม่ฟันธงเพี้ยน)
                dev = (b - other) if other is not None else 0.0
            else:
                dev = b - med
            hist = self.dev_hist[i]
            hist.append(dev)
            if len(hist) > self.smooth:
                hist.pop(0)
            sdev = float(np.mean(hist))
            cond = self._is_bad(sdev)
            bad = self.timers[i].update(cond, now)
            spot = {"i": i + 1, "b": round(b, 2), "dev": round(sdev, 2),
                    "state": "BAD" if bad else "OK"}
            if with_cct:
                k = estimate_cct(frame, assigned[i])
                spot["cct_k"] = round(k) if k else None
            spots.append(spot)

        # ทุกช่อง MISSING นานเกิน → สล็อตที่ยึดไว้น่าจะผิด/กล้องขยับ → เคลียร์ให้ re-detect เอง
        # (เฟรมถัดไปจะ find_spots ใหม่: เจอครบ=ยึดสล็อตใหม่ / ไม่ครบ=ขึ้น "จัดวางให้ครบ")
        if n_missing == self.n:
            if self.all_missing_since is None:
                self.all_missing_since = now
            elif now - self.all_missing_since >= SLOT_RECOVER_SEC:
                self.slots = None
                self.slot_tol = None
                self.all_missing_since = None
        else:
            self.all_missing_since = None

        out = {"t": round(now, 1), "spots": spots,
               "n_found": self.n - n_missing, "n_missing": n_missing,
               "mode": "absolute" if self.reference is not None else "relative"}
        if self.reference is not None:
            out["ref"] = round(self.reference, 2)
        return out

    def annotate(self, frame, status):
        """วาดกรอบ+ค่าลงเฟรม (in-place) ใช้ได้ทั้ง --show และ web stream
        ช่องที่ MISSING วาดกรอบเทาที่ตำแหน่งสล็อตเดิม + "?" (ไม่หายไปจากภาพ)"""
        if self.slots is None or "spots" not in status:
            cv2.putText(frame, status.get("error", "..."), (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            return frame
        for slot, spot in zip(self.slots, status["spots"]):
            x0, y0, x1, y1 = grow(slot, frame.shape)
            st = spot["state"]
            if st == "MISSING":
                color = (140, 140, 140)
                label = f"#{spot['i']} ?"
            else:
                bad = st == "BAD"
                color = (0, 0, 255) if bad else (0, 255, 0)
                label = f"#{spot['i']} {'NG' if bad else 'OK'} {spot['b']:+.1f}"
            cv2.rectangle(frame, (x0, y0), (x1, y1), color, 2)
            cv2.putText(frame, label, (x0, max(20, y0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return frame


def cmd_run(args):
    watch = MultiWatch(args.spots, thresh=args.thresh, hold=args.hold,
                       smooth=args.smooth, detect_rel=args.rel)
    cap = open_camera(args.cam)
    last_report = 0.0
    print(f"เริ่มจับภาพ {args.spots} จุด... (Ctrl+C เพื่อหยุด)", file=sys.stderr)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            if args.flip_x and args.flip_y:
                frame = cv2.flip(frame, -1)
            elif args.flip_x:
                frame = cv2.flip(frame, 1)
            elif args.flip_y:
                frame = cv2.flip(frame, 0)
            now = time.time()
            status = watch.update(frame, now, with_cct=(now - last_report >= args.interval))
            if args.show:
                watch.annotate(frame, status)
                cv2.imshow("multiwatch (q=quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if now - last_report >= args.interval:
                last_report = now
                if args.json:
                    print(json.dumps(status, ensure_ascii=False), flush=True)
                elif "error" in status:
                    print(f"[--] {status['error']}", flush=True)
                else:
                    parts = []
                    for s in status["spots"]:
                        if s["state"] == "MISSING":
                            parts.append(f"#{s['i']}=ไม่พบ")
                        else:
                            parts.append(f"#{s['i']}={s['b']:+.1f}({s['state']})")
                    bad = [s["i"] for s in status["spots"] if s["state"] == "BAD"]
                    miss = [s["i"] for s in status["spots"] if s["state"] == "MISSING"]
                    verdict = "ปกติทั้งหมด" if not bad else f"เพี้ยน: จุด {bad}"
                    if miss:
                        verdict += f" / ไม่พบจุด {miss}"
                    print("  ".join(parts) + f"  → {verdict}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main():
    p = argparse.ArgumentParser(description="ตรวจสีไฟ LED N จุด (แถวแนวนอน)")
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="โหมดวัดต่อเนื่อง")
    r.add_argument("--cam", type=int, default=0)
    r.add_argument("--spots", type=int, required=True,
                   help=f"จำนวนจุดไฟทั้งหมด (2-{MAX_SPOTS})")
    r.add_argument("--thresh", type=float, default=4.0)
    r.add_argument("--hold", type=float, default=5.0)
    r.add_argument("--smooth", type=int, default=15)
    r.add_argument("--rel", type=float, default=DETECT_REL,
                   help=f"เกณฑ์ detection สัดส่วนของความสว่างสูงสุด (default {DETECT_REL})")
    r.add_argument("--interval", type=float, default=1.0)
    r.add_argument("--flip-x", action="store_true", help="กลับภาพแนวนอน (ซ้าย-ขวา)")
    r.add_argument("--flip-y", action="store_true", help="กลับภาพแนวตั้ง (บน-ล่าง)")
    r.add_argument("--json", action="store_true")
    r.add_argument("--show", action="store_true")
    r.set_defaults(func=cmd_run)
    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
