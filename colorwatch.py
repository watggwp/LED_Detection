#!/usr/bin/env python3
"""
colorwatch.py — เทียบสีไฟ 2 จุด (ซ้าย/ขวา) จากกล้อง แล้วบอกว่าฝั่งไหน "เพี้ยน" (ฟ้า)

หลักการ: ไม่ต้อง calibrate — วัด b* (Lab) ของจุดซ้ายและขวาในเฟรมเดียวกัน
แล้วดู diff ระหว่างกัน ฝั่งที่ b* ต่ำกว่าชัดเจน = ฟ้ากว่า = ผิดปกติ
กล้องจะ auto-exposure/WB ยังไง สองจุดโดนพร้อมกัน ค่า diff จึงนิ่ง

ใช้งาน:
    python colorwatch.py run                 # โหมดหลัก พ่นผลทุกวินาที
    python colorwatch.py run --json          # output เป็น JSON lines
    python colorwatch.py run --show          # โชว์หน้าต่าง debug (วาด ROI + ค่า)
    python colorwatch.py run --flip          # สลับ label ซ้าย/ขวา (ถ้ากล้อง mirror)
    python colorwatch.py run --cam 1         # เลือกกล้อง (ลอง 0,1,2 จนเจอ Anker)

ออปชันสำคัญ:
    --thresh 4.0    ส่วนต่าง b* ที่ถือว่า "ต่างกันจริง" (หน่วยเดียวกับ db เดิม)
    --hold 5.0      ต้องค้างเกินกี่วินาทีถึงฟันธง (กัน noise/แสงแกว่ง)

deps: pip install opencv-python numpy
"""

import argparse
import json
import sys
import time

import cv2
import numpy as np

SAT_LIMIT = 250      # pixel ที่ channel ใดแตะค่านี้ = อิ่มตัว (ขาวโพลน) วัดสีไม่ได้
MIN_BLOB_AREA = 200  # ขนาด blob ขั้นต่ำ (px) กันจับฝุ่น/แสงสะท้อนเล็กๆ


def find_two_spots(gray):
    """หา blob สว่างสุด 2 อัน คืน [(x,y,w,h), ...] เรียงซ้าย→ขวา หรือ None"""
    # threshold แบบ relative: เอาส่วนที่สว่างกว่า 85% ของ max
    # (0.6 เดิมทำให้ทั้งแผง diffuser รวมเป็น blob เดียวตอนห้องสว่าง — ทดสอบ 2026-06-12)
    thr = max(int(gray.max() * 0.85), 40)
    _, mask = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)
    )
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    blobs = [
        stats[i] for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= MIN_BLOB_AREA
    ]
    if len(blobs) < 2:
        return None
    blobs.sort(key=lambda s: s[cv2.CC_STAT_AREA], reverse=True)
    a, b = blobs[0], blobs[1]
    rois = sorted(
        [tuple(s[:4]) for s in (a, b)], key=lambda r: r[0]
    )  # sort ตาม x
    # sanity: centroid ต้องห่างกันแนวนอนจริง — กันเคสจับแผงรวมก้อนเดียว
    # + แสงสะท้อนเล็กๆ แล้วได้คู่ ROI ผิด (เคยเกิดตอน threshold 0.6)
    (xl, _, wl, _), (xr, _, wr, _) = rois
    if (xr + wr / 2) - (xl + wl / 2) < max(wl, wr) * 0.6:
        return None
    return rois  # [left, right]


def grow(roi, frame_shape, pad=0.35):
    """ขยาย ROI ออกไปรอบๆ เพื่อเก็บ halo (วงแหวนแสงรอบ core)"""
    x, y, w, h = roi
    H, W = frame_shape[:2]
    dx, dy = int(w * pad), int(h * pad)
    x0, y0 = max(0, x - dx), max(0, y - dy)
    x1, y1 = min(W, x + w + dx), min(H, y + h + dy)
    return x0, y0, x1, y1


def measure_bstar(frame_bgr, roi):
    """วัด mean b* ใน ROI โดยใช้เฉพาะ pixel สว่างแต่ไม่อิ่มตัว คืน (b*, n_pixels)"""
    x0, y0, x1, y1 = grow(roi, frame_bgr.shape)
    patch = frame_bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return None, 0
    maxc = patch.max(axis=2)
    # เอา pixel ที่สว่างพอ (มีแสงไฟจริง) แต่ยังไม่ clip
    # จำกัดเป็นวงรี inscribed ในกรอบ — ตัดมุมกรอบที่อาจกินขอบแผง/วัตถุข้างเคียง
    # (พิสูจน์กับเฟรมจริง: มุมกรอบขวาดึง b* ขึ้น ~1 หน่วย แม้ diff กระทบแค่ ~0.3)
    ph, pw = patch.shape[:2]
    ell = np.zeros((ph, pw), np.uint8)
    cv2.ellipse(ell, ((pw // 2, ph // 2), (pw, ph), 0), 1, -1)
    inside = ell.astype(bool)
    usable = (maxc >= 80) & (maxc < SAT_LIMIT) & inside
    n = int(usable.sum())
    if n < 50:
        # แสงน้อยมาก: ผ่อนเกณฑ์ความสว่างลง (วงรียังต้องคงไว้ — เป็นเรื่องเรขาคณิต
        # กันขอบแผงปน ไม่เกี่ยวกับความสว่าง)
        usable = (maxc >= 40) & (maxc < SAT_LIMIT) & inside
        n = int(usable.sum())
        if n < 50:
            return None, n
    lab = cv2.cvtColor(patch, cv2.COLOR_BGR2Lab)
    b_star = float(lab[..., 2][usable].mean()) - 128.0  # OpenCV เก็บ b* แบบ +128
    return b_star, n


def estimate_cct(frame_bgr, roi):
    """ประมาณอุณหภูมิสี (เคลวิน, สูตร McCamy) ใน ROI — ข้อมูลประกอบเท่านั้น
    ไม่ใช้ตัดสินเพี้ยน/ปกติ (b* นิ่งกว่า ~4 เท่า: SNR 265 vs 61, ทดสอบ 2026-06-12)"""
    x0, y0, x1, y1 = grow(roi, frame_bgr.shape)
    patch = frame_bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return None
    maxc = patch.max(axis=2)
    usable = (maxc >= 80) & (maxc < SAT_LIMIT)
    ph, pw = patch.shape[:2]
    ell = np.zeros((ph, pw), np.uint8)
    cv2.ellipse(ell, ((pw // 2, ph // 2), (pw, ph), 0), 1, -1)
    usable &= ell.astype(bool)
    if usable.sum() < 50:
        return None
    rgb = patch[..., ::-1].astype(np.float64) / 255.0
    lin = np.where(rgb <= 0.04045, rgb / 12.92, ((rgb + 0.055) / 1.055) ** 2.4)
    M = np.array([[0.4124, 0.3576, 0.1805],
                  [0.2126, 0.7152, 0.0722],
                  [0.0193, 0.1192, 0.9505]])
    XYZ = lin[usable] @ M.T
    S = XYZ.sum(axis=1)
    ok = S > 1e-6
    if not ok.any():
        return None
    x = XYZ[ok, 0] / S[ok]
    y = XYZ[ok, 1] / S[ok]
    n = (x - 0.3320) / (0.1858 - y)
    return float(np.median(449 * n**3 + 3525 * n**2 + 6823.3 * n + 5520.33))


class HoldTimer:
    """เงื่อนไขต้องเป็นจริงต่อเนื่องเกิน hold วินาที ถึงจะ trigger"""

    def __init__(self, hold_sec):
        self.hold = hold_sec
        self.since = None

    def update(self, condition, now):
        if not condition:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return (now - self.since) >= self.hold


def open_camera(index):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        sys.exit(f"เปิดกล้อง index {index} ไม่ได้ — ลอง --cam 1 หรือ 2")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap


def cmd_run(args):
    cap = open_camera(args.cam)
    rois = None
    roi_age = 0.0
    diffs = []  # rolling window ของค่า diff
    left_bad = HoldTimer(args.hold)
    right_bad = HoldTimer(args.hold)
    last_report = 0.0

    print("เริ่มจับภาพ... (Ctrl+C เพื่อหยุด)", file=sys.stderr)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            now = time.time()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # re-detect ROI ทุก 10 วิ หรือยังไม่เคยเจอ
            if rois is None or (now - roi_age) > 10.0:
                found = find_two_spots(gray)
                if found:
                    if rois is not None and tuple(map(tuple, found)) != tuple(map(tuple, rois)):
                        diffs.clear()  # ROI ขยับ — ค่าเก่าวัดจากตำแหน่งเดิม อย่าปน
                    rois = found
                    roi_age = now

            if rois is None:
                if args.show:
                    cv2.putText(frame, "NO SPOTS FOUND", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                    cv2.imshow("colorwatch (q=quit)", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                if now - last_report >= 1.0:
                    last_report = now
                    msg = {"t": round(now, 1), "error": "หาไฟ 2 จุดไม่เจอ"}
                    print(json.dumps(msg, ensure_ascii=False) if args.json
                          else "[--] ยังหาไฟ 2 จุดไม่เจอ", flush=True)
                continue

            bl, nl = measure_bstar(frame, rois[0])
            br, nr = measure_bstar(frame, rois[1])
            if bl is None or br is None:
                # วัดไม่ได้ (pixel ใช้ได้ไม่พอ เช่น มืดเกิน) — ต้องบอก ไม่ใช่เงียบ
                # และอย่านับเวลา hold ต่อช่วงที่ไม่มีข้อมูล
                left_bad.since = None
                right_bad.since = None
                if args.show:
                    cv2.putText(frame, "MEASURE FAILED (px not enough)",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                                (0, 0, 255), 2)
                    cv2.imshow("colorwatch (q=quit)", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                if now - last_report >= args.interval:
                    last_report = now
                    msg = {"t": round(now, 1), "error": "pixel ไม่พอ วัดค่าไม่ได้",
                           "n_left": nl, "n_right": nr}
                    print(json.dumps(msg, ensure_ascii=False) if args.json
                          else f"[--] วัดค่าไม่ได้ (pixel ไม่พอ: L={nl}, R={nr})",
                          flush=True)
                continue

            if args.flip:
                bl, br = br, bl

            diffs.append(bl - br)
            if len(diffs) > args.smooth:
                diffs.pop(0)
            diff = float(np.mean(diffs))  # ซ้าย - ขวา (ติดลบ = ซ้ายฟ้ากว่า)

            L = left_bad.update(diff < -args.thresh, now)
            R = right_bad.update(diff > args.thresh, now)

            if args.show:
                draw_debug(frame, rois, bl, br, diff, L, R, args.flip)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            if now - last_report >= args.interval:
                last_report = now
                # CCT คำนวณเฉพาะตอนรายงาน (1/วินาที) — ข้อมูลประกอบ ไม่คุ้มคำนวณทุกเฟรม
                kl = estimate_cct(frame, rois[0])
                kr = estimate_cct(frame, rois[1])
                if args.flip:
                    kl, kr = kr, kl
                report(now, bl, br, kl, kr, diff, L, R, args)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


def report(now, bl, br, kl, kr, diff, L, R, args):
    state_l = "BAD" if L else "OK"
    state_r = "BAD" if R else "OK"
    if args.json:
        print(json.dumps({
            "t": round(now, 1),
            "left":  {"b": round(bl, 2), "state": state_l,
                      "cct_k": round(kl) if kl else None},
            "right": {"b": round(br, 2), "state": state_r,
                      "cct_k": round(kr) if kr else None},
            "diff": round(diff, 2),
        }, ensure_ascii=False), flush=True)
    else:
        verdict = ("ซ้ายเพี้ยน (ฟ้า)" if L else
                   "ขวาเพี้ยน (ฟ้า)" if R else "ปกติทั้งคู่")
        kls = f"~{kl:.0f}K" if kl else "~?K"
        krs = f"~{kr:.0f}K" if kr else "~?K"
        print(f"L={bl:+6.2f} {kls} ({state_l})  R={br:+6.2f} {krs} ({state_r})  "
              f"diff={diff:+6.2f}  → {verdict}", flush=True)


def draw_debug(frame, rois, bl, br, diff, L, R, flipped):
    vals = [(bl, L), (br, R)]
    if flipped:
        vals = vals[::-1]
    for roi, (b, bad) in zip(rois, vals):
        x0, y0, x1, y1 = grow(roi, frame.shape)
        color = (0, 0, 255) if bad else (0, 255, 0)
        cv2.rectangle(frame, (x0, y0), (x1, y1), color, 2)
        cv2.putText(frame, f"b*={b:+.1f}", (x0, max(20, y0 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(frame, f"diff={diff:+.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv2.imshow("colorwatch (q=quit)", frame)


def cmd_list(args):
    """ไล่เช็คว่ามีกล้อง index ไหนเปิดได้บ้าง"""
    for i in range(6):
        cap = cv2.VideoCapture(i)
        ok = cap.isOpened() and cap.read()[0]
        cap.release()
        if ok:
            print(f"camera index {i}: ใช้ได้")


def main():
    p = argparse.ArgumentParser(description="แยกสีไฟซ้าย/ขวา ว่าฝั่งไหนเพี้ยน")
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="โหมดวัดต่อเนื่อง")
    r.add_argument("--cam", type=int, default=0, help="camera index (default 0)")
    r.add_argument("--thresh", type=float, default=4.0,
                   help="ส่วนต่าง b* ที่ถือว่าต่างจริง (default 4.0)")
    r.add_argument("--hold", type=float, default=5.0,
                   help="ต้องค้างกี่วินาทีถึงฟันธง (default 5)")
    r.add_argument("--smooth", type=int, default=15,
                   help="rolling average กี่เฟรม (default 15)")
    r.add_argument("--interval", type=float, default=1.0,
                   help="รายงานทุกกี่วินาที (default 1)")
    r.add_argument("--flip", action="store_true", help="สลับ label ซ้าย/ขวา")
    r.add_argument("--json", action="store_true", help="output เป็น JSON lines")
    r.add_argument("--show", action="store_true", help="โชว์หน้าต่าง debug")
    r.set_defaults(func=cmd_run)

    ls = sub.add_parser("list", help="หา camera index ที่ใช้ได้")
    ls.set_defaults(func=cmd_list)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
