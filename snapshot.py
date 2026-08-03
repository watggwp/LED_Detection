#!/usr/bin/env python3
"""snapshot.py — จับภาพนิ่งจากกล้องแต่ละ index วาดกรอบ ROI + ค่า b* แล้วเซฟเป็น PNG

ใช้ตอน debug/เช็ค ROI โดยไม่ต้องเปิดหน้าต่าง --show:
    python snapshot.py            # จับทุก index 0-2 → snap_cam<N>.png
    python snapshot.py --cam 1    # จับเฉพาะ index 1
    python snapshot.py --warmup 10  # ทิ้งกี่เฟรมแรกให้ auto-exposure นิ่งก่อน
"""

import argparse
import sys

import cv2

from colorwatch import CAP_BACKEND, find_two_spots, grow, measure_bstar


def snap(index, warmup):
    cap = cv2.VideoCapture(index, CAP_BACKEND)
    if not cap.isOpened():
        print(f"cam {index}: เปิดไม่ได้", file=sys.stderr)
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    frame = None
    for _ in range(warmup):
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    if frame is None:
        print(f"cam {index}: อ่านเฟรมไม่ได้", file=sys.stderr)
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rois = find_two_spots(gray)
    note = ""
    if rois:
        for side, roi in zip(("L", "R"), rois):
            b, n = measure_bstar(frame, roi)
            x0, y0, x1, y1 = grow(roi, frame.shape)
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
            label = f"{side} b*={b:+.1f} n={n}" if b is not None else f"{side} (px ไม่พอ n={n})"
            cv2.putText(frame, label, (x0, max(20, y0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        note = "found 2 spots"
    else:
        note = "NO SPOTS"
        cv2.putText(frame, "NO SPOTS FOUND", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    cv2.putText(frame, f"cam {index}  gray max={gray.max()}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    out = f"snap_cam{index}.png"
    cv2.imwrite(out, frame)
    print(f"cam {index}: {note}  gray_max={gray.max()}  -> {out}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cam", type=int, default=None, help="จับเฉพาะ index นี้")
    p.add_argument("--warmup", type=int, default=15,
                   help="จำนวนเฟรมที่อ่านทิ้งให้ auto-exposure นิ่ง (default 15)")
    args = p.parse_args()
    indices = [args.cam] if args.cam is not None else [0, 1, 2]
    for i in indices:
        snap(i, args.warmup)


if __name__ == "__main__":
    main()
