#!/usr/bin/env python3
import argparse
import cv2
import numpy as np
import sys
import os
import select
import time
from st7789 import ST7789
from PIL import Image, ImageDraw, ImageFont

# single-font alias to avoid accidental token corruption
FONT = cv2.FONT_HERSHEY_SIMPLEX


def center_x(display, w):
    return max(0, (display.width - w) // 2)


def show_loading(disp):
    try:
        # Prepare font
        try:
            font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 18)
        except Exception:
            font = ImageFont.load_default()

        title = 'WIPERITE Ver.003'

        # Start with a black frame
        frame = Image.new('RGB', (disp.width, disp.height), (0, 0, 0))
        draw = ImageDraw.Draw(frame)

        # Loading bar frame (draw once)
        bar_w = disp.width - 40
        bar_h = 18
        bar_x = center_x(disp, bar_w)
        bar_y = disp.height // 2 - bar_h
        draw.rectangle((bar_x-1, bar_y-1, bar_x+bar_w+1, bar_y+bar_h+1), outline=(255,255,255))

        steps = bar_w
        for i in range(steps+1):
            draw.rectangle((bar_x, bar_y, bar_x+bar_w-1, bar_y+bar_h-1), fill=(0,0,0))
            if i > 0:
                draw.rectangle((bar_x, bar_y, bar_x+i-1, bar_y+bar_h-1), fill=(0,0,255))

            np_frame = np.array(frame.rotate(180), dtype=np.uint8)[:, :, ::-1]
            disp.draw_frame(np_frame)
            time.sleep(0.001)

        # Clear the screen (remove loading bar) before drawing the title
        draw.rectangle((0, 0, disp.width, disp.height), fill=(0, 0, 0))
        np_frame = np.array(frame.rotate(180), dtype=np.uint8)[:, :, ::-1]
        disp.draw_frame(np_frame)
        time.sleep(0.05)

        # Draw title
        txt_w, txt_h = draw.textsize(title, font=font)
        txt_x = center_x(disp, txt_w)
        txt_y = bar_y + bar_h + 8
        draw.rectangle((txt_x-2, txt_y-2, txt_x+txt_w+2, txt_y+txt_h+2), fill=(0,0,0))
        draw.text((txt_x, txt_y), title, font=font, fill=(255,255,255))
        np_frame = np.array(frame.rotate(180), dtype=np.uint8)[:, :, ::-1]
        disp.draw_frame(np_frame)
        # Hold the title on-screen for a few seconds so it's readable
        time.sleep(2.0)
    except Exception:
        # If PIL or display fails, ignore and continue
        return


def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def detect_boards(frame, min_area=5000):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    edged = cv2.Canny(blurred, 50, 150)
    res = cv2.findContours(edged, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # OpenCV 2/4: returns (contours, hierarchy). OpenCV 3: returns (image, contours, hierarchy).
    if len(res) == 2:
        cnts = res[0]
    else:
        cnts = res[1]
    boards = []
    h, w = frame.shape[:2]
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            area = cv2.contourArea(approx)
            if area < min_area:
                continue
            pts = approx.reshape(4, 2).astype('float32')
            rect = order_points(pts)
            boards.append((rect, int(area)))
    boards.sort(key=lambda x: x[1], reverse=True)
    return boards

def warp_board(frame, rect, dst_size=(600,400)):
    (tl, tr, br, bl) = rect
    widthA = np.linalg.norm(br - bl)
    widthB = np.linalg.norm(tr - tl)
    maxW = max(int(widthA), int(widthB), 1)
    heightA = np.linalg.norm(tr - br)
    heightB = np.linalg.norm(tl - bl)
    maxH = max(int(heightA), int(heightB), 1)
    dst = np.array([[0,0],[maxW-1,0],[maxW-1,maxH-1],[0,maxH-1]], dtype='float32')
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(frame, M, (maxW, maxH))
    return warped

def uyvy_to_bgr(data, width, height):
    # data: raw UYVY bytes (2 bytes per pixel)
    arr = np.frombuffer(data, dtype=np.uint8)
    try:
        arr = arr.reshape((height, width, 2))
    except Exception:
        return None
    # OpenCV supports conversion from UYVY to BGR
    bgr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_UYVY)
    return bgr

def color_mask(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if color == 'red':
        lower1 = np.array([0,120,70]); upper1 = np.array([10,255,255])
        lower2 = np.array([170,120,70]); upper2 = np.array([180,255,255])
        m1 = cv2.inRange(hsv, lower1, upper1)
        m2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(m1, m2)
    elif color == 'green':
        lower = np.array([40,40,40]); upper = np.array([80,255,255])
        mask = cv2.inRange(hsv, lower, upper)
    elif color == 'blue':
        lower = np.array([90,50,50]); upper = np.array([140,255,255])
        mask = cv2.inRange(hsv, lower, upper)
    else:
        mask = None
    if mask is not None:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
    return mask

def detect_marker(frame, color):
    mask = color_mask(frame, color)
    if mask is None:
        return None, None
    res = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(res) == 2:
        cnts = res[0]
    else:
        cnts = res[1]
    if not cnts:
        return None, mask
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < 100:
        return None, mask
    M = cv2.moments(c)
    if M['m00'] == 0:
        return None, mask
    cx = int(M['m10']/M['m00']); cy = int(M['m01']/M['m00'])
    return (cx, cy), mask

def point_in_quad(pt, quad):
    poly = np.array(quad, dtype=np.int32)
    return cv2.pointPolygonTest(poly, (int(pt[0]), int(pt[1])), False) >= 0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default=0, help='camera index or device path (default=0)')
    ap.add_argument('--raw-device', help='path to raw UYVY device (e.g. /dev/xillybus_read_32)')
    ap.add_argument('--width', type=int, default=640, help='frame width for raw device')
    ap.add_argument('--height', type=int, default=480, help='frame height for raw device')
    ap.add_argument('--min-area', type=int, default=5000)
    ap.add_argument('--color', choices=['red','green','blue'], default='red')
    ap.add_argument('--no-display', action='store_true')
    ap.add_argument('--debug', action='store_true', help='Enable debug logging with timestamps')
    ap.add_argument('--rotation', type=int, choices=[0,90,180,270], default=0,
                    help='Rotate output to display by given degrees')
    ap.add_argument('--lcd-order', choices=['BGR','BRG'], default='BGR',
                    help='Pixel channel order expected by the LCD (default=BGR)')
    args = ap.parse_args()

    # Initialize display (framebuffer-backed ST7789) unless disabled
    disp = None
    if not args.no_display:
        try:
            disp = ST7789()
        except Exception as e:
            print('Warning: display init failed:', e)
    # Show startup loading/banner on the ST7789 if available
    if disp is not None:
        try:
            show_loading(disp)
        except Exception:
            pass

    cap = None
    raw_f = None
    raw_fd = None
    frame_size = None
    if args.raw_device:
        if not os.path.exists(args.raw_device):
            print('Raw device not found:', args.raw_device)
            sys.exit(1)
        # open raw device non-blocking and use select to avoid hangs
        try:
            raw_fd = os.open(args.raw_device, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            raw_fd = os.open(args.raw_device, os.O_RDONLY)
        frame_size = int(args.width) * int(args.height) * 2
    else:
        try:
            cam_index = int(args.camera)
            cap = cv2.VideoCapture(cam_index)
        except Exception:
            cap = cv2.VideoCapture(args.camera)

        if not cap.isOpened():
            print('Failed to open camera:', args.camera)
            sys.exit(1)

    try:
        stall_count = 0
        stall_threshold = 5

        def drain_raw_fifo(fd):
            # attempt to read and discard any pending bytes to clear FIFO
            if fd is None:
                return
            try:
                while True:
                    chunk = os.read(fd, 65536)
                    if not chunk:
                        break
            except BlockingIOError:
                pass
            except OSError:
                pass

        while True:
            frame = None
            if raw_fd is not None:
                # wait for data with timeout to detect stalls
                rlist, _, _ = select.select([raw_fd], [], [], 1.0)
                if not rlist:
                    if args.debug:
                        print('[DEBUG] no data available on raw device (timeout)')
                    stall_count += 1
                    if stall_count >= stall_threshold:
                        if args.debug:
                            print('[DEBUG] draining raw FIFO due to repeated timeouts')
                        drain_raw_fifo(raw_fd)
                        # try reopening the device to recover
                        try:
                            os.close(raw_fd)
                        except Exception:
                            pass
                        try:
                            raw_fd = os.open(args.raw_device, os.O_RDONLY | os.O_NONBLOCK)
                        except Exception:
                            try:
                                raw_fd = os.open(args.raw_device, os.O_RDONLY)
                            except Exception as e:
                                if args.debug:
                                    print('[DEBUG] failed to reopen raw device:', e)
                                raw_fd = None
                        stall_count = 0
                    continue

                # read until we have a full frame
                chunks = []
                got = 0
                while got < frame_size:
                    try:
                        chunk = os.read(raw_fd, frame_size - got)
                    except BlockingIOError:
                        # no more data right now; break and wait again
                        break
                    if not chunk:
                        break
                    chunks.append(chunk)
                    got += len(chunk)

                data = b''.join(chunks)
                if len(data) < frame_size:
                    if args.debug:
                        print('[DEBUG] incomplete raw frame: got {0} of {1}'.format(len(data), frame_size))
                    stall_count += 1
                    if stall_count >= stall_threshold:
                        if args.debug:
                            print('[DEBUG] draining raw FIFO due to repeated incomplete frames')
                        drain_raw_fifo(raw_fd)
                        try:
                            os.close(raw_fd)
                        except Exception:
                            pass
                        try:
                            raw_fd = os.open(args.raw_device, os.O_RDONLY | os.O_NONBLOCK)
                        except Exception:
                            try:
                                raw_fd = os.open(args.raw_device, os.O_RDONLY)
                            except Exception as e:
                                if args.debug:
                                    print('[DEBUG] failed to reopen raw device:', e)
                                raw_fd = None
                        stall_count = 0
                    continue

                frame = uyvy_to_bgr(data, args.width, args.height)
                if frame is None:
                    if args.debug:
                        print('[DEBUG] uyvy_to_bgr returned None')
                    continue
            else:
                ret, frame = cap.read()
                if not ret:
                    break
            boards = detect_boards(frame, min_area=args.min_area)
            for i, (rect, area) in enumerate(boards):
                pts = rect.astype(int)
                color = (0,255,0) if i==0 else (0,200,200)
                cv2.polylines(frame, [pts], True, color, max(2, int(np.sqrt(area)/100)))
                cv2.putText(frame, 'Board#{} {}'.format(i+1, area), tuple(pts[0]), FONT, 0.6, color, 2)

            marker, mask = detect_marker(frame, args.color)
            if marker is not None:
                cv2.circle(frame, marker, 8, (0,0,255), -1)
                for rect, _ in boards:
                    if point_in_quad(marker, rect):
                        cv2.putText(frame, 'Robot on board', (marker[0]+10, marker[1]), FONT, 0.6, (0,255,0),2)
                        break

            if not args.no_display:
                if disp:
                    try:
                        # debug timing
                        if args.debug:
                            t0 = time.time()
                        # Apply requested rotation for display output
                        if args.rotation == 90:
                            out = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                        elif args.rotation == 180:
                            out = cv2.rotate(frame, cv2.ROTATE_180)
                        elif args.rotation == 270:
                            out = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                        else:
                            out = frame
                        # draw_frame expects a BGR numpy array; ensure contiguous uint8
                        out = np.ascontiguousarray(out, dtype=np.uint8)
                        # Convert channel order if LCD expects BRG instead of BGR
                        if args.lcd_order == 'BRG':
                            try:
                                out = out[:, :, [0, 2, 1]]
                            except Exception:
                                pass
                        disp.draw_frame(out)
                        if args.debug:
                            t1 = time.time(); print('[DEBUG] draw_frame took {0:.3f}s'.format(t1-t0))
                    except Exception as e:
                        print('Display update failed:', e)
                        disp = None
                else:
                    # No framebuffer display: attempt to show windows.
                    # Wrap in try/except to avoid Gtk errors on headless systems.
                    try:
                        # Apply rotation for window preview as well
                        if args.rotation == 90:
                            win_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                            win_mask = cv2.rotate(mask, cv2.ROTATE_90_CLOCKWISE) if mask is not None else None
                        elif args.rotation == 180:
                            win_frame = cv2.rotate(frame, cv2.ROTATE_180)
                            win_mask = cv2.rotate(mask, cv2.ROTATE_180) if mask is not None else None
                        elif args.rotation == 270:
                            win_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                            win_mask = cv2.rotate(mask, cv2.ROTATE_90_COUNTERCLOCKWISE) if mask is not None else None
                        else:
                            win_frame = frame
                            win_mask = mask
                        cv2.imshow('Tracker', win_frame)
                        if win_mask is not None:
                            cv2.imshow('Mask', win_mask)
                    except Exception:
                        # GUI not available (e.g. no X display). Continue headless.
                        pass
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break

    except KeyboardInterrupt:
        if args.debug:
            print('[DEBUG] interrupted by user')
    finally:
        # Cleanup resources
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass
        if raw_fd is not None:
            try:
                os.close(raw_fd)
            except Exception:
                pass
        # Clear the framebuffer display (if initialized) so the screen is blank on exit
        if disp is not None:
            try:
                disp.clear()
            except Exception:
                pass
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
