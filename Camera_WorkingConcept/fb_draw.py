# fb_draw.py - simple loading bar + banner using ST7789 framebuffer display
import time
from st7789 import ST7789
from PIL import Image, ImageDraw, ImageFont
import numpy as np


def center_x(display, w):
    return max(0, (display.width - w) // 2)


def main():
    disp = ST7789()  # use default /dev/fb1

    # Print framebuffer info for debugging
    print('FB info: width=%d height=%d bpp=%d stride=%d' % (disp.width, disp.height, disp.bpp, disp.stride))

    # Prepare font
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 18)
    except Exception:
        font = ImageFont.load_default()

    title = 'WIPERITE Ver.001'

    try:
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
            # redraw background for the bar area to avoid artifacts
            draw.rectangle((bar_x, bar_y, bar_x+bar_w-1, bar_y+bar_h-1), fill=(0,0,0))
            if i > 0:
                draw.rectangle((bar_x, bar_y, bar_x+i-1, bar_y+bar_h-1), fill=(0,0,255))

            # Rotate frame 180° then convert PIL RGB -> BGR numpy for draw_frame
            np_frame = np.array(frame.rotate(180), dtype=np.uint8)[:, :, ::-1]
            disp.draw_frame(np_frame)
            time.sleep(0.01)

        # Clear the screen (remove loading bar) before drawing the title
        draw.rectangle((0, 0, disp.width, disp.height), fill=(0, 0, 0))
        np_frame = np.array(frame.rotate(180), dtype=np.uint8)[:, :, ::-1]
        disp.draw_frame(np_frame)
        time.sleep(0.05)

        # After loading, draw title with background rectangle for clarity
        txt_w, txt_h = draw.textsize(title, font=font)
        txt_x = center_x(disp, txt_w)
        txt_y = bar_y + bar_h + 8
        # draw bg then text
        draw.rectangle((txt_x-2, txt_y-2, txt_x+txt_w+2, txt_y+txt_h+2), fill=(0,0,0))
        draw.text((txt_x, txt_y), title, font=font, fill=(255,255,255))
        # Rotate frame 180° for correct orientation
        np_frame = np.array(frame.rotate(180), dtype=np.uint8)[:, :, ::-1]
        disp.draw_frame(np_frame)

        # Hold until interrupted
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        disp.clear()


if __name__ == '__main__':
    main()