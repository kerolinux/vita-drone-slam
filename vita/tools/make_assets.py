"""Draws the Vita Drone icon and LiveArea pictures (needs Pillow: pip install pillow).

Run from the vita/ folder:  python tools/make_assets.py
Writes sce_sys/icon0.png (128x128), sce_sys/livearea/contents/bg.png (840x500) and startup.png (280x158),
as 8-bit palette PNGs like the official VitaSDK samples. Edit the colours below to restyle them.
"""
import math
import os

from PIL import Image, ImageDraw, ImageFilter, ImageFont

TOP, BOTTOM = (8, 22, 52), (14, 98, 114)          # background gradient
ARM, BODY, RING, ACCENT = (205, 218, 230), (244, 248, 252), (90, 225, 255), (255, 150, 40)
FONT_PATHS = ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "C:/Windows/Fonts/arialbd.ttf",
              "/System/Library/Fonts/Supplemental/Arial Bold.ttf")
SS = 4                                             # supersampling factor for smooth edges


def font(size):
    for path in FONT_PATHS:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


def gradient(w, h):
    img = Image.new("RGB", (w, h))
    px = ImageDraw.Draw(img)
    for y in range(h):
        t = y / max(1, h - 1)
        px.line([(0, y), (w, y)], fill=tuple(int(TOP[i] + (BOTTOM[i] - TOP[i]) * t) for i in range(3)))
    return img.convert("RGBA")


def drone(size, alpha=255):
    """A top-down quadcopter on a transparent square canvas of `size` pixels."""
    s = size * SS
    layer = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    c, off, rr = s / 2, s * 0.285, s * 0.175
    for sx, sy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
        x, y = c + sx * off, c + sy * off
        d.line([(c, c), (x, y)], fill=ARM + (alpha,), width=int(s * 0.05))
        d.ellipse([x - s * 0.025, y - s * 0.025, x + s * 0.025, y + s * 0.025], fill=ARM + (alpha,))
        d.ellipse([x - rr, y - rr, x + rr, y + rr], fill=RING + (int(alpha * 0.22),),
                  outline=RING + (alpha,), width=max(2, int(s * 0.016)))
        for k in (0.3, 1.3):                                            # two blades per rotor
            a = k * math.pi / 2 + (0.5 if sx * sy > 0 else -0.5)
            d.line([(x - rr * 0.85 * math.cos(a), y - rr * 0.85 * math.sin(a)),
                    (x + rr * 0.85 * math.cos(a), y + rr * 0.85 * math.sin(a))],
                   fill=(255, 255, 255, int(alpha * 0.55)), width=max(2, int(s * 0.012)))
        hub = s * 0.03
        d.ellipse([x - hub, y - hub, x + hub, y + hub], fill=(ACCENT if sy < 0 else (230, 240, 250)) + (alpha,))
    bw, bh = s * 0.15, s * 0.19
    d.rounded_rectangle([c - bw, c - bh, c + bw, c + bh], radius=s * 0.06, fill=BODY + (alpha,))
    d.ellipse([c - s * 0.055, c - s * 0.12, c + s * 0.055, c - s * 0.01], fill=(18, 30, 52, alpha))
    d.ellipse([c - s * 0.03, c - s * 0.1, c + s * 0.0, c - s * 0.07], fill=(120, 200, 255, alpha))
    d.polygon([(c, c + s * 0.03), (c - s * 0.045, c + s * 0.15), (c + s * 0.045, c + s * 0.15)],
              fill=ACCENT + (alpha,))
    return layer.resize((size, size), Image.LANCZOS)


def glow(canvas, cx, cy, radius, strength=90):
    g = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    ImageDraw.Draw(g).ellipse([cx - radius, cy - radius, cx + radius, cy + radius], fill=(60, 200, 235, strength))
    canvas.alpha_composite(g.filter(ImageFilter.GaussianBlur(radius * 0.5)))


def paste_drone(canvas, cx, cy, size, alpha=255):
    canvas.alpha_composite(drone(size, alpha), (int(cx - size / 2), int(cy - size / 2)))


def save_palette(img, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    img.convert("RGB").quantize(colors=256, method=Image.Quantize.MEDIANCUT,
                                dither=Image.Dither.FLOYDSTEINBERG).save(path, optimize=True)
    print("wrote", path, img.size)


def fit_text(draw, text, x, y, max_width, start_size, fill):
    """Draw text at (x, y), shrinking the font until it is at most max_width pixels wide."""
    size = start_size
    while size > 8 and font(size).getlength(text) > max_width:
        size -= 1
    draw.text((x, y), text, font=font(size), fill=fill)


def make_icon(path):
    canvas = gradient(128, 128)
    glow(canvas, 64, 64, 46)
    paste_drone(canvas, 64, 64, 118)
    save_palette(canvas, path)


def make_startup(path):
    canvas = gradient(280, 158)
    glow(canvas, 70, 79, 58)
    paste_drone(canvas, 72, 79, 128)
    d = ImageDraw.Draw(canvas)
    fit_text(d, "VITA", 140, 38, 132, 46, (255, 255, 255, 255))
    fit_text(d, "DRONE", 140, 88, 132, 40, RING + (255,))
    save_palette(canvas, path)


def make_background(path):
    canvas = gradient(840, 500)
    d = ImageDraw.Draw(canvas)
    for x in range(0, 840, 60):
        d.line([(x, 0), (x, 500)], fill=(255, 255, 255, 10))
    for y in range(0, 500, 60):
        d.line([(0, y), (840, y)], fill=(255, 255, 255, 10))
    glow(canvas, 650, 250, 190, 70)
    paste_drone(canvas, 650, 250, 420, alpha=60)
    save_palette(canvas, path)


if __name__ == "__main__":
    make_icon("sce_sys/icon0.png")
    make_startup("sce_sys/livearea/contents/startup.png")
    make_background("sce_sys/livearea/contents/bg.png")