#!/usr/bin/env python3
"""Textures of the campus scene (logos and signs of the Air-Sea Test Basin building).

Made from the official ENSTA / IP Paris logo (horizontal version, high resolution) and drawn signs, following
the photos of the building:
  ensta_logo_stacked.png  ENSTA over a thin line, IP Paris seal + "IP PARIS" (long wall and arched end walls)
  ensta_slogan.png        "L'école des grandes avancées" (long wall)
  basin_sign.png          three round pictograms + "BASSIN D'EXPÉRIMENTATION AIR-MER / AIR-SEA TEST BASIN" (entrance)
  sign_no_entry.png, sign_weight_limit.png, sign_restricted.png   road signs at the foot of the ramp

All images have a transparent background (they are applied as decals on the walls).

Usage:
  python3 Tools/scene_assets/make_textures.py <logo-ensta.jpg> [--out Assets/Environment/Textures]
"""
import argparse
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FONT_DIR = '/usr/share/fonts/truetype/lato'
BOLD = os.path.join(FONT_DIR, 'Lato-Bold.ttf')
BLACK = os.path.join(FONT_DIR, 'Lato-Black.ttf')
REGULAR = os.path.join(FONT_DIR, 'Lato-Regular.ttf')
NAVY = (30, 44, 120, 255)          # dark blue of the texts on the building
INK = (25, 25, 25, 255)


def to_alpha(img):
    """White background → transparent (alpha from the darkness of each pixel), color kept."""
    a = np.asarray(img.convert('RGB')).astype(float)
    alpha = np.clip((255 - a.min(axis=2)) * 1.6, 0, 255)          # how far from white
    rgba = np.dstack([a, alpha]).astype(np.uint8)
    return Image.fromarray(rgba, 'RGBA')


def crop_ink(img, box):
    """Crops box from the logo and trims the empty margins."""
    part = to_alpha(img.crop(box))
    return part.crop(part.getbbox())


def stacked_logo(logo, out):
    w, h = logo.size
    ensta = crop_ink(logo, (0, 0, int(w * 0.49), h))               # ENSTA wordmark (left half, before the separator)
    seal = crop_ink(logo, (int(w * 0.525), 0, int(w * 0.64), h))   # IP Paris seal
    W = ensta.width + 200
    seal_h = int(ensta.height * 0.62)
    seal = seal.resize((int(seal.width * seal_h / seal.height), seal_h), Image.LANCZOS)
    font = ImageFont.truetype(BOLD, int(seal_h * 0.55))
    canvas = Image.new('RGBA', (W, ensta.height + seal_h + 260), (0, 0, 0, 0))
    canvas.alpha_composite(ensta, (100, 40))
    d = ImageDraw.Draw(canvas)
    y_line = 40 + ensta.height + 70
    d.line([(100, y_line), (100 + ensta.width, y_line)], fill=(60, 60, 60, 255), width=8)
    canvas.alpha_composite(seal, (100, y_line + 60))
    d.text((100 + seal.width + 50, y_line + 60 + seal_h / 2), 'IP PARIS', font=font, fill=INK, anchor='lm')
    canvas.save(os.path.join(out, 'ensta_logo_stacked.png'))


def slogan(out):
    font = ImageFont.truetype(BOLD, 150)
    lines = ["L'école", 'des grandes', 'avancées']
    img = Image.new('RGBA', (1100, 600), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    for i, line in enumerate(lines):
        d.text((20, 20 + i * 185), line, font=font, fill=NAVY)
    img.crop(img.getbbox()).save(os.path.join(out, 'ensta_slogan.png'))


def pictogram(d, cx, cy, r, kind):
    """Round black pictogram with a white symbol (wave, drone, boat)."""
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=INK)
    w = 'white'
    if kind == 'wave':
        pts = [(cx - r * 0.6 + i * r * 0.06, cy + r * 0.15 * np.sin(i * 0.55)) for i in range(21)]
        d.line(pts, fill=w, width=int(r * 0.12))
        d.line([(x, y + r * 0.35) for x, y in pts], fill=w, width=int(r * 0.12))
    elif kind == 'drone':
        for sx in (-1, 1):
            for sy in (-1, 1):
                d.line([(cx, cy), (cx + sx * r * 0.42, cy + sy * r * 0.42)], fill=w, width=int(r * 0.1))
                d.ellipse([cx + sx * r * 0.42 - r * 0.17, cy + sy * r * 0.42 - r * 0.17,
                           cx + sx * r * 0.42 + r * 0.17, cy + sy * r * 0.42 + r * 0.17], outline=w, width=int(r * 0.07))
        d.rectangle([cx - r * 0.12, cy - r * 0.12, cx + r * 0.12, cy + r * 0.12], fill=w)
    else:  # boat
        d.polygon([(cx - r * 0.6, cy + r * 0.1), (cx + r * 0.6, cy + r * 0.1), (cx + r * 0.4, cy + r * 0.4), (cx - r * 0.4, cy + r * 0.4)], fill=w)
        d.polygon([(cx - r * 0.05, cy - r * 0.55), (cx - r * 0.05, cy + r * 0.05), (cx + r * 0.4, cy + r * 0.05)], fill=w)


def basin_sign(out):
    img = Image.new('RGBA', (2400, 560), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    r = 75
    for i, kind in enumerate(['wave', 'drone', 'boat']):
        pictogram(d, 1250 + i * 190, 95, r, kind)
    d.text((1200, 280), "BASSIN D'EXPÉRIMENTATION AIR-MER", font=ImageFont.truetype(BLACK, 92), fill=INK, anchor='mm')
    d.text((1200, 400), 'AIR-SEA TEST BASIN', font=ImageFont.truetype(REGULAR, 84), fill=(70, 70, 70, 255), anchor='mm')
    img.crop(img.getbbox()).save(os.path.join(out, 'basin_sign.png'))


def road_signs(out):
    s = 512
    img = Image.new('RGBA', (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.ellipse([8, 8, s - 8, s - 8], fill=(200, 20, 30, 255), outline='white', width=14)
    d.rectangle([90, s / 2 - 45, s - 90, s / 2 + 45], fill='white')
    img.save(os.path.join(out, 'sign_no_entry.png'))

    img = Image.new('RGBA', (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.ellipse([8, 8, s - 8, s - 8], fill='white', outline=(200, 20, 30, 255), width=60)
    d.text((s / 2, s / 2), '3,5 t', font=ImageFont.truetype(BOLD, 130), fill=INK, anchor='mm')
    img.save(os.path.join(out, 'sign_weight_limit.png'))

    img = Image.new('RGBA', (560, 300), (255, 255, 255, 255))
    d = ImageDraw.Draw(img)
    d.rectangle([4, 4, 555, 295], outline=INK, width=8)
    for i, line in enumerate(['ACCÈS RÉSERVÉ', 'EXCLUSIVEMENT AU', 'PERSONNEL AUTORISÉ']):
        d.text((280, 70 + i * 80), line, font=ImageFont.truetype(BOLD, 50), fill=INK, anchor='mm')
    img.save(os.path.join(out, 'sign_restricted.png'))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logo', help='official horizontal logo ENSTA | IP Paris (logo-ensta.jpg)')
    ap.add_argument('--out', default=os.path.join(HERE, '..', '..', 'Assets', 'Environment', 'Textures'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    logo = Image.open(args.logo)
    stacked_logo(logo, args.out)
    slogan(args.out)
    basin_sign(args.out)
    road_signs(args.out)
    print('written to', os.path.abspath(args.out))


if __name__ == '__main__':
    main()
