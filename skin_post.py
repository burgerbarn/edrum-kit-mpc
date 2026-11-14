#!/usr/bin/env python3
"""EDrum Kit skin post-processing: draws a background per page (top-down drum kit, the page's drums
lit in amber) and wires them into the generated Plugin Skins/TUI.json. Run after tools/build.py."""
import json, math, random, sys, os
from PIL import Image, ImageDraw, ImageFilter, ImageFont

SK = sys.argv[1]   # .../build/<folder>/Plugin Skins
W, H = 1280, 628
AMBER = (255, 163, 26)
FONT_B = "/System/Library/Fonts/Supplemental/DIN Condensed Bold.ttf"
FONT_A = "/System/Library/Fonts/Avenir Next Condensed.ttc"

# Drummer's-eye top view, normalised: x in box width, y in box height, r in box height.
KIT = {
    "crash1": (0.15, 0.24, 0.20, "cym"), "hats": (0.08, 0.62, 0.15, "cym"), "snare": (0.27, 0.72, 0.15, "drum"),
    "tom1": (0.39, 0.30, 0.13, "drum"), "tom2": (0.55, 0.30, 0.14, "drum"), "kick": (0.47, 0.74, 0.25, "kick"),
    "tom3": (0.70, 0.68, 0.18, "drum"), "crash2": (0.77, 0.20, 0.19, "cym"), "ride": (0.93, 0.58, 0.20, "cym"),
}

def base():
    img = Image.new("RGB", (W, H))
    d = ImageDraw.Draw(img)
    for y in range(H):
        t = y / (H - 1)
        c = tuple(int(a + (b - a) * t) for a, b in zip((22, 23, 28), (11, 12, 15)))
        d.line([(0, y), (W, y)], fill=c)
    rnd = random.Random(7)
    tex = Image.new("L", (W, H), 0); td = ImageDraw.Draw(tex)
    for _ in range(2600):                     # faint brushed-metal streaks
        x = rnd.randrange(-50, W); y = rnd.randrange(H); L = rnd.randrange(20, 140)
        td.line([(x, y), (x + L, y + rnd.choice((-1, 0, 1)))], fill=rnd.randrange(4, 12))
    img = Image.composite(Image.new("RGB", (W, H), (60, 62, 70)), img, tex)
    return img

def draw_kit(img, box, lit, strength=1.0, all_soft=False):
    bx, by, bw, bh = box
    glow = Image.new("RGBA", (W, H), (0, 0, 0, 0)); gd = ImageDraw.Draw(glow)
    lay = Image.new("RGBA", (W, H), (0, 0, 0, 0)); ld = ImageDraw.Draw(lay)
    order = ["kick", "tom3", "snare", "tom1", "tom2", "hats", "crash1", "crash2", "ride"]
    for name in order:
        nx, ny, nr, kind = KIT[name]
        cx, cy, r = bx + nx * bw, by + ny * bh, nr * bh
        on = name in lit or all_soft
        a = int((230 if name in lit else (120 if all_soft else 70)) * strength)
        edge = AMBER + (a,) if on else (120, 124, 134, a)
        fill = (AMBER + (int((60 if name in lit else 22) * strength),)) if on else (30, 32, 38, int(150 * strength))
        if on and name in lit:
            gd.ellipse([cx - r * 1.25, cy - r * 1.25, cx + r * 1.25, cy + r * 1.25], fill=AMBER + (int(110 * strength),))
        if kind == "cym":
            ld.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill, outline=edge, width=3)
            for k in (0.72, 0.45):
                rr = r * k; ld.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=edge[:3] + (edge[3] // 3,), width=1)
            rb = r * 0.18; ld.ellipse([cx - rb, cy - rb, cx + rb, cy + rb], fill=edge[:3] + (edge[3] // 2,))
        elif kind == "kick":
            ld.rounded_rectangle([cx - r * 0.9, cy - r * 0.55, cx + r * 0.9, cy + r * 0.55], radius=r * 0.25, fill=fill, outline=edge, width=4)
            ld.line([cx - r * 0.75, cy, cx + r * 0.75, cy], fill=edge[:3] + (edge[3] // 3,), width=2)
        else:
            ld.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill, outline=edge, width=5)
            ri = r * 0.80; ld.ellipse([cx - ri, cy - ri, cx + ri, cy + ri], outline=edge[:3] + (edge[3] // 3,), width=1)
            for k in range(8):                   # tension lugs
                ang = k * math.pi / 4; lx, ly = cx + math.cos(ang) * r, cy + math.sin(ang) * r
                ld.ellipse([lx - 3, ly - 3, lx + 3, ly + 3], fill=edge)
    glow = glow.filter(ImageFilter.GaussianBlur(28))
    out = Image.alpha_composite(img.convert("RGBA"), glow)
    return Image.alpha_composite(out, lay).convert("RGB")

def title(img, x, y):
    d = ImageDraw.Draw(img)
    big = ImageFont.truetype(FONT_B, 74); small = ImageFont.truetype(FONT_A, 21, index=0)
    d.text((x, y), "EDRUM", font=big, fill=AMBER)
    w = d.textlength("EDRUM", font=big)
    d.text((x + w + 10, y), "KIT", font=big, fill=(236, 232, 226))
    d.text((x + 3, y + 80), "V-DRUMS  >  MPC   ·   NO DAW", font=small, fill=(140, 134, 124))
    return img

BOTTOM = (40, 318, 1200, 300)
PAGES = {
    "KIT":     dict(box=(650, 330, 600, 285), lit=set(), soft=True, title=(338, 448)),
    "MIX":     dict(box=BOTTOM, lit=set(KIT), soft=False),
    "KICK":    dict(box=BOTTOM, lit={"kick"}),
    "SNARE":   dict(box=BOTTOM, lit={"snare"}),
    "TOMS":    dict(box=(0, 20, W, 600), lit={"tom1", "tom2", "tom3"}, strength=0.35),
    "HATS":    dict(box=BOTTOM, lit={"hats"}),
    "CYMBALS": dict(box=(0, 20, W, 600), lit={"crash1", "crash2"}, strength=0.35),
    "RIDE":    dict(box=BOTTOM, lit={"ride"}),
}

tui_path = os.path.join(SK, "TUI.json")
tui = json.load(open(tui_path))
for tab in tui["pageData"]["tabs"]:
    name = tab["tabName"]; spec = PAGES.get(name)
    if not spec: continue
    img = draw_kit(base(), spec["box"], spec["lit"], spec.get("strength", 1.0), spec.get("soft", False))
    if "title" in spec: img = title(img, *spec["title"])
    fn = "bg_%s.png" % name.lower()
    img.save(os.path.join(SK, fn), optimize=True)
    bg = {"version": 1, "focussed": {"version": 1, "colour": "ff101114", "image": fn},
          "unfocussed": {"version": 1, "colour": "ff101114", "image": fn}}
    cd = tab.get("componentDefinition")
    if cd is not None: cd["backgroundData"] = bg
    # Q-Link frames: the generator draws 248 px frames from x 38, but each control column is 278 px from x 23,
    # so knobs and long values poke out. Fit each frame around its column with a little margin.
    tab["qlinkBoundsData"] = ["%d 0 298 624" % (13 + 320 * c) for c in range(4)]
print("backgrounds:", ", ".join(t["tabName"] for t in tui["pageData"]["tabs"]))
json.dump(tui, open(tui_path, "w"), indent=4)
