import math, sys
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION
from PIL import Image
from pptx.enum.dml import MSO_LINE
import qrcode

P = sys.argv[1]            # poster dir
VB = len(sys.argv) > 3 and sys.argv[3] == "B"
BH = 8.5 if VB else 6.85
BANNER = "banner_B.jpg" if VB else "banner_header.jpg"
OUT = sys.argv[2]

NAVY = RGBColor(0x0B, 0x2F, 0x4E)
TEAL = RGBColor(0x0E, 0x8F, 0x9A)
GREEN = RGBColor(0x2E, 0x7D, 0x32)
AMBER = RGBColor(0xC4, 0x6A, 0x00)
INK = RGBColor(0x1B, 0x24, 0x30)
MUTE = RGBColor(0x4A, 0x55, 0x65)
PANEL = RGBColor(0xF1, 0xF5, 0xF8)
HILITE = RGBColor(0xDD, 0xF1, 0xF2)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
LINE = RGBColor(0xC5, 0xCF, 0xD9)
FONT = "Arial"
MONO = "Courier New"

W, H = 90 / 2.54, 120 / 2.54
M = 1.0
GAP = 0.8
CW = (W - 2 * M - GAP) / 2
XL, XR = M, M + CW + GAP

prs = Presentation()
prs.slide_width, prs.slide_height = Inches(W), Inches(H)
sl = prs.slides.add_slide(prs.slide_layouts[6])


def rect(x, y, w, h, fill=None, line=None, shape=MSO_SHAPE.RECTANGLE, lw=1.5, name=None):
    s = sl.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    if fill is None:
        s.fill.background()
    else:
        s.fill.solid(); s.fill.fore_color.rgb = fill
    if line is None:
        s.line.fill.background()
    else:
        s.line.color.rgb = line; s.line.width = Pt(lw)
    s.shadow.inherit = False
    st = s._element.find('{http://schemas.openxmlformats.org/presentationml/2006/main}style')
    if st is not None: s._element.remove(st)
    if name: s.name = name
    return s


def text(x, y, w, h, paras, size=28, color=INK, bold=False, align=PP_ALIGN.LEFT,
         anchor=MSO_ANCHOR.TOP, font=FONT, space_after=0, line_spacing=1.08, name=None, margin=0.0):
    """paras: list of str | list of runs [(text, {bold,color,italic,font,size})] | dict(bullet=..)"""
    tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    if name: tb.name = name
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(margin)
    tf.margin_top = tf.margin_bottom = Inches(0)
    tf.vertical_anchor = anchor
    if isinstance(paras, str): paras = [paras]
    first = True
    for p in paras:
        para = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        bullet = False
        if isinstance(p, dict):
            bullet = True; p = p["b"]
        runs = p if isinstance(p, list) else [(p, {})]
        para.alignment = align
        para.line_spacing = line_spacing
        para.space_after = Pt(space_after)
        if bullet:
            pPr = para._p.get_or_add_pPr()
            ind = int(Pt(size) * 1.1)
            pPr.set("marL", str(ind)); pPr.set("indent", str(-ind))
            from lxml import etree
            ns = "http://schemas.openxmlformats.org/drawingml/2006/main"
            bc = etree.SubElement(pPr, "{%s}buClr" % ns)
            sc = etree.SubElement(bc, "{%s}srgbClr" % ns); sc.set("val", "0E8F9A")
            bu = etree.SubElement(pPr, "{%s}buChar" % ns); bu.set("char", "■")
        for t, o in runs:
            r = para.add_run(); r.text = t
            f = r.font
            f.name = o.get("font", font); f.size = Pt(o.get("size", size))
            f.bold = o.get("bold", bold); f.italic = o.get("italic", False)
            f.color.rgb = o.get("color", color)
    return tb


def est(paras, size, w, bullet_indent=True, ls=1.08, space_after=0):
    """estimated height (in) of paragraphs at size pt in width w (in)"""
    cw = 0.50 * size / 72
    tot = 0
    for p in paras:
        b = isinstance(p, dict)
        if b: p = p["b"]
        s = "".join(t for t, _ in p) if isinstance(p, list) else p
        ww = w - (size * 1.1 / 72 if b else 0)
        n = max(1, math.ceil(len(s) * cw / ww * 1.04))
        tot += n * size / 72 * 1.17 * ls + space_after / 72
    return tot


def heading(x, y, num, title):
    rect(x, y, CW, 1.0, fill=NAVY, name="Heading bar")
    rect(x, y, 0.22, 1.0, fill=TEAL)
    text(x + 0.5, y, CW - 0.7, 1.0, [[(f"{num}  ", {"color": RGBColor(0x7F, 0xD6, 0xDC)}), (title, {})]],
         size=42, color=WHITE, bold=True, anchor=MSO_ANCHOR.MIDDLE, name=f"Heading {title}")
    return y + 1.0 + 0.35


def bullets(x, y, items, size=27, w=CW, sa=10):
    ps = [{"b": i} for i in items]
    h = est(ps, size, w, space_after=sa)
    text(x, y, w, h, ps, size=size, space_after=sa)
    return y + h


def tile(x, y, w, h, big, small, color=NAVY):
    rect(x, y, w, h, fill=PANEL, line=None, shape=MSO_SHAPE.ROUNDED_RECTANGLE).adjustments[0] = 0.08
    rect(x, y + 0.25, 0.14, h - 0.5, fill=color)
    text(x + 0.4, y + 0.2, w - 0.6, h * 0.55, big, size=60, bold=True, color=color, anchor=MSO_ANCHOR.MIDDLE)
    text(x + 0.4, y + h * 0.58, w - 0.6, h * 0.4, small, size=22, color=MUTE, line_spacing=1.0)


def pic(path, x, y, w=None, h=None):
    im = Image.open(path); r = im.width / im.height
    if w is None: w = h * r
    if h is None: h = w / r
    sl.shapes.add_picture(path, Inches(x), Inches(y), Inches(w), Inches(h))
    return w, h



def tile2(x, y, w, h, big, small, color=NAVY, bigsize=96, smallsize=32):
    rect(x, y, w, h, fill=PANEL, line=None, shape=MSO_SHAPE.ROUNDED_RECTANGLE).adjustments[0] = 0.08
    rect(x, y + 0.3, 0.18, h - 0.6, fill=color)
    text(x + 0.5, y + 0.25, w - 0.7, h * 0.52, big, size=bigsize, bold=True, color=color, anchor=MSO_ANCHOR.MIDDLE)
    text(x + 0.5, y + h * 0.58, w - 0.7, h * 0.4, small, size=smallsize, color=MUTE, line_spacing=1.0)




# =================== V3: no estilo dos posteres do orientador (ASE 2017 / TACAS 2019) ===================
from pptx.enum.shapes import MSO_SHAPE as SH
BLUE = RGBColor(0x0B, 0x2F, 0x4E)       # destaque (marinho)
BLUE2 = RGBColor(0x07, 0x1F, 0x35)
PALE = RGBColor(0xDD, 0xF1, 0xF2)
BODY = 40
SMALL = 30

# fundo azul
bg = rect(0, 0, W, H, fill=RGBColor(0xEC, 0xF2, 0xF5), name="Background")

def panel(x, y, w):
    p = rect(x, y, w, 1.0, fill=WHITE, line=LINE, shape=SH.ROUNDED_RECTANGLE, lw=1.5, name="Panel")
    p.adjustments[0] = 0.015
    return p

def phead(x, y, w, num, title):
    rect(x + 0.45, y + 0.35, 1.15, 1.15, fill=BLUE, shape=SH.OVAL)
    text(x + 0.45, y + 0.35, 1.15, 1.15, str(num), size=48, bold=True, color=WHITE,
         align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(x + 1.9, y + 0.35, w - 2.2, 1.15, title, size=62, bold=True, color=BLUE, anchor=MSO_ANCHOR.MIDDLE)
    return y + 1.85

def pbullets(x, y, w, items, size=BODY, sa=14):
    ps = [{"b": i} for i in items]
    h = est(ps, size, w, space_after=sa)
    text(x, y, w, h, ps, size=size, space_after=sa)
    return y + h

def close_panel(p, y0, y_end):
    p.height = Inches(y_end + 0.5 - y0)
    return y_end + 0.5

def table(x, y, colw, rows, size=SMALL, rowh=0.85, hdr_fill=BLUE, aligns=None, colors=None):
    nr, nc = len(rows), len(colw)
    tot_h = sum(rowh) if isinstance(rowh, list) else rowh * nr
    gf = sl.shapes.add_table(nr, nc, Inches(x), Inches(y), Inches(sum(colw)), Inches(tot_h))
    gf.name = "Table"
    t = gf.table
    t.first_row = True; t.horz_banding = False
    for c in range(nc): t.columns[c].width = Inches(colw[c])
    tot = 0
    for r in range(nr):
        rh = rowh[r] if isinstance(rowh, list) else rowh
        t.rows[r].height = Inches(rh); tot += rh
        for c in range(nc):
            cell = t.cell(r, c)
            cell.fill.solid()
            cell.fill.fore_color.rgb = hdr_fill if r == 0 else (PANEL if r % 2 == 0 else WHITE)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.margin_left = cell.margin_right = Inches(0.18)
            cell.margin_top = cell.margin_bottom = Inches(0.06)
            tf = cell.text_frame; tf.word_wrap = True
            para = tf.paragraphs[0]
            run = para.add_run(); run.text = rows[r][c]
            f = run.font; f.name = FONT; f.size = Pt(size)
            f.bold = (r == 0) or (c == 0)
            f.color.rgb = WHITE if r == 0 else (colors[r][c] if colors and colors[r][c] else INK)
    return y + tot

def box(x, y, w, h, label, fill=PALE, line=BLUE, color=INK, size=30, bold=True, shape=SH.ROUNDED_RECTANGLE):
    b = rect(x, y, w, h, fill=fill, line=line, lw=2.5, shape=shape)
    if shape == SH.ROUNDED_RECTANGLE: b.adjustments[0] = 0.15
    text(x + 0.15, y + 0.05, w - 0.3, h - 0.1, label.split("\n"), size=size, bold=bold, color=color,
         align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)

def darrow(cx, y, h=0.55, w=0.6, color=BLUE):
    rect(cx - w / 2, y, w, h, fill=color, shape=SH.DOWN_ARROW)

def rarrow(x, cy, w=0.7, h=0.55, color=BLUE):
    rect(x, cy - h / 2, w, h, fill=color, shape=SH.RIGHT_ARROW)


# ============================================================ v7: limpo, sem floresta
# ===== v10: estrutura do regulamento SBESC (Introduction, Proposal, Development, Results, Conclusions, References, Acknowledgment)
NAVY = RGBColor(0x0B, 0x3A, 0x63)
BLUE = RGBColor(0x1F, 0x6F, 0xB2)
LBLUE = RGBColor(0xE8, 0xF1, 0xF9)
ORANGE = RGBColor(0xD9, 0x73, 0x0D)
GREEN = RGBColor(0x2E, 0x7D, 0x32)
DGRAY = RGBColor(0x1F, 0x29, 0x33)
MGRAY = RGBColor(0x55, 0x60, 0x6E)
LGRAY = RGBColor(0xF2, 0xF4, 0xF7)
TEAL = BLUE; PALE = LBLUE; PANEL = LGRAY; INK = DGRAY; MUTE = MGRAY
bg.fill.fore_color.rgb = WHITE

HB = 5.2
rect(0, 0, W, HB, fill=NAVY, name="Header")
rect(M, 0.55, 4.1, 4.1, fill=WHITE, shape=SH.OVAL)
pic(f"{P}/logos/ufam_round.png", M + 0.1, 0.65, h=3.9)
tx, tw = M + 4.7, W - 2 * M - 4.7 - 0.2
text(tx, 0.4, tw, 2.5,
     "Formal Verification of Critical Modules in the PX4 Autopilot using Bounded Model Checking",
     size=68, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0, name="Title")
text(tx, 3.0, tw, 0.8, "Marcela Alves Rodrigues · Lucas C. Cordeiro · José Reginaldo H. de Carvalho",
     size=40, bold=True, color=WHITE, align=PP_ALIGN.CENTER, name="Authors")
text(tx, 3.85, tw, 1.2,
     ["Graduate Program in Electrical Engineering (PPGEE) · Institute of Computing · Federal University of Amazonas (UFAM), Manaus, Brazil",
      "XVI Symposium on Computing Systems Engineering · Nov 24–27, 2026 · Salvador, BA"],
     size=26, color=RGBColor(0xCF, 0xE0, 0xF0), align=PP_ALIGN.CENTER, line_spacing=1.1, space_after=4, name="Affiliation")

TOP = HB + 0.8
PW = CW
PAD = 0.6
IW = PW - 2 * PAD
FH = 2.9
FY = H - 0.8 - FH
END = FY - 0.6
X0L = XL + PAD
X0R = XR + PAD
GAPS = 0.65


def bar(x, y, w, num, title, fs=54):
    rect(x, y, w, 1.35, fill=NAVY, name="Section bar")
    rect(x, y, 1.35, 1.35, fill=BLUE)
    text(x, y, 1.35, 1.35, str(num), size=52, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(x + 1.6, y, w - 1.7, 1.35, title, size=fs, bold=True, color=WHITE, anchor=MSO_ANCHOR.MIDDLE)
    return y + 1.35 + 0.55


def sub(x, y, w, t, size=34):
    text(x, y, w, 0.75, t, size=size, bold=True, color=NAVY)
    return y + 0.85


def mark_bullets(x, y, w, items, size=38, sa=14):
    ps = [{"b": i} for i in items]
    h = est(ps, size, w, space_after=sa)
    text(x, y, w, h, ps, size=size, space_after=sa)
    return y + h


# ============================================================ coluna esquerda
y = TOP
# ---- 2 Introduction
y = bar(XL, y, PW, 1, "Introduction")
y = mark_bullets(X0L, y, IW, [
    [("Safety-critical firmware: ", {"bold": True}), ("PX4 has about 710,000 lines of C/C++ in one address space.", {})],
    "A fault in one driver can corrupt flight-critical state.",
    [("Gap: ", {"bold": True, "color": BLUE}), ("production driver code is little examined by software verifiers.", {})],
])
y += 0.4
bw_ = 2.45; gp_ = (IW - 5 * bw_) / 4; bh_ = 1.4
cont = rect(X0L + (bw_ + gp_) - 0.2, y + 0.5, 3 * bw_ + 2 * gp_ + 0.4, bh_ + 0.45, line=BLUE, lw=3, shape=SH.ROUNDED_RECTANGLE)
cont.adjustments[0] = 0.08; cont.line.dash_style = MSO_LINE.DASH
text(X0L + (bw_ + gp_) - 0.2, y, 3 * bw_ + 2 * gp_ + 0.4, 0.5, "one shared address space", size=26, bold=True,
     color=BLUE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
by_ = y + 0.72
for k_, lab_ in enumerate(["Sensors", "PX4\ndrivers", "uORB\nbus", "Flight\ncontrol", "Vehicle\n(motors)"]):
    bx_ = X0L + k_ * (bw_ + gp_); edge = k_ in (0, 4)
    box(bx_, by_, bw_, bh_, lab_, size=28, fill=LGRAY if edge else LBLUE, line=LINE if edge else BLUE)
    if k_ < 4: rarrow(bx_ + bw_ + 0.05, by_ + bh_ / 2, w=gp_ - 0.1, h=0.5, color=NAVY)
ly_ = by_ + bh_ + 0.4
rect(X0L + 4 * (bw_ + gp_) + bw_ / 2 - 0.02, by_ + bh_, 0.04, 0.42, fill=NAVY)
rect(X0L + bw_ / 2 - 0.02, ly_ - 0.02, 4 * (bw_ + gp_) + 0.04, 0.04, fill=NAVY)
rect(X0L + bw_ / 2 - 0.02, by_ + bh_ + 0.2, 0.04, 0.22, fill=NAVY)
rect(X0L + bw_ / 2 - 0.15, by_ + bh_ + 0.03, 0.3, 0.22, fill=NAVY, shape=SH.ISOSCELES_TRIANGLE)
text(X0L + 2 * (bw_ + gp_) - 0.5, ly_ + 0.08, bw_ + 1.0, 0.45, "feedback loop", size=22, color=MUTE, align=PP_ALIGN.CENTER)
y = ly_ + 0.8 + GAPS

# ---- 3 Proposal
y = bar(XL, y, PW, 2, "Proposal")
kb = rect(X0L, y, IW, 2.1, fill=LBLUE, shape=SH.ROUNDED_RECTANGLE); kb.adjustments[0] = 0.08
rect(X0L, y + 0.25, 0.2, 1.6, fill=BLUE)
text(X0L + 0.6, y + 0.1, IW - 1.0, 1.9,
     [[("Bounded model checking ", {"bold": True, "color": NAVY}),
       ("of production PX4 modules with hardware-free harnesses and datasheet-derived assertions.", {})]],
     size=36, anchor=MSO_ANCHOR.MIDDLE)
y += 2.1 + 0.4
t3 = (IW - 0.8) / 3
for k_, (big, small) in enumerate([("12", "PX4 modules"), ("2", "tools: ESBMC, CBMC"), ("8", "methodology stages")]):
    tile2(X0L + k_ * (t3 + 0.4), y, t3, 2.1, big, small, NAVY, bigsize=80, smallsize=28)
y += 2.1 + GAPS

# ---- 4 Development
y = bar(XL, y, PW, 3, "Development")
b1w = (IW - 1.0) / 2
box(X0L, y, b1w, 1.35, "PX4 module source\n(stage 1)", size=28)
text(X0L + b1w, y, 1.0, 1.35, "+", size=56, bold=True, color=NAVY, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
box(X0L + b1w + 1.0, y, b1w, 1.35, "Harness: hardware stubs +\ndatasheet assertions (2–3)", size=28)
y += 1.35
darrow(X0L + IW / 2, y + 0.08, h=0.5, color=NAVY); y += 0.68
box(X0L, y, IW, 1.2, "ESBMC run (4)  ·  CBMC cross-check (7)", size=32, fill=NAVY, line=NAVY, color=WHITE)
y += 1.2
lw_ = (IW - 0.8) / 2
darrow(X0L + lw_ / 2, y + 0.08, h=0.5, color=NAVY); darrow(X0L + lw_ + 0.8 + lw_ / 2, y + 0.08, h=0.5, color=NAVY)
y += 0.68
box(X0L, y, lw_, 1.4, "No violation within\nbound k = verified (model)", size=26, fill=LGRAY, line=GREEN, color=GREEN)
box(X0L + lw_ + 0.8, y, lw_, 1.4, "Counterexample\n(violation in the model)", size=26, fill=LGRAY, line=ORANGE, color=ORANGE)
y += 1.4
darrow(X0L + lw_ + 0.8 + lw_ / 2, y + 0.08, h=0.5, color=NAVY); y += 0.68
box(X0L + lw_ + 0.8, y, lw_, 1.2, "Check vs. production code,\nreport to PX4 (5–6)", size=24)
y += 1.2 + 0.5
y = sub(X0L, y, IW, "How BMC finds it: every execution up to bound k", 28)
im_ = Image.open(f"{P}/build/eq1_t.png"); w_eq = 11.6; h_eq = w_eq * im_.height / im_.width
pic(f"{P}/build/eq1_t.png", X0L + (IW - w_eq) / 2, y, w=w_eq)
y += h_eq + 0.4
y = sub(X0L, y, IW, "Modules covered, by input class", 28)
rows = [("Input class", "Modules"), ("Serial sensor", "GPS uBlox, IMU BMI088"), ("Telemetry", "MAVLink"),
        ("Radio", "Expo RC, RC Update"), ("Power sensor", "Battery Monitor"), ("Actuation", "PWM Output, VTOL"),
        ("Mission logic", "Navigator"), ("State estimation", "EKF2 Covariance"), ("Sensor aggregation", "Magnetometer, Barometer")]
y = table(X0L, y, [5.6, IW - 5.6], rows, size=26, rowh=0.56, hdr_fill=NAVY)
yL = y
print("fim esquerda %.2f limite %.2f" % (yL, END))

# ============================================================ coluna direita
y = TOP
y = bar(XR, y, PW, 4, "Results")
tw3 = (IW - 0.8) / 3
TH = 2.8
tile2(X0R, y, tw3, TH, "10 of 12", "harnesses with no violation found", GREEN, bigsize=60, smallsize=26)
tile2(X0R + tw3 + 0.4, y, tw3, TH, "3", "counterexamples, 0 confirmed bugs", ORANGE, bigsize=110, smallsize=26)
tile2(X0R + 2 * (tw3 + 0.4), y, tw3, TH, "≤ 65 s", "verification time per harness", NAVY, bigsize=76, smallsize=26)
y += TH + 0.5
y = sub(X0R, y, IW, "Verification time per harness (s, median of 3 runs)", 30)
data = [("GPS uBlox", 64.9, 1), ("MAVLink Parser", 49.2, 0), ("EKF2 Covariance", 12.2, 0), ("RC Update", 9.5, 0),
        ("Battery Monitor", 6.7, 0), ("Magnetometer", 6.6, 0), ("Expo RC", 4.9, 0), ("VTOL", 4.0, 0),
        ("Barometer", 0.4, 0), ("PWM Output", 0.2, 0), ("IMU BMI088", 0.1, 1), ("Navigator/Mission", 0.1, 0)]
LW_ = 5.0; BX = X0R + LW_ + 0.2; BWMAX = IW - LW_ - 0.2 - 1.8; RH = 0.62
for i, (nm, v, ce) in enumerate(data):
    yy = y + i * RH
    if i % 2 == 0: rect(X0R, yy, IW, RH, fill=LGRAY)
    text(X0R + 0.1, yy, LW_ - 0.1, RH, nm, size=26, color=INK, align=PP_ALIGN.RIGHT, anchor=MSO_ANCHOR.MIDDLE)
    bw = max(BWMAX * v / 65.0, 0.08)
    rect(BX, yy + 0.09, bw, RH - 0.18, fill=ORANGE if ce else GREEN)
    text(BX + bw + 0.15, yy, 1.8, RH, f"{v:.1f}", size=26, bold=True, color=INK, anchor=MSO_ANCHOR.MIDDLE)
y += len(data) * RH + 0.2
rect(BX, y + 0.1, 0.45, 0.34, fill=GREEN); text(BX + 0.6, y, 3.8, 0.55, "no violation", size=24, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
rect(BX + 4.4, y + 0.1, 0.45, 0.34, fill=ORANGE); text(BX + 5.05, y, 5.0, 0.55, "counterexample", size=24, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
y += 0.9
y = sub(X0R, y, IW, "The 3 counterexamples", 30)
cm_w = 4.2; co_w = 4.3
xm, xp, xo = X0R, X0R + cm_w + 0.1, X0R + IW - co_w
RHT = 1.85
for mod, prob, res in [("GPS uBlox", "Out-of-bounds copy in the model; unreachable in production", ["False", "positive"]),
                       ("BMI088 FIFO", "Harness omitted 3 guards present in the driver", ["False", "positive"]),
                       ("BMI088 temperature", "86.25 °C vs. 85 °C datasheet limit; PX4 declined it", ["Disputed", "specification"])]:
    rr = rect(X0R, y, IW, RHT, fill=LGRAY, shape=SH.ROUNDED_RECTANGLE); rr.adjustments[0] = 0.08
    rect(X0R, y + 0.2, 0.18, RHT - 0.4, fill=ORANGE)
    text(xm + 0.45, y, cm_w - 0.5, RHT, mod, size=32, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
    text(xp, y, xo - xp - 0.3, RHT, prob, size=26, color=INK, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.05)
    pill = rect(xo + 0.2, y + 0.25, co_w - 0.4, RHT - 0.5, fill=ORANGE, shape=SH.ROUNDED_RECTANGLE); pill.adjustments[0] = 0.3
    text(xo + 0.2, y + 0.25, co_w - 0.4, RHT - 0.5, res, size=26, bold=True, color=WHITE, align=PP_ALIGN.CENTER,
         anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
    y += RHT + 0.22
y += -0.22 + GAPS

# ---- 6 Conclusions / Future work
y = bar(XR, y, PW, 5, "Conclusions / Future work")
CHT = 1.7
for k_, (t_, d_) in enumerate([("End-to-end methodology", "Hardware stubs, datasheet assertions, cross-tool check."),
                               ("Feasible on production PX4 code", "10 of 12 harnesses clean within the bound."),
                               ("Model-level evidence, not a proof", "Bounded models; no confirmed defect in this campaign.")], 1):
    rect(X0R, y, IW, CHT, fill=LBLUE, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.08
    rect(X0R + 0.35, y + (CHT - 0.9) / 2, 0.9, 0.9, fill=BLUE, shape=SH.OVAL)
    text(X0R + 0.35, y + (CHT - 0.9) / 2, 0.9, 0.9, str(k_), size=34, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(X0R + 1.6, y + 0.05, IW - 1.8, CHT - 0.1, [[(t_, {"bold": True, "size": 32, "color": NAVY})], [(d_, {"size": 26})]],
         size=26, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.05, space_after=2)
    y += CHT + 0.2
y += 0.1
text(X0R, y, IW, 1.3, [[("Next: ", {"bold": True, "color": BLUE}),
                        ("k-induction, failsafe and geofence modules, hardware-in-the-loop.", {})]], size=30)
y += 1.0 + GAPS

# ---- 7 References | 8 Acknowledgment
hw = (PW - 0.6) / 2
ry = y
yy1 = bar(XR, ry, hw, 6, "References", 40)
text(XR + 0.1, yy1, hw - 0.2, 4.6,
     ["[1] Biere et al. Symbolic model checking without BDDs. TACAS, 1999.",
      "[2] Cordeiro, Fischer, Marques-Silva. SMT-based bounded model checking for embedded ANSI-C software. IEEE TSE, 2012.",
      "[3] Gadelha, Ismail, Cordeiro. Handling loops in BMC of C programs via k-induction. STTT, 2017."],
     size=23, color=INK, space_after=8, line_spacing=1.05)
xa = XR + hw + 0.6
yy2 = bar(xa, ry, hw, 7, "Acknowledgment", 38)
QR = 2.8
text(xa + 0.1, yy2, hw - QR - 0.5, 4.6,
     [[("CAPES (Finance Code 001) and FAPEAM. Part of the SAGRES Project.", {})],
      [("Artifact (harnesses, logs):", {"bold": True})], [("doi.org/10.5281/", {"color": BLUE})], [("zenodo.23282236", {"color": BLUE})]],
     size=23, color=INK, space_after=10, line_spacing=1.05)
qimg = qrcode.make("https://doi.org/10.5281/zenodo.23282236", box_size=20, border=1)
qimg.save(f"{P}/build/qr.png")
pic(f"{P}/build/qr.png", xa + hw - QR, yy2 - 0.1, w=QR)
yR = max(yy1 + 3.0, yy2 + QR) 
print("fim direita %.2f limite %.2f" % (yR, END))

# ============================================================ rodape (todos os logos do regulamento)
rect(M, FY, W - 2 * M, 0.06, fill=NAVY)
groups = [("Event", ["l10"]), ("Organization", ["l01", "l02"]), ("Promotion", ["l11"]), ("Funding Agencies", ["l04", "l05"]),
          ("Support", ["l08", "l07"]), ("Sponsors", ["l06", "hwit"]), ("Author’s institution", ["ufam"])]
LH = 1.55
hmap = {"l04": 0.95, "l07": 0.9, "l06": 1.4, "l10": 1.0}
widths = []
for g, ls in groups:
    ws = []
    for l in ls:
        im = Image.open(f"{P}/logos/{l}.png"); hh = hmap.get(l, LH); ws.append(hh * im.width / im.height)
    widths.append(ws)
inner = 0.4
gw = [max(sum(ws) + inner * (len(ws) - 1), 0.5) for ws in widths]
IN = 0.4
space = (W - 2 * M - 2 * IN - sum(gw)) / (len(groups) - 1)
x = M + IN
for (g, ls), ws, gwid in zip(groups, widths, gw):
    text(x - 1.0, FY + 0.2, gwid + 2.0, 0.45, g, size=22, bold=True, color=MUTE, align=PP_ALIGN.CENTER)
    xx = x
    for l, w_ in zip(ls, ws):
        hh = hmap.get(l, LH)
        pic(f"{P}/logos/{l}.png", xx, FY + 0.8 + (LH - hh) / 2, h=hh)
        xx += w_ + inner
    x += gwid + space
prs.save(OUT)
