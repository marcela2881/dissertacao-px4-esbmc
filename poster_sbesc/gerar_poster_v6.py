import math, sys
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION
from PIL import Image
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

# ---------------------------------------------------------------- cabecalho
sl.shapes.add_picture(f"{P}/build/{BANNER}", 0, 0, Inches(W), Inches(BH)).name = "Header forest and drone"
pic(f"{P}/logos/ufam_round.png", M, M, h=4.0)
pic(f"{P}/logos/l10.png", W - M - 5.0, M + 0.35, w=5.0)
text(W - M - 5.0, M + 2.75, 5.0, 1.2,
     ["XVI Symposium on Computing", "Systems Engineering", "Nov 24–27, 2026 · Salvador, BA"],
     size=20, color=MUTE, align=PP_ALIGN.CENTER, line_spacing=1.05)
tx, tw = M + 4.5, W - 2 * M - 4.5 - 5.4
text(tx, M - 0.05, tw, 3.1,
     "Formal Verification of Critical Modules in the PX4 Autopilot using Bounded Model Checking",
     size=72, bold=True, color=NAVY, align=PP_ALIGN.CENTER, line_spacing=1.0, name="Title")
text(tx, M + 3.0, tw, 0.9,
     "Marcela Alves Rodrigues · Lucas C. Cordeiro · José Reginaldo H. de Carvalho",
     size=42, bold=True, color=INK, align=PP_ALIGN.CENTER, name="Authors")
text(tx, M + 3.95, tw, 1.1,
     ["Graduate Program in Electrical Engineering (PPGEE) · Institute of Computing",
      "Federal University of Amazonas (UFAM), Manaus, Brazil"],
     size=28, color=MUTE, align=PP_ALIGN.CENTER, name="Affiliation")
rect(0, BH, W, 0.12, fill=TEAL)

TOP = BH + 0.8
PW = CW
PAD = 0.7
IW = PW - 2 * PAD
tw3 = (IW - 0.8) / 3

# =============================================================== coluna esquerda
# ---- 1 Motivation
from pptx.enum.dml import MSO_LINE
import math as _m

def drone(cx, cy, sc=1.0):
    ang = 30
    for a in (ang, -ang):
        arm = rect(cx - 0.95 * sc, cy - 0.045 * sc, 1.9 * sc, 0.09 * sc, fill=BLUE); arm.rotation = a
    for sx in (-1, 1):
        for sy in (-1, 1):
            rx = cx + sx * 0.95 * sc * _m.cos(_m.radians(ang)); ry = cy + sy * 0.95 * sc * _m.sin(_m.radians(ang))
            rect(rx - 0.42 * sc, ry - 0.16 * sc, 0.84 * sc, 0.32 * sc, fill=RGBColor(0xC9, 0xD3, 0xDB),
                 line=RGBColor(0x8A, 0x97, 0xA3), shape=SH.OVAL, lw=1.5)
    b_ = rect(cx - 0.4 * sc, cy - 0.27 * sc, 0.8 * sc, 0.54 * sc, fill=BLUE, shape=SH.ROUNDED_RECTANGLE); b_.adjustments[0] = 0.3
    c_ = rect(cx - 0.2 * sc, cy - 0.14 * sc, 0.4 * sc, 0.28 * sc, fill=TEAL, shape=SH.ROUNDED_RECTANGLE); c_.adjustments[0] = 0.3

y0 = TOP
p = panel(XL, y0, PW)
y = phead(XL, y0, PW, 1, "Motivation")
y = pbullets(XL + PAD, y, IW, [
    [("Autopilot firmware is safety-critical. ", {"bold": True}),
     ("PX4: about 710,000 lines of C/C++, all modules in one address space (no MMU).", {})],
    "A memory or arithmetic error in one driver can corrupt flight-critical state.",
    [("Gap: ", {"bold": True, "color": BLUE}), ("production PX4 driver code is still little examined by software verifiers.", {})],
], size=BODY - 2)
y += 0.15
dx = XL + PAD
bw_ = 2.55; gp_ = (IW - 5 * bw_) / 4; bh_ = 1.45
text(dx + (bw_ + gp_) - 0.1, y, 3 * bw_ + 2 * gp_ + 0.2, 0.55, "one shared address space",
     size=26, bold=True, color=TEAL, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
cont = rect(dx + (bw_ + gp_) - 0.25, y + 0.55, 3 * bw_ + 2 * gp_ + 0.5, bh_ + 0.5, line=TEAL, lw=3,
            shape=SH.ROUNDED_RECTANGLE); cont.adjustments[0] = 0.08
cont.line.dash_style = MSO_LINE.DASH
by_ = y + 0.8
labs_ = ["Sensors", "PX4\ndrivers", "uORB\nbus", "Flight\ncontrol"]
for k_, lab_ in enumerate(labs_):
    bx_ = dx + k_ * (bw_ + gp_)
    box(bx_, by_, bw_, bh_, lab_, size=30, fill=PALE if k_ else PANEL, line=BLUE if k_ else LINE)
    rarrow(bx_ + bw_ + 0.06, by_ + bh_ / 2, w=gp_ - 0.12, h=0.5)
bx_ = dx + 4 * (bw_ + gp_)
box(bx_, by_, bw_, bh_, "Vehicle\n(motors)", size=30, fill=PANEL, line=LINE)
# retroalimentacao (malha de controle): linha de volta do veiculo aos sensores
ly_ = by_ + bh_ + 0.32
rect(bx_ + bw_ / 2 - 0.02, by_ + bh_, 0.04, 0.34, fill=NAVY)
rect(dx + bw_ / 2 - 0.02, ly_ - 0.02, (bx_ + bw_ / 2) - (dx + bw_ / 2) + 0.04, 0.04, fill=NAVY)
rect(dx + bw_ / 2 - 0.02, by_ + bh_ + 0.12, 0.04, 0.22, fill=NAVY)
uarrow = rect(dx + bw_ / 2 - 0.14, by_ + bh_ + 0.02, 0.28, 0.2, fill=NAVY, shape=SH.ISOSCELES_TRIANGLE)
text(dx + 2 * (bw_ + gp_) - 0.3, ly_ + 0.02, 2 * bw_ + 0.6, 0.45, "feedback loop", size=22, color=MUTE, align=PP_ALIGN.CENTER)
y = by_ + bh_ + 0.85
text(dx, y, IW, 0.65, "An unsigned edge case (GPS harness)", size=SMALL, bold=True, color=BLUE)
y += 0.7
im_ = Image.open(f"{P}/build/eq2_t.png"); w_eq = 11.0; h_eq = w_eq * im_.height / im_.width
pic(f"{P}/build/eq2_t.png", dx + (IW - w_eq) / 2, y, w=w_eq)
y += h_eq + 0.35
text(dx, y, IW, 0.65, "How BMC finds it: every execution up to bound k", size=SMALL, bold=True, color=BLUE)
y += 0.7
im_ = Image.open(f"{P}/build/eq1_t.png"); w_eq = 13.4; h_eq = w_eq * im_.height / im_.width
pic(f"{P}/build/eq1_t.png", dx + (IW - w_eq) / 2, y, w=w_eq)
y += h_eq + 0.3
y1 = close_panel(p, y0, y) + 0.45

# ---- 2 Approach and Uniqueness
y0 = y1
p = panel(XL, y0, PW)
y = phead(XL, y0, PW, 2, "Approach and Uniqueness")
kb = rect(XL + PAD, y, IW, 2.8, fill=PALE, line=BLUE, lw=2.5, shape=SH.ROUNDED_RECTANGLE); kb.adjustments[0] = 0.1
text(XL + PAD + 0.45, y + 0.15, IW - 0.9, 2.5,
     [[("Bounded model checking ", {"bold": True, "color": BLUE}),
       ("of 12 PX4 modules (7 input classes) through hardware-free harnesses with datasheet-derived assertions (ESBMC, cross-checked with CBMC).", {})]],
     size=BODY - 2, anchor=MSO_ANCHOR.MIDDLE)
y += 2.8 + 0.45
X0 = XL + PAD
# fluxo (o "desenho" da metodologia)
b1w = 5.4
box(X0, y, b1w, 1.6, "PX4 module source\n(stage 1)", size=30)
text(X0 + b1w, y, 0.9, 1.6, "+", size=60, bold=True, color=BLUE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
box(X0 + b1w + 0.9, y, IW - b1w - 0.9, 1.6, "Harness: hardware stubs +\ndatasheet assertions (2–3)", size=30)
y += 1.6
darrow(X0 + IW / 2, y + 0.1)
y += 0.7
box(X0, y, IW, 1.3, "ESBMC run (4), CBMC cross-check (7)", fill=BLUE, line=BLUE2, color=WHITE, size=32)
y += 1.3
lw_ = 6.9; rw_ = IW - lw_ - 0.5
darrow(X0 + lw_ / 2, y + 0.1); darrow(X0 + lw_ + 0.5 + rw_ / 2, y + 0.1)
y += 0.7
box(X0, y, lw_, 1.5, "No violation within bound k\n= verified (model)", fill=RGBColor(0xDF, 0xF0, 0xE0), line=GREEN, color=GREEN, size=30)
box(X0 + lw_ + 0.5, y, rw_, 1.5, "Counterexample\n(violation in the model)", fill=RGBColor(0xFB, 0xEA, 0xD2), line=AMBER, color=AMBER, size=30)
y += 1.5
darrow(X0 + lw_ + 0.5 + rw_ / 2, y + 0.1)
y += 0.7
box(X0, y, lw_, 1.6, "ArduPilot check\nand CI (8)", fill=PANEL, line=LINE, color=MUTE, size=28)
box(X0 + lw_ + 0.5, y, rw_, 1.6, "Check vs. production\ncode, report to PX4 (5–6)", size=29)
y += 1.6
y2 = close_panel(p, y0, y) + 0.45

y0 = y2
p = panel(XL, y0, PW)
QR = 2.6
y = y0 + 0.55
img = qrcode.make("https://doi.org/10.5281/zenodo.23282236", box_size=20, border=2)
img.save(f"{P}/build/qr.png")
pic(f"{P}/build/qr.png", XL + PAD + IW - QR, y, w=QR)
text(XL + PAD, y, IW - QR - 0.5, QR,
     [[("Artifact", {"bold": True, "color": BLUE, "size": 44})],
      [("Harnesses and logs. Scan the code or visit", {})],
      [("doi.org/10.5281/zenodo.23282236", {"bold": True, "color": BLUE})],
      [("Supported by CAPES (Finance Code 001) and FAPEAM; part of the SAGRES Project.", {"size": 24, "color": MUTE})]],
     size=32, color=INK, line_spacing=1.1, space_after=8)
y += QR
y3 = close_panel(p, y0, y)
YL_END = y3

# =============================================================== coluna direita
GAPV = 0.6
XI = XR + PAD
y0 = TOP
p = panel(XR, y0, PW)
y = phead(XR, y0, PW, 3, "Results")
tile2(XI, y, tw3, 3.0, "10 of 12", "harnesses with no violation found", GREEN, bigsize=66, smallsize=32)
tile2(XI + tw3 + 0.4, y, tw3, 3.0, "3", "counterexamples, 0 confirmed bugs", AMBER, bigsize=130, smallsize=32)
tile2(XI + 2 * (tw3 + 0.4), y, tw3, 3.0, "\u2264 65 s", "verification time per harness", NAVY, bigsize=84, smallsize=32)
y += 3.0 + 0.5
text(XI, y, IW, 0.9, "Verification time per harness (s, median of 3 runs)", size=38, bold=True, color=BLUE)
y += 1.05
data = [("GPS uBlox", 64.9, 1), ("MAVLink Parser", 49.2, 0), ("EKF2 Covariance", 12.2, 0), ("RC Update", 9.5, 0),
        ("Battery Monitor", 6.7, 0), ("Magnetometer", 6.6, 0), ("Expo RC", 4.9, 0), ("VTOL", 4.0, 0),
        ("Barometer", 0.4, 0), ("PWM Output", 0.2, 0), ("IMU BMI088", 0.1, 1), ("Navigator/Mission", 0.1, 0)]
cd = CategoryChartData()
cd.categories = [d[0] for d in reversed(data)]
cd.add_series("Time (s)", [d[1] for d in reversed(data)])
CH = 7.0
gf = sl.shapes.add_chart(XL_CHART_TYPE.BAR_CLUSTERED, Inches(XI), Inches(y), Inches(IW), Inches(CH), cd)
gf.name = "Time chart"
ch = gf.chart
ch.has_legend = False; ch.has_title = False
ch.font.size = Pt(34); ch.font.name = FONT; ch.font.color.rgb = INK
pl = ch.plots[0]; pl.gap_width = 35
pl.has_data_labels = True
dl = pl.data_labels; dl.number_format = '0.0'; dl.number_format_is_linked = False
dl.position = XL_LABEL_POSITION.OUTSIDE_END; dl.font.size = Pt(34); dl.font.bold = True
ser = pl.series[0]
for i, d in enumerate(reversed(data)):
    pt = ser.points[i]; pt.format.fill.solid()
    pt.format.fill.fore_color.rgb = AMBER if d[2] else GREEN
va = ch.value_axis; va.maximum_scale = 80; va.minimum_scale = 0; va.major_unit = 20
va.has_major_gridlines = True; va.major_gridlines.format.line.color.rgb = LINE
va.format.line.fill.background(); va.tick_labels.font.size = Pt(30)
ca = ch.category_axis; ca.format.line.color.rgb = LINE; ca.tick_labels.font.size = Pt(34)
y += CH + 0.05
rect(XI + 0.2, y + 0.12, 0.6, 0.48, fill=GREEN)
text(XI + 1.0, y, 5.2, 0.72, "no violation", size=32, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
rect(XI + 6.3, y + 0.12, 0.6, 0.48, fill=AMBER)
text(XI + 7.1, y, IW - 7.1, 0.72, "counterexample (stops at 1st violation)", size=32, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
y += 0.8
y4 = close_panel(p, y0, y) + GAPV

# ---- 4 Os 3 contraexemplos: tres linhas bem definidas
y0 = y4
p = panel(XR, y0, PW)
y = phead(XR, y0, PW, 4, "The 3 counterexamples")
cm_w = 3.9; co_w = 4.2
xm, xp, xo = XI, XI + cm_w, XI + IW - co_w
for xx, ww, lab in ((xm + 0.35, cm_w, "Module"), (xp, xo - xp - 0.2, "Problem identified"), (xo, co_w, "Analysis result")):
    text(xx, y, ww, 0.6, lab, size=28, bold=True, color=MUTE)
y += 0.7
rows = [("GPS uBlox", "Out-of-bounds copy in the model; unreachable in production", ["False", "positive"]),
        ("BMI088 FIFO", "Harness omitted 3 guards present in the driver", ["False", "positive"]),
        ("BMI088 temperature", "86.25 \u00b0C above the 85 \u00b0C datasheet limit; PX4 declined it", ["Disputed", "specification"])]
RHT = 1.8
for mod, prob, res in rows:
    rr = rect(XI, y, IW, RHT, fill=PANEL, shape=SH.ROUNDED_RECTANGLE); rr.adjustments[0] = 0.08
    rect(XI, y + 0.22, 0.2, RHT - 0.44, fill=AMBER)
    text(xm + 0.45, y, cm_w - 0.5, RHT, mod, size=36, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
    text(xp, y, xo - xp - 0.3, RHT, prob, size=32, color=INK, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.05)
    pill = rect(xo, y + 0.3, co_w - 0.2, RHT - 0.6, fill=AMBER, shape=SH.ROUNDED_RECTANGLE); pill.adjustments[0] = 0.3
    text(xo, y + 0.3, co_w - 0.2, RHT - 0.6, res, size=32, bold=True, color=WHITE, align=PP_ALIGN.CENTER,
         anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
    y += RHT + 0.25
y -= 0.25
y5 = close_panel(p, y0, y) + GAPV

# ---- 5 Contribuicoes em destaque
y0 = y5
p = panel(XR, y0, PW)
y = phead(XR, y0, PW, 5, "Contributions")
cards = [("8-stage methodology", "Hardware stubs, datasheet assertions, cross-tool checks."),
         ("Feasible on production PX4 code", "10 of 12 harnesses clean within the bound."),
         ("Model-level evidence, not a proof", "Next: k-induction, failsafe and geofence modules, HIL.")]
CHT = 1.8
for k_, (t_, d_) in enumerate(cards, 1):
    cb = rect(XI, y, IW, CHT, fill=PALE, line=None, shape=SH.ROUNDED_RECTANGLE); cb.adjustments[0] = 0.08
    rect(XI, y + 0.22, 0.2, CHT - 0.44, fill=TEAL)
    rect(XI + 0.5, y + (CHT - 1.0) / 2, 1.0, 1.0, fill=TEAL, shape=SH.OVAL)
    text(XI + 0.5, y + (CHT - 1.0) / 2, 1.0, 1.0, str(k_), size=40, bold=True, color=WHITE,
         align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(XI + 1.85, y + 0.1, IW - 2.1, CHT - 0.2,
         [[(t_, {"bold": True, "size": 40, "color": NAVY})], [(d_, {"size": 32})]],
         size=32, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.05, space_after=4)
    y += CHT + 0.25
y -= 0.25
y6 = close_panel(p, y0, y)
YR_END = y6

# ---------------------------------------------------------------- rodape (logos maiores e padronizados)
FH = 2.7
FY = H - M - FH
fb = rect(M, FY, W - 2 * M, FH, fill=WHITE, shape=SH.ROUNDED_RECTANGLE); fb.adjustments[0] = 0.07
groups = [("Organization", ["l01", "l02"]), ("Promotion", ["l11"]), ("Funding Agencies", ["l04", "l05"]),
          ("Support", ["l08", "l07"]), ("Sponsors", ["l06", "hwit"]), ("Author’s institution", ["ufam"])]
LH = 1.6                               # altura padrao dos logos
hmap = {"l04": 1.1, "l07": 1.0, "l06": 1.65}   # logos muito largos, para manter a largura visual
widths = []
for g, ls in groups:
    ws = []
    for l in ls:
        im = Image.open(f"{P}/logos/{l}.png"); hh = hmap.get(l, LH); ws.append(hh * im.width / im.height)
    widths.append(ws)
inner = 0.5
gw = [max(sum(ws) + inner * (len(ws) - 1), 0.5) for ws in widths]
IN = 1.0
space = (W - 2 * M - 2 * IN - sum(gw)) / (len(groups) - 1)
x = M + IN
for (g, ls), ws, gwid in zip(groups, widths, gw):
    text(x - 1.2, FY + 0.2, gwid + 2.4, 0.5, g, size=22, bold=True, color=MUTE, align=PP_ALIGN.CENTER)
    xx = x
    for l, w_ in zip(ls, ws):
        hh = hmap.get(l, LH)
        pic(f"{P}/logos/{l}.png", xx, FY + 0.85 + (LH - hh) / 2, h=hh)   # centralizados na mesma linha
        xx += w_ + inner
    x += gwid + space

print("fim esquerda %.2f | fim direita %.2f | rodape %.2f | pagina %.2f" % (YL_END, YR_END, FY, H))
prs.save(OUT)
