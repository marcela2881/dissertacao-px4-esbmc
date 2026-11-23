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
# ===== v8: paleta sobria na identidade do SBESC (turquesa + cinza)
TURQ = RGBColor(0x00, 0xB3, 0xBE)
DTURQ = RGBColor(0x00, 0x7A, 0x83)
DGRAY = RGBColor(0x33, 0x33, 0x33)
MGRAY = RGBColor(0x6B, 0x6B, 0x6B)
LGRAY = RGBColor(0xF2, 0xF2, 0xF2)
NAVY = DGRAY; BLUE = DGRAY; TEAL = TURQ; GREEN = DTURQ; AMBER = MGRAY
PALE = LGRAY; PANEL = LGRAY; INK = DGRAY; MUTE = MGRAY
bg.fill.fore_color.rgb = WHITE
HB = 5.6                                   # altura do cabecalho
rect(0, 0, W, HB, fill=TURQ, name="Header")
# logos em cartoes brancos
rect(M, 0.9, 4.5, 4.5, fill=WHITE, shape=SH.OVAL)
pic(f"{P}/logos/ufam_round.png", M + 0.12, 1.02, h=4.26)
cw_ = 6.2
rect(W - M - cw_, 0.9, cw_, 3.3, fill=WHITE, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.08
pic(f"{P}/logos/l10.png", W - M - cw_ + 0.3, 1.05, w=cw_ - 0.6)
text(W - M - cw_, 4.35, cw_, 1.2,
     ["XVI Symposium on Computing Systems Engineering", "Nov 24–27, 2026 · Salvador, BA"],
     size=24, color=WHITE, align=PP_ALIGN.CENTER, line_spacing=1.05)
tx, tw = M + 5.2, W - 2 * M - 5.2 - 6.9
text(tx, 0.45, tw, 2.9,
     "Formal Verification of Critical Modules in the PX4 Autopilot using Bounded Model Checking",
     size=68, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0, name="Title")
text(tx, 3.55, tw, 0.8,
     "Marcela Alves Rodrigues · Lucas C. Cordeiro · José Reginaldo H. de Carvalho",
     size=38, bold=True, color=WHITE, align=PP_ALIGN.CENTER, name="Authors")
text(tx, 4.4, tw, 1.0,
     ["Graduate Program in Electrical Engineering (PPGEE) · Institute of Computing",
      "Federal University of Amazonas (UFAM), Manaus, Brazil"],
     size=28, color=WHITE, align=PP_ALIGN.CENTER, name="Affiliation")

TOP = HB + 0.18 + 0.8
PW = CW
PAD = 0.75
IW = PW - 2 * PAD
GP = 0.7
FH = 3.0
FY = H - 0.8 - FH
END = FY - 0.7                              # limite do conteudo


def card(x, y, w, h, num, title):
    c = rect(x, y, w, h, fill=WHITE, line=LINE, shape=SH.ROUNDED_RECTANGLE, lw=1.5, name="Card")
    c.adjustments[0] = 0.012
    text(x + PAD, y + 0.55, w - 2 * PAD, 1.3, [[(f"{num}  ", {"color": TURQ}), (title, {})]],
         size=64, bold=True, color=DTURQ, anchor=MSO_ANCHOR.MIDDLE)
    rect(x + PAD, y + 1.8, 5.5, 0.1, fill=TEAL)
    return y + 2.1


# ---------------------------------------------------------------- coluna esquerda
# 1 Motivation
y0 = TOP
h1 = 17.3
y = card(XL, y0, PW, h1, 1, "Motivation")
X0 = XL + PAD
y = pbullets(X0, y, IW, [
    [("Autopilot firmware is safety-critical. ", {"bold": True}),
     ("PX4: about 710,000 lines of C/C++, all modules in one address space (no MMU).", {})],
    "A memory or arithmetic error in one driver can corrupt flight-critical state.",
    [("Gap: ", {"bold": True, "color": TEAL}), ("production PX4 driver code is still little examined by software verifiers.", {})],
], size=BODY, sa=16)
y += 0.35
# malha de controle (estilo do poster do orientador)
bw_ = 2.5; gp_ = (IW - 5 * bw_) / 4; bh_ = 1.5
cont = rect(X0 + (bw_ + gp_) - 0.22, y + 0.5, 3 * bw_ + 2 * gp_ + 0.44, bh_ + 0.5, line=TEAL, lw=3,
            shape=SH.ROUNDED_RECTANGLE); cont.adjustments[0] = 0.08
cont.line.dash_style = MSO_LINE.DASH
text(X0 + (bw_ + gp_) - 0.2, y, 3 * bw_ + 2 * gp_ + 0.4, 0.5, "one shared address space",
     size=26, bold=True, color=TEAL, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
by_ = y + 0.75
for k_, lab_ in enumerate(["Sensors", "PX4\ndrivers", "uORB\nbus", "Flight\ncontrol", "Vehicle\n(motors)"]):
    bx_ = X0 + k_ * (bw_ + gp_)
    edge = k_ in (0, 4)
    box(bx_, by_, bw_, bh_, lab_, size=30, fill=PANEL if edge else PALE, line=LINE if edge else BLUE)
    if k_ < 4: rarrow(bx_ + bw_ + 0.05, by_ + bh_ / 2, w=gp_ - 0.1, h=0.5)
ly_ = by_ + bh_ + 0.42
rect(X0 + 4 * (bw_ + gp_) + bw_ / 2 - 0.02, by_ + bh_, 0.04, 0.44, fill=NAVY)
rect(X0 + bw_ / 2 - 0.02, ly_ - 0.02, 4 * (bw_ + gp_) + 0.04, 0.04, fill=NAVY)
rect(X0 + bw_ / 2 - 0.02, by_ + bh_ + 0.2, 0.04, 0.24, fill=NAVY)
rect(X0 + bw_ / 2 - 0.15, by_ + bh_ + 0.03, 0.3, 0.22, fill=NAVY, shape=SH.ISOSCELES_TRIANGLE)
text(X0 + 2 * (bw_ + gp_), ly_ + 0.08, bw_, 0.45, "feedback loop", size=22, color=MUTE, align=PP_ALIGN.CENTER)
y = ly_ + 0.9
text(X0, y, IW, 0.65, "An unsigned edge case (GPS harness)", size=SMALL, bold=True, color=NAVY)
y += 0.75
im_ = Image.open(f"{P}/build/eq2_t.png"); w_eq = 12.2; h_eq = w_eq * im_.height / im_.width
pic(f"{P}/build/eq2_t.png", X0 + (IW - w_eq) / 2, y, w=w_eq)
y += h_eq + 0.4
text(X0, y, IW, 0.65, "How BMC finds it: every execution up to bound k", size=SMALL, bold=True, color=NAVY)
y += 0.75
im_ = Image.open(f"{P}/build/eq1_t.png"); w_eq = 14.2; h_eq = w_eq * im_.height / im_.width
pic(f"{P}/build/eq1_t.png", X0 + (IW - w_eq) / 2, y, w=w_eq)
y += h_eq
print("motivacao precisa de", round(y + 0.5 - y0, 2), "tem", h1)
y1 = y0 + h1 + GP

# 2 Approach
y0 = y1
h2 = 14.4
y = card(XL, y0, PW, h2, 2, "Approach and Uniqueness")
kb = rect(X0, y, IW, 2.4, fill=PALE, line=None, shape=SH.ROUNDED_RECTANGLE); kb.adjustments[0] = 0.08
rect(X0, y + 0.25, 0.2, 1.9, fill=TEAL)
text(X0 + 0.6, y + 0.1, IW - 1.0, 2.2,
     [[("Bounded model checking ", {"bold": True, "color": NAVY}),
       ("of 12 PX4 modules (7 input classes) through hardware-free harnesses with datasheet-derived assertions (ESBMC, cross-checked with CBMC).", {})]],
     size=BODY - 4, anchor=MSO_ANCHOR.MIDDLE)
y += 2.4 + 0.45
b1w = (IW - 1.0) / 2
box(X0, y, b1w, 1.6, "PX4 module source\n(stage 1)", size=32)
text(X0 + b1w, y, 1.0, 1.6, "+", size=64, bold=True, color=NAVY, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
box(X0 + b1w + 1.0, y, b1w, 1.6, "Harness: hardware stubs +\ndatasheet assertions (2–3)", size=32)
y += 1.6
darrow(X0 + IW / 2, y + 0.12, h=0.6)
y += 0.85
box(X0, y, IW, 1.4, "ESBMC run (4)  ·  CBMC cross-check (7)", size=34, fill=NAVY, line=NAVY, color=WHITE)
y += 1.4
lw_ = (IW - 0.8) / 2
darrow(X0 + lw_ / 2, y + 0.12, h=0.6); darrow(X0 + lw_ + 0.8 + lw_ / 2, y + 0.12, h=0.6)
y += 0.85
box(X0, y, lw_, 1.7, "No violation within\nbound k = verified (model)", size=30,
    fill=LGRAY, line=GREEN, color=GREEN)
box(X0 + lw_ + 0.8, y, lw_, 1.7, "Counterexample\n(violation in the model)", size=30,
    fill=LGRAY, line=AMBER, color=AMBER)
y += 1.7
darrow(X0 + lw_ + 0.8 + lw_ / 2, y + 0.12, h=0.6)
y += 0.85
box(X0 + lw_ + 0.8, y, lw_, 1.6, "Check vs. production code,\nreport to PX4 (5–6)", size=28)
y += 1.6
print("abordagem precisa de", round(y + 0.5 - y0, 2), "tem", h2)
y2 = y0 + h2 + GP

# 3 Artifact
y0 = y2
h3 = END - y0
c = rect(XL, y0, PW, h3, fill=NAVY, shape=SH.ROUNDED_RECTANGLE, name="Artifact card"); c.adjustments[0] = 0.04
QR = h3 - 0.8
img = qrcode.make("https://doi.org/10.5281/zenodo.23282236", box_size=20, border=2)
img.save(f"{P}/build/qr.png")
rect(XL + PW - PAD - QR - 0.15, y0 + 0.35, QR + 0.3, QR + 0.3, fill=WHITE, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.06
pic(f"{P}/build/qr.png", XL + PW - PAD - QR, y0 + 0.5, w=QR)
text(X0, y0 + 0.5, IW - QR - 0.7, QR,
     [[("Artifact", {"bold": True, "size": 54, "color": WHITE})],
      [("Harnesses and logs: scan the code or visit", {})],
      [("doi.org/10.5281/zenodo.23282236", {"bold": True})],
      [("CAPES (Finance Code 001), FAPEAM · SAGRES Project", {"size": 24, "color": WHITE})]],
     size=30, color=WHITE, line_spacing=1.12, space_after=6, anchor=MSO_ANCHOR.MIDDLE)
YL = y0 + h3

# ---------------------------------------------------------------- coluna direita
XR_ = XR + PAD
y0 = TOP
h4 = 14.95
y = card(XR, y0, PW, h4, 3, "Results")
tw3 = (IW - 0.8) / 3
TH = 2.8
tile2(XR_, y, tw3, TH, "10 of 12", "harnesses with no violation found", GREEN, bigsize=66, smallsize=30)
tile2(XR_ + tw3 + 0.4, y, tw3, TH, "3", "counterexamples, 0 confirmed bugs", AMBER, bigsize=130, smallsize=30)
tile2(XR_ + 2 * (tw3 + 0.4), y, tw3, TH, "≤ 65 s", "verification time per harness", NAVY, bigsize=84, smallsize=30)
y += TH + 0.6
text(XR_, y, IW, 0.8, "Verification time per harness (s, median of 3 runs)", size=36, bold=True, color=NAVY)
y += 1.05
data = [("GPS uBlox", 64.9, 1), ("MAVLink Parser", 49.2, 0), ("EKF2 Covariance", 12.2, 0), ("RC Update", 9.5, 0),
        ("Battery Monitor", 6.7, 0), ("Magnetometer", 6.6, 0), ("Expo RC", 4.9, 0), ("VTOL", 4.0, 0),
        ("Barometer", 0.4, 0), ("PWM Output", 0.2, 0), ("IMU BMI088", 0.1, 1), ("Navigator/Mission", 0.1, 0)]
LW_ = 5.6; BX = XR_ + LW_ + 0.2; BWMAX = IW - LW_ - 0.2 - 2.0; RH = 0.58
for i, (nm, v, ce) in enumerate(data):
    yy = y + i * RH
    if i % 2 == 0: rect(XR_, yy, IW, RH, fill=LGRAY)
    text(XR_ + 0.1, yy, LW_ - 0.1, RH, nm, size=30, color=INK, align=PP_ALIGN.RIGHT, anchor=MSO_ANCHOR.MIDDLE)
    bw = max(BWMAX * v / 65.0, 0.08)
    rect(BX, yy + 0.1, bw, RH - 0.2, fill=AMBER if ce else GREEN)
    text(BX + bw + 0.15, yy, 2.0, RH, f"{v:.1f}", size=30, bold=True, color=INK, anchor=MSO_ANCHOR.MIDDLE)
y += len(data) * RH + 0.25
rect(BX, y + 0.12, 0.5, 0.4, fill=GREEN)
text(BX + 0.65, y, 4.0, 0.64, "no violation", size=28, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
rect(BX + 4.6, y + 0.12, 0.5, 0.4, fill=AMBER)
text(BX + 5.25, y, 5.0, 0.64, "counterexample", size=28, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
y += 0.7
print("resultados precisa de", round(y + 0.5 - y0, 2), "tem", h4)
y3 = y0 + h4 + GP

# 4 Counterexamples
y0 = y3
h5 = 9.5
y = card(XR, y0, PW, h5, 4, "The 3 counterexamples")
cm_w = 4.4; co_w = 4.6
xm, xp, xo = XR_, XR_ + cm_w + 0.1, XR_ + IW - co_w
for xx, ww, lab in ((xm + 0.5, cm_w, "MODULE"), (xp, xo - xp - 0.2, "PROBLEM"), (xo + 0.2, co_w, "ANALYSIS RESULT")):
    text(xx, y, ww, 0.6, lab, size=26, bold=True, color=MUTE)
y += 0.7
rows = [("GPS uBlox", "Out-of-bounds copy in the model; unreachable in production", ["False", "positive"]),
        ("BMI088 FIFO", "Harness omitted 3 guards present in the driver", ["False", "positive"]),
        ("BMI088 temperature", "86.25 °C vs. 85 °C datasheet limit; PX4 declined it", ["Disputed", "specification"])]
RHT = 1.85
for mod, prob, res in rows:
    rr = rect(XR_, y, IW, RHT, fill=PANEL, shape=SH.ROUNDED_RECTANGLE); rr.adjustments[0] = 0.08
    rect(XR_, y + 0.22, 0.2, RHT - 0.44, fill=AMBER)
    text(xm + 0.5, y, cm_w - 0.5, RHT, mod, size=36, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
    text(xp, y, xo - xp - 0.3, RHT, prob, size=30, color=INK, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.05)
    pill = rect(xo + 0.2, y + 0.3, co_w - 0.4, RHT - 0.6, fill=AMBER, shape=SH.ROUNDED_RECTANGLE); pill.adjustments[0] = 0.3
    text(xo + 0.2, y + 0.3, co_w - 0.4, RHT - 0.6, res, size=30, bold=True, color=WHITE, align=PP_ALIGN.CENTER,
         anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
    y += RHT + 0.3
y -= 0.3
print("contraexemplos precisa de", round(y + 0.5 - y0, 2), "tem", h5)
y4 = y0 + h5 + GP

# 5 Contributions
y0 = y4
h6 = END - y0
y = card(XR, y0, PW, h6, 5, "Contributions")
cards = [("8-stage methodology", "Hardware stubs, datasheet assertions, cross-tool checks."),
         ("Feasible on production PX4 code", "10 of 12 harnesses clean within the bound."),
         ("Model-level evidence, not a proof", "Next: k-induction, failsafe and geofence modules, HIL.")]
avail = y0 + h6 - 0.6 - y
CHT = (avail - 2 * 0.3) / 3
for k_, (t_, d_) in enumerate(cards, 1):
    cb = rect(XR_, y, IW, CHT, fill=PALE, shape=SH.ROUNDED_RECTANGLE); cb.adjustments[0] = 0.08
    rect(XR_, y + 0.22, 0.2, CHT - 0.44, fill=TEAL)
    rect(XR_ + 0.6, y + (CHT - 1.0) / 2, 1.0, 1.0, fill=TEAL, shape=SH.OVAL)
    text(XR_ + 0.6, y + (CHT - 1.0) / 2, 1.0, 1.0, str(k_), size=40, bold=True, color=WHITE,
         align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(XR_ + 2.0, y + 0.1, IW - 2.3, CHT - 0.2,
         [[(t_, {"bold": True, "size": 40, "color": NAVY})], [(d_, {"size": 32})]],
         size=32, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.05, space_after=4)
    y += CHT + 0.3
YR = y0 + h6

# ---------------------------------------------------------------- rodape
fb = rect(M, FY, W - 2 * M, FH, fill=WHITE, line=LINE, shape=SH.ROUNDED_RECTANGLE); fb.adjustments[0] = 0.05
groups = [("Organization", ["l01", "l02"]), ("Promotion", ["l11"]), ("Funding Agencies", ["l04", "l05"]),
          ("Support", ["l08", "l07"]), ("Sponsors", ["l06", "hwit"]), ("Author’s institution", ["ufam"])]
LH = 1.9
hmap = {"l04": 1.2, "l07": 1.1, "l06": 1.7}
widths = []
for g, ls in groups:
    ws = []
    for l in ls:
        im = Image.open(f"{P}/logos/{l}.png"); hh = hmap.get(l, LH); ws.append(hh * im.width / im.height)
    widths.append(ws)
inner = 0.5
gw = [max(sum(ws) + inner * (len(ws) - 1), 0.5) for ws in widths]
IN = 1.2
space = (W - 2 * M - 2 * IN - sum(gw)) / (len(groups) - 1)
x = M + IN
for (g, ls), ws, gwid in zip(groups, widths, gw):
    text(x - 1.2, FY + 0.3, gwid + 2.4, 0.5, g, size=24, bold=True, color=MUTE, align=PP_ALIGN.CENTER)
    xx = x
    for l, w_ in zip(ls, ws):
        hh = hmap.get(l, LH)
        pic(f"{P}/logos/{l}.png", xx, FY + 1.05 + (LH - hh) / 2, h=hh)
        xx += w_ + inner
    x += gwid + space
print("fim esquerda %.2f | fim direita %.2f | limite %.2f" % (YL, YR, END))
prs.save(OUT)
