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
FX, FW = 0.7, 33.9
CW = (FW - 1.2 - 0.7) / 2
XL = FX + 0.6
XR = XL + CW + 0.7

prs = Presentation(f"{P}/modelo_oficial_SBESC.pptx")
sl = prs.slides[0]
for _sh in list(sl.shapes):
    if _sh.name == "Google Shape;57;p13":
        _sh._element.getparent().remove(_sh._element)
    elif _sh.name == "Google Shape;58;p13":
        _sh.text_frame.clear()


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
bg = rect(0, 0, 0.01, 0.01, fill=RGBColor(0xFF, 0xFF, 0xFF), name="Spacer")

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

HB = 4.9
text(FX + 0.6, 5.1, FW - 1.2, 2.2,
     "Formal Verification of Critical Modules in the PX4 Autopilot using Bounded Model Checking",
     size=60, bold=True, color=NAVY, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0, name="Title")
text(FX + 0.6, 7.35, FW - 1.2, 0.7, "Marcela Alves Rodrigues \u00b7 Lucas C. Cordeiro \u00b7 Jos\u00e9 Reginaldo H. de Carvalho",
     size=36, bold=True, color=BLUE, align=PP_ALIGN.CENTER, name="Authors")
text(FX + 0.6, 8.15, FW - 1.2, 0.5,
     "Graduate Program in Electrical Engineering (PPGEE) \u00b7 Institute of Computing \u00b7 Federal University of Amazonas (UFAM), Manaus, Brazil",
     size=24, color=MGRAY, align=PP_ALIGN.CENTER, name="Affiliation")
rect(FX + 0.6, 8.8, FW - 1.2, 0.05, fill=NAVY)
TOP = 9.2
PW = CW
PAD = 0.6
IW = PW - 2 * PAD
END = 41.0
X0L = XL + PAD
X0R = XR + PAD
GAPS = 0.5


def bar(x, y, w, num, title, fs=54):
    rect(x, y, w, 1.35, fill=NAVY, name="Section bar")
    rect(x, y, 1.35, 1.35, fill=BLUE)
    text(x, y, 1.35, 1.35, str(num), size=52, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(x + 1.6, y, w - 1.7, 1.35, title, size=fs, bold=True, color=WHITE, anchor=MSO_ANCHOR.MIDDLE)
    return y + 1.35 + 0.55


def sub(x, y, w, t, size=34):
    text(x, y, w, 0.75, t, size=size, bold=True, color=NAVY)
    return y + 0.85


def mark_bullets(x, y, w, items, size=34, sa=12):
    ps = [{"b": i} for i in items]
    h = est(ps, size, w, space_after=sa)
    text(x, y, w, h, ps, size=size, space_after=sa)
    return y + h


# ============================================================ v13: cartoes arredondados, fitas numeradas e icones
SOFT = RGBColor(0xC9, 0xDC, 0xEE)
CARDS = []


def section(x, y, w, num, title, fs=50):
    """fita numerada sobre um cartao; devolve o y do conteudo e registra o cartao para fechar depois"""
    card = rect(x, y + 0.65, w, 1.0, fill=WHITE, line=SOFT, lw=2.5, shape=SH.ROUNDED_RECTANGLE, name="Card")
    card.adjustments[0] = 0.02
    rb = rect(x + 0.9, y, w - 1.4, 1.3, fill=NAVY, shape=SH.ROUNDED_RECTANGLE, name="Ribbon"); rb.adjustments[0] = 0.5
    rect(x + 0.1, y - 0.1, 1.5, 1.5, fill=BLUE, line=WHITE, lw=4, shape=SH.OVAL)
    text(x + 0.1, y - 0.1, 1.5, 1.5, str(num), size=52, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(x + 2.0, y, w - 2.6, 1.3, title, size=fs, bold=True, color=WHITE, anchor=MSO_ANCHOR.MIDDLE)
    CARDS.append((card, y + 0.65))
    return y + 1.3 + 0.55


def endcard(y):
    card, y0 = CARDS[-1]
    card.height = Inches(y + 0.35 - y0)
    return y + 0.35


def icon_check(x, y, d, color=GREEN):
    rect(x, y, d, d, fill=color, shape=SH.OVAL)
    a = rect(x + d * 0.22, y + d * 0.50, d * 0.30, d * 0.10, fill=WHITE); a.rotation = 45
    b = rect(x + d * 0.30, y + d * 0.43, d * 0.58, d * 0.10, fill=WHITE); b.rotation = -50


def icon_warn(x, y, d, color=ORANGE):
    rect(x, y, d, d * 0.9, fill=color, shape=SH.ISOSCELES_TRIANGLE)
    text(x, y + d * 0.25, d, d * 0.7, "!", size=int(d * 45), bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)


def icon_clock(x, y, d, color=NAVY):
    rect(x, y, d, d, fill=None, line=color, lw=5, shape=SH.OVAL)
    rect(x + d / 2 - 0.03, y + d * 0.2, 0.06, d * 0.33, fill=color)
    rect(x + d / 2 - 0.03, y + d / 2 - 0.03, d * 0.28, 0.06, fill=color)


def tile3(x, y, w, h, big, small, color, icon, bigsize):
    rect(x, y, w, h, fill=LGRAY, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.08
    d = 0.9
    icon(x + 0.35, y + 0.3, d, color)
    text(x + 0.35 + d + 0.25, y + 0.15, w - d - 0.9, h * 0.5, big, size=bigsize, bold=True, color=color, anchor=MSO_ANCHOR.MIDDLE)
    text(x + 0.4, y + h * 0.58, w - 0.7, h * 0.4, small, size=26, color=MUTE, line_spacing=1.0)


GAPS = 0.6
# ---------------------------------------------------------------- esquerda
y = TOP
y = section(XL, y, PW, 1, "Introduction")
y = mark_bullets(X0L, y, IW, [
    [("Safety-critical firmware: ", {"bold": True}), ("PX4 has about 710,000 lines of C/C++ in one address space.", {})],
    "A fault in one driver can corrupt flight-critical state.",
    [("Gap: ", {"bold": True, "color": BLUE}), ("production driver code is rarely examined by verifiers.", {})],
], size=40, sa=14)
y += 0.4
bw_ = 2.45; gp_ = (IW - 5 * bw_) / 4; bh_ = 1.5
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
y = endcard(ly_ + 0.55) + GAPS

y = section(XL, y, PW, 2, "Proposal")
kb = rect(X0L, y, IW, 1.9, fill=LBLUE, shape=SH.ROUNDED_RECTANGLE); kb.adjustments[0] = 0.1
text(X0L + 0.5, y + 0.05, IW - 1.0, 1.8,
     [[("Bounded model checking ", {"bold": True, "color": NAVY}),
       ("of 12 PX4 modules with hardware-free harnesses and datasheet assertions.", {})]],
     size=34, anchor=MSO_ANCHOR.MIDDLE)
y += 1.9 + 0.45
b1w = (IW - 1.0) / 2
box(X0L, y, b1w, 1.3, "PX4 module source", size=28)
text(X0L + b1w, y, 1.0, 1.3, "+", size=56, bold=True, color=NAVY, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
box(X0L + b1w + 1.0, y, b1w, 1.3, "Harness: stubs + assertions", size=28)
y += 1.3
darrow(X0L + IW / 2, y + 0.08, h=0.5, color=NAVY); y += 0.68
box(X0L, y, IW, 1.2, "ESBMC run  ·  CBMC cross-check", size=32, fill=NAVY, line=NAVY, color=WHITE)
y += 1.2
lw_ = (IW - 0.8) / 2
darrow(X0L + lw_ / 2, y + 0.08, h=0.5, color=NAVY); darrow(X0L + lw_ + 0.8 + lw_ / 2, y + 0.08, h=0.5, color=NAVY)
y += 0.68
box(X0L, y, lw_, 1.4, "No violation within\nbound (verified)", size=26, fill=LGRAY, line=GREEN, color=GREEN)
box(X0L + lw_ + 0.8, y, lw_, 1.4, "Counterexample\n(violation in model)", size=26, fill=LGRAY, line=ORANGE, color=ORANGE)
y = endcard(y + 1.4) + GAPS

y = section(XL, y, PW, 3, "Modules Covered")
rows = [("Input class", "Modules"), ("Serial sensor", "GPS uBlox, IMU BMI088"), ("Telemetry", "MAVLink"),
        ("Radio", "Expo RC, RC Update"), ("Power sensor", "Battery Monitor"), ("Actuation", "PWM Output, VTOL"),
        ("Mission logic", "Navigator"), ("State estimation", "EKF2 Covariance"), ("Sensor aggregation", "Magnetometer, Barometer")]
y = table(X0L, y, [5.6, IW - 5.6], rows, size=27, rowh=0.62, hdr_fill=NAVY)
y = endcard(y)
print("fim esquerda %.2f limite %.2f" % (y, END))

# ---------------------------------------------------------------- direita
y = TOP
y = section(XR, y, PW, 4, "Results")
tw3 = (IW - 0.8) / 3
TH = 2.3
tile3(X0R, y, tw3, TH, "10 of 12", "harnesses with no violation", GREEN, icon_check, 44)
tile3(X0R + tw3 + 0.4, y, tw3, TH, "3", "counterexamples, 0 confirmed bugs", ORANGE, icon_warn, 80)
tile3(X0R + 2 * (tw3 + 0.4), y, tw3, TH, "≤ 65 s", "verification time per harness", NAVY, icon_clock, 52)
y += TH + 0.45
y = sub(X0R, y, IW, "Verification time per harness (s, median of 3 runs)", 30)
data = [("GPS uBlox", 64.9, 1), ("MAVLink Parser", 49.2, 0), ("EKF2 Covariance", 12.2, 0), ("RC Update", 9.5, 0),
        ("Battery Monitor", 6.7, 0), ("Magnetometer", 6.6, 0), ("Expo RC", 4.9, 0), ("VTOL", 4.0, 0),
        ("Barometer", 0.4, 0), ("PWM Output", 0.2, 0), ("IMU BMI088", 0.1, 1), ("Navigator/Mission", 0.1, 0)]
LW_ = 5.0; BX = X0R + LW_ + 0.2; BWMAX = IW - LW_ - 0.2 - 1.8; RH = 0.43
for i, (nm, v, ce) in enumerate(data):
    yy = y + i * RH
    if i % 2 == 0: rect(X0R, yy, IW, RH, fill=LGRAY)
    text(X0R + 0.1, yy, LW_ - 0.1, RH, nm, size=26, color=INK, align=PP_ALIGN.RIGHT, anchor=MSO_ANCHOR.MIDDLE)
    bw = max(BWMAX * v / 65.0, 0.08)
    rect(BX, yy + 0.09, bw, RH - 0.18, fill=ORANGE if ce else GREEN)
    text(BX + bw + 0.15, yy, 1.8, RH, f"{v:.1f}", size=26, bold=True, color=INK, anchor=MSO_ANCHOR.MIDDLE)
y += len(data) * RH + 0.15
rect(BX, y + 0.1, 0.45, 0.34, fill=GREEN); text(BX + 0.6, y, 3.8, 0.55, "no violation", size=24, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
rect(BX + 4.4, y + 0.1, 0.45, 0.34, fill=ORANGE); text(BX + 5.05, y, 5.0, 0.55, "counterexample", size=24, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
y = endcard(y + 0.6) + GAPS

y = section(XR, y, PW, 5, "Counterexamples")
RHT = 1.35
for mod, prob, res in [("GPS uBlox", "Out-of-bounds copy in the model only", "False positive"),
                       ("BMI088 FIFO", "Harness omitted 3 driver guards", "False positive"),
                       ("BMI088 temp.", "86.25 °C vs. 85 °C datasheet limit", "Disputed spec.")]:
    rr = rect(X0R, y, IW, RHT, fill=LGRAY, shape=SH.ROUNDED_RECTANGLE); rr.adjustments[0] = 0.1
    rect(X0R, y + 0.2, 0.18, RHT - 0.4, fill=ORANGE)
    text(X0R + 0.5, y, 4.3, RHT, mod, size=30, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
    text(X0R + 4.9, y, IW - 4.9 - 4.1, RHT, prob, size=24, color=INK, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
    pill = rect(X0R + IW - 3.9, y + 0.25, 3.6, RHT - 0.5, fill=ORANGE, shape=SH.ROUNDED_RECTANGLE); pill.adjustments[0] = 0.3
    text(X0R + IW - 3.9, y + 0.25, 3.6, RHT - 0.5, res, size=26, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    y += RHT + 0.2
y = endcard(y - 0.2) + GAPS

y = section(XR, y, PW, 6, "Conclusions / Future work", 46)
CHT = 1.15
for t_ in ["End-to-end methodology: stubs, assertions, cross-check",
           "Feasible on production PX4 code: 10 of 12 clean",
           "Model-level evidence, not a proof"]:
    icon_check(X0R, y + (CHT - 0.8) / 2, 0.8, BLUE)
    text(X0R + 1.2, y, IW - 1.2, CHT, t_, size=28, bold=False, color=INK, anchor=MSO_ANCHOR.MIDDLE)
    y += CHT + 0.1
text(X0R, y + 0.1, IW, 0.8, [[("Next: ", {"bold": True, "color": BLUE}), ("k-induction, failsafe and geofence modules, HIL.", {})]], size=28)
y = endcard(y + 0.9)
print("fim direita (sem refs) %.2f" % y)
ry = y + GAPS
hw = (PW - 0.6) / 2
yy1 = section(XR, ry, hw, 7, "References", 34)
text(XR + 0.5, yy1, hw - 1.0, 2.6,
     ["[1] Biere et al., TACAS, 1999.", "[2] Cordeiro et al., IEEE TSE, 2012.", "[3] Gadelha et al., STTT, 2017."],
     size=23, color=INK, space_after=6)
endcard(yy1 + 2.1)
xa = XR + hw + 0.6
yy2 = section(xa, ry, hw, 8, "Acknowledgment", 32)
QR = 2.4
text(xa + 0.5, yy2, hw - QR - 1.2, 2.6,
     [[("CAPES, FAPEAM and the SAGRES Project.", {})], [("Artifact: ", {"bold": True}), ("scan the code.", {"color": BLUE})]],
     size=23, color=INK, space_after=8)
qimg = qrcode.make("https://doi.org/10.5281/zenodo.23282236", box_size=20, border=1)
qimg.save(f"{P}/build/qr.png")
pic(f"{P}/build/qr.png", xa + hw - QR - 0.4, yy2 - 0.1, w=QR)
y = endcard(yy2 + 2.3)
print("fim direita %.2f limite %.2f" % (y, END))

# rodape: o do modelo oficial permanece; acrescenta-se a instituicao do aluno
text(20.0, 43.3, 2.4, 0.9, "Institution", size=28, align=PP_ALIGN.CENTER, color=INK)
pic(f"{P}/logos/ufam_round.png", 20.55, 44.65, h=1.55)
prs.save(OUT)
