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


# ============================================================ v12: enxuto (pouca informacao, fontes grandes)
GAPS = 0.7
# ---------------------------------------------------------------- esquerda
y = TOP
y = bar(XL, y, PW, 1, "Introduction")
y = mark_bullets(X0L, y, IW, [
    [("Safety-critical: ", {"bold": True}), ("PX4 runs on drones.", {})],
    "One faulty driver can corrupt flight state.",
    [("Gap: ", {"bold": True, "color": BLUE}), ("driver code is rarely verified.", {})],
], size=46, sa=16)
y += 0.5
bw_ = 2.55; gp_ = (IW - 5 * bw_) / 4; bh_ = 1.9
cont = rect(X0L + (bw_ + gp_) - 0.2, y + 0.55, 3 * bw_ + 2 * gp_ + 0.4, bh_ + 0.5, line=BLUE, lw=3, shape=SH.ROUNDED_RECTANGLE)
cont.adjustments[0] = 0.08; cont.line.dash_style = MSO_LINE.DASH
text(X0L + (bw_ + gp_) - 0.2, y, 3 * bw_ + 2 * gp_ + 0.4, 0.55, "one shared address space", size=28, bold=True,
     color=BLUE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
by_ = y + 0.8
for k_, lab_ in enumerate(["Sensors", "PX4\ndrivers", "uORB\nbus", "Flight\ncontrol", "Vehicle\n(motors)"]):
    bx_ = X0L + k_ * (bw_ + gp_); edge = k_ in (0, 4)
    box(bx_, by_, bw_, bh_, lab_, size=30, fill=LGRAY if edge else LBLUE, line=LINE if edge else BLUE)
    if k_ < 4: rarrow(bx_ + bw_ + 0.05, by_ + bh_ / 2, w=gp_ - 0.1, h=0.55, color=NAVY)
ly_ = by_ + bh_ + 0.45
rect(X0L + 4 * (bw_ + gp_) + bw_ / 2 - 0.025, by_ + bh_, 0.05, 0.47, fill=NAVY)
rect(X0L + bw_ / 2 - 0.025, ly_ - 0.025, 4 * (bw_ + gp_) + 0.05, 0.05, fill=NAVY)
rect(X0L + bw_ / 2 - 0.025, by_ + bh_ + 0.22, 0.05, 0.25, fill=NAVY)
rect(X0L + bw_ / 2 - 0.17, by_ + bh_ + 0.03, 0.34, 0.25, fill=NAVY, shape=SH.ISOSCELES_TRIANGLE)
text(X0L + 2 * (bw_ + gp_) - 0.5, ly_ + 0.1, bw_ + 1.0, 0.5, "feedback loop", size=24, color=MUTE, align=PP_ALIGN.CENTER)
y = ly_ + 0.9 + GAPS

y = bar(XL, y, PW, 2, "Proposal")
kb = rect(X0L, y, IW, 2.5, fill=LBLUE, shape=SH.ROUNDED_RECTANGLE); kb.adjustments[0] = 0.08
rect(X0L, y + 0.3, 0.2, 1.9, fill=BLUE)
text(X0L + 0.6, y + 0.1, IW - 1.0, 2.3,
     [[("Bounded model checking ", {"bold": True, "color": NAVY}), ("of 12 PX4 modules.", {})]],
     size=44, anchor=MSO_ANCHOR.MIDDLE)
y += 2.5 + 0.6
b1w = (IW - 1.0) / 2
box(X0L, y, b1w, 1.7, "PX4 module", size=34)
text(X0L + b1w, y, 1.0, 1.7, "+", size=64, bold=True, color=NAVY, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
box(X0L + b1w + 1.0, y, b1w, 1.7, "Harness", size=34)
y += 1.7
darrow(X0L + IW / 2, y + 0.1, h=0.6, color=NAVY); y += 0.8
box(X0L, y, IW, 1.5, "ESBMC  ·  CBMC", size=40, fill=NAVY, line=NAVY, color=WHITE)
y += 1.5
lw_ = (IW - 0.8) / 2
darrow(X0L + lw_ / 2, y + 0.1, h=0.6, color=NAVY); darrow(X0L + lw_ + 0.8 + lw_ / 2, y + 0.1, h=0.6, color=NAVY)
y += 0.8
box(X0L, y, lw_, 1.7, "Verified\n(within bound)", size=32, fill=LGRAY, line=GREEN, color=GREEN)
box(X0L + lw_ + 0.8, y, lw_, 1.7, "Counter-\nexample", size=32, fill=LGRAY, line=ORANGE, color=ORANGE)
y += 1.7 + GAPS

y = bar(XL, y, PW, 3, "Conclusions / Future work")
CHT = 1.6
for k_, t_ in enumerate(["End-to-end methodology", "Feasible on production PX4 code", "Model-level evidence, not a proof"], 1):
    rect(X0L, y, IW, CHT, fill=LBLUE, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.08
    rect(X0L + 0.35, y + (CHT - 1.0) / 2, 1.0, 1.0, fill=BLUE, shape=SH.OVAL)
    text(X0L + 0.35, y + (CHT - 1.0) / 2, 1.0, 1.0, str(k_), size=38, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(X0L + 1.8, y, IW - 2.0, CHT, t_, size=36, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE)
    y += CHT + 0.25
y += 0.1
text(X0L, y, IW, 1.0, [[("Next: ", {"bold": True, "color": BLUE}), ("k-induction and hardware-in-the-loop.", {})]], size=34)
y += 0.9
yL = y
print("fim esquerda %.2f limite %.2f" % (yL, END))

# ---------------------------------------------------------------- direita
y = TOP
y = bar(XR, y, PW, 4, "Results")
tw3 = (IW - 0.8) / 3
TH = 4.2
tile2(X0R, y, tw3, TH, "10 of 12", "harnesses clean", GREEN, bigsize=56, smallsize=30)
tile2(X0R + tw3 + 0.4, y, tw3, TH, "3", "counterexamples", ORANGE, bigsize=100, smallsize=30)
tile2(X0R + 2 * (tw3 + 0.4), y, tw3, TH, "≤ 65 s", "per harness", NAVY, bigsize=66, smallsize=30)
y += TH + 0.6
y = sub(X0R, y, IW, "Verification time (s, median of 3 runs)", 34)
data = [("GPS uBlox", 64.9, 1), ("MAVLink Parser", 49.2, 0), ("EKF2 Covariance", 12.2, 0), ("RC Update", 9.5, 0),
        ("Other 8 harnesses", 6.7, 0)]
LW_ = 5.6; BX = X0R + LW_ + 0.2; BWMAX = IW - LW_ - 0.2 - 1.9; RH = 1.3
for i, (nm, v, ce) in enumerate(data):
    yy = y + i * RH
    if i % 2 == 0: rect(X0R, yy, IW, RH, fill=LGRAY)
    text(X0R + 0.1, yy, LW_ - 0.1, RH, nm, size=32, color=INK, align=PP_ALIGN.RIGHT, anchor=MSO_ANCHOR.MIDDLE)
    bw = max(BWMAX * v / 65.0, 0.08)
    rect(BX, yy + 0.15, bw, RH - 0.3, fill=ORANGE if ce else GREEN)
    lab = ("≤ " if i == 4 else "") + f"{v:.1f}"
    text(BX + bw + 0.15, yy, 1.9, RH, lab, size=32, bold=True, color=INK, anchor=MSO_ANCHOR.MIDDLE)
y += len(data) * RH + 0.25
rect(BX, y + 0.12, 0.5, 0.4, fill=GREEN); text(BX + 0.65, y, 3.8, 0.65, "no violation", size=28, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
rect(BX + 4.4, y + 0.12, 0.5, 0.4, fill=ORANGE); text(BX + 5.05, y, 5.0, 0.65, "counterexample", size=28, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
y += 1.0
y = sub(X0R, y, IW, "The 3 counterexamples", 34)
RHT = 2.0
for mod, res in [("GPS uBlox", "False positive"), ("BMI088 FIFO", "False positive"), ("BMI088 temperature", "Disputed spec.")]:
    rr = rect(X0R, y, IW, RHT, fill=LGRAY, shape=SH.ROUNDED_RECTANGLE); rr.adjustments[0] = 0.08
    rect(X0R, y + 0.2, 0.2, RHT - 0.4, fill=ORANGE)
    text(X0R + 0.55, y, 7.0, RHT, mod, size=36, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE)
    pill = rect(X0R + IW - 5.6, y + 0.25, 5.3, RHT - 0.5, fill=ORANGE, shape=SH.ROUNDED_RECTANGLE); pill.adjustments[0] = 0.3
    text(X0R + IW - 5.6, y + 0.25, 5.3, RHT - 0.5, res, size=32, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    y += RHT + 0.22
print("fim resultados %.2f" % y)

# References | Acknowledgment (embaixo)
hw = (PW - 0.6) / 2
ry = END - 6.6
yy1 = bar(XR, ry, hw, 5, "References", 38)
text(XR + 0.1, yy1, hw - 0.2, 3.3,
     ["[1] Biere et al., TACAS, 1999.", "[2] Cordeiro et al., IEEE TSE, 2012.", "[3] Gadelha et al., STTT, 2017."],
     size=30, color=INK, space_after=14, line_spacing=1.05)
xa = XR + hw + 0.6
yy2 = bar(xa, ry, hw, 6, "Acknowledgment", 36)
QR = 3.4
text(xa + 0.1, yy2, hw - QR - 0.5, 3.3,
     [[("CAPES, FAPEAM, SAGRES Project", {})], [("Artifact: ", {"bold": True}), ("scan the code", {"color": BLUE})]],
     size=30, color=INK, space_after=14, line_spacing=1.05)
qimg = qrcode.make("https://doi.org/10.5281/zenodo.23282236", box_size=20, border=1)
qimg.save(f"{P}/build/qr.png")
pic(f"{P}/build/qr.png", xa + hw - QR, yy2 - 0.1, w=QR)

# rodape: o do modelo oficial permanece; acrescenta-se a instituicao do aluno
text(20.0, 43.3, 2.4, 0.9, "Institution", size=28, align=PP_ALIGN.CENTER, color=INK)
pic(f"{P}/logos/ufam_round.png", 20.55, 44.65, h=1.55)
prs.save(OUT)
