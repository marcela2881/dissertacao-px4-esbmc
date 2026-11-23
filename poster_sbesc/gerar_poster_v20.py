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
GREEN = RGBColor(0x0B, 0x3A, 0x63)
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
    elif _sh.name == "Google Shape;54;p13":
        for _r in _sh.text_frame.paragraphs[1].runs:
            _r.font.size = Pt(48)      # data do evento menor (era 72 pt)
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
ORANGE = RGBColor(0x6B, 0x77, 0x85)
GREEN = RGBColor(0x0B, 0x3A, 0x63)
DGRAY = RGBColor(0x1F, 0x29, 0x33)
MGRAY = RGBColor(0x55, 0x60, 0x6E)
LGRAY = RGBColor(0xF2, 0xF4, 0xF7)
TEAL = BLUE; PALE = LBLUE; PANEL = LGRAY; INK = DGRAY; MUTE = MGRAY
bg.fill.fore_color.rgb = WHITE

HB = 4.9
# moldura do modelo oficial: linha cinza discreta
for _sh in sl.shapes:
    if _sh.name == "Google Shape;58;p13":
        _sh.line.color.rgb = RGBColor(0xB8, 0xC2, 0xCC); _sh.line.width = Pt(1.5)
pic(f"{P}/build/faixa_titulo.png", FX + 0.05, 5.0, w=FW - 0.1)
text(FX + 0.7, 5.1, FW - 5.2, 2.5,
     "Formal Verification of Critical Modules in the PX4 Autopilot using Bounded Model Checking",
     size=58, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0, name="Title")
text(FX + 0.9, 7.55, FW - 1.8, 0.7, "Marcela Alves Rodrigues\u00b9 \u00b7 Lucas C. Cordeiro\u00b2 \u00b7 Jos\u00e9 Reginaldo H. de Carvalho\u00b3",
     size=38, bold=True, color=WHITE, align=PP_ALIGN.CENTER, name="Authors")
text(FX + 0.9, 8.35, FW - 1.8, 0.9,
     "\u00b9Graduate Program in Electrical Engineering (PPGEE) \u00b7 \u00b2Dept. of Electrical and Computer Engineering",
     size=30, color=WHITE, align=PP_ALIGN.CENTER, name="Affiliation")
text(FX + 0.9, 8.85, FW - 1.8, 0.7,
     "\u00b3Institute of Computing \u00b7 Federal University of Amazonas (UFAM), Manaus, Brazil",
     size=30, color=WHITE, align=PP_ALIGN.CENTER, name="Affiliation2")
TOP = 10.3
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


# ============================================================ v14: legivel, sobrio, visual
SOFT = RGBColor(0xC9, 0xDC, 0xEE)
CARDS = []


def section(x, y, w, num, title, fs=52):
    card = rect(x, y + 0.65, w, 1.0, fill=WHITE, line=SOFT, lw=2.5, shape=SH.ROUNDED_RECTANGLE, name="Card")
    card.adjustments[0] = 0.02
    rb = rect(x + 0.9, y, w - 1.4, 1.3, fill=NAVY, shape=SH.ROUNDED_RECTANGLE, name="Ribbon"); rb.adjustments[0] = 0.5
    if num is None:
        text(x + 1.3, y, w - 1.9, 1.3, title, size=fs, bold=True, color=WHITE, anchor=MSO_ANCHOR.MIDDLE)
    else:
        rect(x + 0.1, y - 0.1, 1.5, 1.5, fill=BLUE, line=WHITE, lw=4, shape=SH.OVAL)
        text(x + 0.1, y - 0.1, 1.5, 1.5, str(num), size=52, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
        text(x + 2.0, y, w - 2.6, 1.3, title, size=fs, bold=True, color=WHITE, anchor=MSO_ANCHOR.MIDDLE)
    CARDS.append((card, y + 0.65))
    return y + 1.3 + 0.5


def endcard(y):
    card, y0 = CARDS[-1]
    card.height = Inches(y + 0.3 - y0)
    return y + 0.3


def icon_check(x, y, d, color=NAVY):
    rect(x, y, d, d, fill=color, shape=SH.OVAL)
    a = rect(x + d * 0.22, y + d * 0.50, d * 0.30, d * 0.10, fill=WHITE); a.rotation = 45
    b = rect(x + d * 0.30, y + d * 0.43, d * 0.58, d * 0.10, fill=WHITE); b.rotation = -50


def icon_warn(x, y, d, color=MGRAY):
    rect(x, y, d, d * 0.9, fill=color, shape=SH.ISOSCELES_TRIANGLE)
    text(x, y + d * 0.25, d, d * 0.7, "!", size=int(d * 45), bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)


def icon_clock(x, y, d, color=NAVY):
    rect(x, y, d, d, fill=None, line=color, lw=5, shape=SH.OVAL)
    rect(x + d / 2 - 0.03, y + d * 0.2, 0.06, d * 0.33, fill=color)
    rect(x + d / 2 - 0.03, y + d / 2 - 0.03, d * 0.28, 0.06, fill=color)


def tile3(x, y, w, h, big, small, color, icon, bigsize):
    rect(x, y, w, h, fill=LGRAY, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.08
    d = 1.0
    icon(x + 0.35, y + 0.35, d, color)
    text(x + 0.35 + d + 0.2, y + 0.2, w - d - 0.8, h * 0.52, big, size=bigsize, bold=True, color=color, anchor=MSO_ANCHOR.MIDDLE)
    text(x + 0.4, y + h * 0.60, w - 0.7, h * 0.38, small, size=28, color=INK, line_spacing=1.0)


GAPS = 0.35
# ---------------------------------------------------------------- esquerda
y = TOP
y = section(XL, y, PW, 1, "Introduction")
y = mark_bullets(X0L, y, IW, [
    [("Safety-critical: ", {"bold": True}), ("PX4 flies real drones [5].", {})],
    "One faulty driver can corrupt flight state.",
    [("Gap: ", {"bold": True, "color": BLUE}), ("driver code is rarely verified.", {})],
], size=36, sa=10)
y += 0.15
bw_ = 2.5; gp_ = (IW - 5 * bw_) / 4; bh_ = 1.5
cont = rect(X0L + (bw_ + gp_) - 0.2, y + 0.55, 3 * bw_ + 2 * gp_ + 0.4, bh_ + 0.5, line=BLUE, lw=3, shape=SH.ROUNDED_RECTANGLE)
cont.adjustments[0] = 0.08; cont.line.dash_style = MSO_LINE.DASH
text(X0L + (bw_ + gp_) - 0.2, y, 3 * bw_ + 2 * gp_ + 0.4, 0.55, "one shared address space", size=28, bold=True,
     color=BLUE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
by_ = y + 0.8
for k_, lab_ in enumerate(["Sensors", "PX4\ndrivers", "uORB\nbus", "Flight\ncontrol", "Vehicle\n(motors)"]):
    bx_ = X0L + k_ * (bw_ + gp_); edge = k_ in (0, 4)
    box(bx_, by_, bw_, bh_, lab_, size=28, fill=LGRAY if edge else LBLUE, line=LINE if edge else BLUE)
    if k_ < 4: rarrow(bx_ + bw_ + 0.05, by_ + bh_ / 2, w=gp_ - 0.1, h=0.5, color=NAVY)
ly_ = by_ + bh_ + 0.42
rect(X0L + 4 * (bw_ + gp_) + bw_ / 2 - 0.025, by_ + bh_, 0.05, 0.44, fill=NAVY)
rect(X0L + bw_ / 2 - 0.025, ly_ - 0.025, 4 * (bw_ + gp_) + 0.05, 0.05, fill=NAVY)
rect(X0L + bw_ / 2 - 0.025, by_ + bh_ + 0.2, 0.05, 0.24, fill=NAVY)
rect(X0L + bw_ / 2 - 0.16, by_ + bh_ + 0.03, 0.32, 0.24, fill=NAVY, shape=SH.ISOSCELES_TRIANGLE)
text(X0L + 2 * (bw_ + gp_) - 0.5, ly_ + 0.1, bw_ + 1.0, 0.5, "feedback loop", size=24, color=MUTE, align=PP_ALIGN.CENTER)
y = endcard(ly_ + 0.6) + GAPS

y = section(XL, y, PW, 2, "Proposal")
kb = rect(X0L, y, IW, 1.6, fill=LBLUE, shape=SH.ROUNDED_RECTANGLE); kb.adjustments[0] = 0.1
text(X0L + 0.5, y + 0.05, IW - 1.0, 1.5,
     [[("Bounded model checking ", {"bold": True, "color": NAVY}), ("[1, 2] of harnesses built from excerpts of 12 PX4 modules", {})]],
     size=38, anchor=MSO_ANCHOR.MIDDLE)
y += 1.6 + 0.2
text(X0L, y, IW, 1.0, [[("Scope: ", {"bold": True, "color": NAVY}), ("the harnesses are copied excerpts, not compiled against PX4; some constants differ (GPS buffer: 200 bytes in the harness, 79 in PX4).", {})]],
     size=24, color=INK, line_spacing=1.0)
y += 0.95
stages = ["Select modules", "Hardware stubs", "Datasheet properties", "ESBMC run [2]",
          "Analyze counterexample", "Report to PX4", "Cross-check (CBMC) [4]", "ArduPilot + CI"]
cw4 = (IW - 3 * 0.3) / 4
CHH = 1.25
for k_, st_ in enumerate(stages):
    r_, c_ = divmod(k_, 4)
    cx_ = X0L + c_ * (cw4 + 0.3); cy2 = y + r_ * (CHH + 0.3)
    added = k_ in (1, 2, 6, 7)
    rr = rect(cx_, cy2, cw4, CHH, fill=NAVY if added else LBLUE, line=NAVY, lw=2.5, shape=SH.ROUNDED_RECTANGLE); rr.adjustments[0] = 0.15
    text(cx_ + 0.1, cy2 + 0.05, cw4 - 0.2, 0.6, str(k_ + 1), size=30, bold=True, color=WHITE if added else BLUE, align=PP_ALIGN.CENTER)
    text(cx_ + 0.12, cy2 + 0.55, cw4 - 0.24, CHH - 0.6, st_, size=24, bold=True, color=WHITE if added else NAVY,
         align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, line_spacing=0.95)
y += 2 * CHH + 0.3 + 0.1
text(X0L, y, IW, 0.8, "Dark: our additions (stages 2–3 and 7–8); the others follow Sousa et al. [3]", size=24, color=MUTE)
y += 0.5
text(X0L, y, IW, 0.8, "Stage 7: the patch check was inconclusive (0 of 7 failed with and without the patch)", size=24, color=INK)
y += 0.65
for rq_, t_ in (("RQ1", "Every timed harness: under 90 s"), ("RQ2", "Yes, in the GPS harness (model-level)"),
                ("RQ3", "3 settings: MAVLink k = 300, Navigator k = 15, VTOL no library")):
    rect(X0L, y + 0.05, 1.4, 0.8, fill=BLUE, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.3
    text(X0L, y + 0.05, 1.4, 0.8, rq_, size=28, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(X0L + 1.65, y, IW - 1.65, 0.9, t_, size=26, color=INK, anchor=MSO_ANCHOR.MIDDLE, line_spacing=0.95)
    y += 0.8
y = endcard(y) + GAPS

# referencias e agradecimento (sem numero), embaixo da coluna esquerda
y = section(XL, y, PW, None, "References", 40)
text(X0L, y - 0.15, IW, 3.2,
     ["[1] Biere et al., “Symbolic model checking without BDDs,” TACAS, 1999.",
      "[2] Cordeiro, Fischer, Marques-Silva, “SMT-based bounded model checking for embedded ANSI-C software,” IEEE TSE, 2012.",
      "[3] Sousa et al., “Finding software vulnerabilities in large C projects via bounded model checking,” STTT, 2026.",
      "[4] Kroening, Tautschnig, \u201cCBMC \u2013 C bounded model checker (competition contribution),\u201d TACAS, 2014.",
      "[5] PX4 Development Team, \u201cPX4-Autopilot: Open source autopilot for drones,\u201d 2024."],
     size=24, color=INK, space_after=4, line_spacing=1.0)
y = endcard(y + 3.7) + GAPS
y = section(XL, y, PW, None, "Acknowledgment", 40)
QR = 1.8
text(X0L, y - 0.1, IW - QR - 0.5, QR,
     [[("CAPES (Finance Code 001), FAPEAM, SAGRES Project", {})], [("Artifact (harnesses and logs): ", {"bold": True}), ("scan the code", {"color": BLUE})]],
     size=24, color=INK, space_after=6, line_spacing=1.0)
qimg = qrcode.make("https://doi.org/10.5281/zenodo.23282236", box_size=20, border=1)
qimg.save(f"{P}/build/qr.png")
pic(f"{P}/build/qr.png", X0L + IW - QR, y - 0.2, w=QR)
y = endcard(y - 0.2 + QR)
print("fim esquerda %.2f limite %.2f" % (y, END))

# ---------------------------------------------------------------- direita
y = TOP
y = section(XR, y, PW, 3, "Results")
tw3 = (IW - 0.8) / 3
TH = 2.6
tile3(X0R, y, tw3, TH, "10 of 12", "harnesses with no violation", NAVY, icon_check, 44)
tile3(X0R + tw3 + 0.4, y, tw3, TH, "3", "findings across 2 harnesses", MGRAY, icon_warn, 80)
tile3(X0R + 2 * (tw3 + 0.4), y, tw3, TH, "≤ 65 s", "per harness", NAVY, icon_clock, 52)
y += TH + 0.4
y = sub(X0R, y, IW, "Verification time (s, median of 3 runs)", 32)
data = [("GPS uBlox", 64.9, 1), ("MAVLink Parser", 49.2, 0), ("EKF2 Covariance", 12.2, 0), ("RC Update", 9.5, 0),
        ("7 other harnesses", 6.7, 0), ("IMU BMI088", 0.1, 1)]
LW_ = 5.8; BX = X0R + LW_ + 0.2; BWMAX = IW - LW_ - 0.2 - 1.9; RH = 0.72
for i, (nm, v, ce) in enumerate(data):
    yy = y + i * RH
    if i % 2 == 0: rect(X0R, yy, IW, RH, fill=LGRAY)
    text(X0R + 0.1, yy, LW_ - 0.1, RH, nm, size=32, color=INK, align=PP_ALIGN.RIGHT, anchor=MSO_ANCHOR.MIDDLE)
    bw = max(BWMAX * v / 65.0, 0.3)
    rect(BX, yy + 0.13, bw, RH - 0.26, fill=MGRAY if ce else NAVY)
    text(BX + bw + 0.15, yy, 1.9, RH, ("≤ " if i == 4 else "") + f"{v:.1f}", size=30, bold=True, color=INK, anchor=MSO_ANCHOR.MIDDLE)
y += len(data) * RH + 0.12
rect(BX, y + 0.1, 0.45, 0.36, fill=NAVY); text(BX + 0.6, y, 3.8, 0.58, "no violation", size=26, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
rect(BX + 4.2, y + 0.1, 0.45, 0.36, fill=MGRAY); text(BX + 4.85, y, 5.5, 0.58, "violation (time to first)", size=26, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
y += 0.7
text(X0R, y, IW, 0.8, "VTOL was checked with --no-library: ESBMC’s C library checks are not verified.", size=24, color=MUTE, line_spacing=1.0)
y = endcard(y + 0.8) + GAPS

y = section(XR, y, PW, None, "Counterexamples", 46)
lines = ["[Counterexample]",
         "State 3 -- test_gps_real_bit_operation, line 179",
         "  input_len = 10",
         "State 8 -- test_gps_real_bit_operation, line 185",
         "  dump_buffer.len = 207",
         "State 21 -- __memcpy_impl, line 280",
         "  Violated property: dereference failure:",
         "  Access to object out of bounds",
         "",
         "VERIFICATION FAILED"]
LH_ = 0.40
bxh = LH_ * len(lines) + 0.4
b_ = rect(X0R, y, IW, bxh, fill=RGBColor(0xF7, 0xF8, 0xFA), line=RGBColor(0x9A, 0xA4, 0xAF), lw=1.5)
text(X0R + 0.3, y + 0.2, IW - 0.5, bxh - 0.3, [[(l, {"font": MONO, "bold": l.startswith("VERIF")})] for l in lines], size=22, space_after=0, line_spacing=1.1)
y += bxh + 0.1
text(X0R, y, IW, 0.5, "ESBMC output for GPS uBlox, as in Listing 2 of the paper", size=24, color=MUTE)
y += 0.6
RHT = 1.0
for mod, res in [("GPS uBlox", "Model-only"), ("BMI088 FIFO", "False positive"), ("BMI088 temperature", "Contested")]:
    rr = rect(X0R, y, IW, RHT, fill=LGRAY, shape=SH.ROUNDED_RECTANGLE); rr.adjustments[0] = 0.12
    text(X0R + 0.4, y, 7.5, RHT, mod, size=32, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE)
    pill = rect(X0R + IW - 5.0, y + 0.18, 4.7, RHT - 0.36, fill=MGRAY, shape=SH.ROUNDED_RECTANGLE); pill.adjustments[0] = 0.3
    text(X0R + IW - 5.0, y + 0.18, 4.7, RHT - 0.36, res, size=30, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    y += RHT + 0.12
y += -0.12 + 0.1
text(X0R, y, IW, 0.7, "None of the three is a confirmed bug.", size=30, bold=True, color=NAVY)
y = endcard(y + 0.65) + GAPS

y = section(XR, y, PW, 4, "Conclusions / Future work", 46)
CHT = 1.2
for t_ in ["Eight-stage methodology", "Feasible for selected PX4 module harnesses", "Model-level evidence, not a proof"]:
    icon_check(X0R, y + (CHT - 0.95) / 2, 0.95, BLUE)
    text(X0R + 1.4, y, IW - 1.4, CHT, t_, size=36, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE)
    y += CHT + 0.05
text(X0R, y + 0.05, IW, 0.9, [[("Next: ", {"bold": True, "color": BLUE}), ("k-induction and hardware-in-the-loop.", {})]], size=34)
y = endcard(y + 0.9)
print("fim direita %.2f limite %.2f" % (y, END))

text(20.0, 43.3, 2.4, 0.9, "Institution", size=28, align=PP_ALIGN.CENTER, color=INK)
pic(f"{P}/logos/ufam_round.png", 20.55, 44.65, h=1.55)
prs.save(OUT)
