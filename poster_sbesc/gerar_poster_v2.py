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



# =================== V2: pouco texto, fonte grande (pedido do orientador) ===================
BODY = 43      # pt, texto corrido
SMALL = 30     # pt, legendas
HEAD = 58      # pt, titulos de secao


def heading2(x, y, num, title, w=None):
    w = CW if w is None else w
    rect(x, y, w, 1.5, fill=NAVY, name="Heading bar")
    rect(x, y, 0.28, 1.5, fill=TEAL)
    text(x + 0.65, y, w - 0.9, 1.5, [[(f"{num}  ", {"color": RGBColor(0x7F, 0xD6, 0xDC)}), (title, {})]],
         size=HEAD, color=WHITE, bold=True, anchor=MSO_ANCHOR.MIDDLE, name=f"Heading {title}")
    return y + 1.5 + 0.45


def bullets2(x, y, items, size=BODY, w=CW, sa=14):
    ps = [{"b": i} for i in items]
    h = est(ps, size, w, space_after=sa)
    text(x, y, w, h, ps, size=size, space_after=sa)
    return y + h


def tile2(x, y, w, h, big, small, color=NAVY, bigsize=96, smallsize=32):
    rect(x, y, w, h, fill=PANEL, line=None, shape=MSO_SHAPE.ROUNDED_RECTANGLE).adjustments[0] = 0.08
    rect(x, y + 0.3, 0.18, h - 0.6, fill=color)
    text(x + 0.5, y + 0.25, w - 0.7, h * 0.52, big, size=bigsize, bold=True, color=color, anchor=MSO_ANCHOR.MIDDLE)
    text(x + 0.5, y + h * 0.58, w - 0.7, h * 0.4, small, size=smallsize, color=MUTE, line_spacing=1.0)


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
text(tx, M + 2.95, tw, 0.8,
     "Marcela Alves Rodrigues · Lucas C. Cordeiro · José Reginaldo H. de Carvalho",
     size=38, bold=True, color=INK, align=PP_ALIGN.CENTER, name="Authors")
text(tx, M + 3.7, tw, 1.1,
     ["Graduate Program in Electrical Engineering (PPGEE) · Institute of Computing",
      "Federal University of Amazonas (UFAM), Manaus, Brazil"],
     size=26, color=MUTE, align=PP_ALIGN.CENTER, name="Affiliation")
rect(0, BH, W, 0.12, fill=TEAL)

TOP = BH + 0.8
tw3 = (CW - 0.8) / 3

# ---------------------------------------------------------------- coluna esquerda
y = heading2(XL, TOP, 1, "Why it matters")
y = bullets2(XL, y, [
    [("Autopilot firmware is safety-critical.", {"bold": True})],
    "PX4: about 710,000 lines of C/C++ on microcontrollers without memory protection.",
    "Tests can miss extreme sensor values and unsigned-arithmetic edge cases.",
])
y += 0.25
b = rect(XL, y, CW, 2.7, fill=HILITE, line=TEAL, lw=2.5, shape=MSO_SHAPE.ROUNDED_RECTANGLE); b.adjustments[0] = 0.08
text(XL + 0.5, y + 0.2, CW - 1.0, 2.3,
     [[("Gap: ", {"bold": True, "color": TEAL}),
       ("production PX4 driver code is still little examined by software verifiers.", {})]],
     size=BODY, anchor=MSO_ANCHOR.MIDDLE)
y += 2.7 + 0.7

y = heading2(XL, y, 2, "What we did")
y = bullets2(XL, y, [
    "Bounded model checking (ESBMC and CBMC) on harnesses of 12 PX4 modules.",
    "Hardware replaced by nondeterministic stubs; datasheet-derived assertions.",
])
y += 0.3
tile2(XL, y, tw3, 3.4, "12", "PX4 modules")
tile2(XL + tw3 + 0.4, y, tw3, 3.4, "2", "verifiers: ESBMC and CBMC", TEAL)
tile2(XL + 2 * (tw3 + 0.4), y, tw3, 3.4, "8", "verification stages")
y += 3.4 + 0.7

y = heading2(XL, y, 3, "Method: 8 stages")
stages = ["Select modules", "Hardware stubs", "Datasheet assertions", "Run ESBMC",
          "Check counterexamples", "Report to PX4", "Cross-check (CBMC)", "ArduPilot + CI"]
new = {1, 2, 6, 7}
bw = (CW - 0.4) / 2
RH2, RG2 = 1.55, 0.28
for i, t in enumerate(stages):
    col, row = i % 2, i // 2
    # ordem de leitura: 1,2 / 3,4 / ...
    xx = XL + col * (bw + 0.4)
    yy = y + row * (RH2 + RG2)
    n_ = i in new
    bb = rect(xx, yy, bw, RH2, fill=HILITE if n_ else PANEL, line=TEAL if n_ else LINE,
              shape=MSO_SHAPE.ROUNDED_RECTANGLE, lw=3 if n_ else 1.5, name=f"Stage {i+1}")
    bb.adjustments[0] = 0.18
    rect(xx + 0.25, yy + (RH2 - 1.0) / 2, 1.0, 1.0, fill=TEAL if n_ else NAVY, shape=MSO_SHAPE.OVAL)
    text(xx + 0.25, yy + (RH2 - 1.0) / 2, 1.0, 1.0, str(i + 1), size=40, bold=True, color=WHITE,
         align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(xx + 1.5, yy + 0.1, bw - 1.65, RH2 - 0.2, t, size=34, bold=True, color=NAVY,
         anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
y += 4 * RH2 + 3 * RG2 + 0.3
rect(XL, y + 0.1, 0.6, 0.45, fill=HILITE, line=TEAL, lw=2.5)
text(XL + 0.85, y, CW - 0.85, 0.9, "Highlighted stages are the additions of this work.", size=SMALL - 4, color=MUTE,
     anchor=MSO_ANCHOR.MIDDLE)
y += 0.95
QR = 2.9
y += 0.2
img = qrcode.make("https://doi.org/10.5281/zenodo.23282236", box_size=20, border=2)
img.save(f"{P}/build/qr.png")
pic(f"{P}/build/qr.png", XL + CW - QR, y, w=QR)
text(XL, y + 0.1, CW - QR - 0.4, QR - 0.2,
     [[("Artifact: ", {"bold": True, "color": NAVY}), ("harnesses and logs. Scan the code or visit ", {})],
      [("doi.org/10.5281/zenodo.23282236", {"bold": True, "color": TEAL})]],
     size=28, color=INK, line_spacing=1.08)
y += QR + 0.2
text(XL, y, CW, 0.9,
     [[("Supported by CAPES (Finance Code 001) and FAPEAM; part of the SAGRES Project.", {})]], size=22, color=MUTE)
y += 0.9
YL_END = y

# ---------------------------------------------------------------- coluna direita
y = heading2(XR, TOP, 4, "Results")
tile2(XR, y, tw3, 3.0, "10 of 12", "harnesses with no violation found", GREEN, bigsize=64)
tile2(XR + tw3 + 0.4, y, tw3, 3.0, "3", "counterexamples, none a confirmed bug", AMBER, bigsize=96)
tile2(XR + 2 * (tw3 + 0.4), y, tw3, 3.0, "≤ 65 s", "verification time per harness", NAVY, bigsize=72)
y += 3.0 + 0.45

text(XR, y, CW, 0.8, "Verification time per harness (s, median of 3 runs)", size=34, bold=True, color=NAVY)
y += 0.95
data = [("GPS uBlox", 64.9, 1), ("MAVLink Parser", 49.2, 0), ("EKF2 Covariance", 12.2, 0), ("RC Update", 9.5, 0),
        ("Battery Monitor", 6.7, 0), ("Magnetometer", 6.6, 0), ("Expo RC", 4.9, 0), ("VTOL", 4.0, 0),
        ("Barometer", 0.4, 0), ("PWM Output", 0.2, 0), ("IMU BMI088", 0.1, 1), ("Navigator/Mission", 0.1, 0)]
cd = CategoryChartData()
cd.categories = [d[0] for d in reversed(data)]
cd.add_series("Time (s)", [d[1] for d in reversed(data)])
CH = 7.0
gf = sl.shapes.add_chart(XL_CHART_TYPE.BAR_CLUSTERED, Inches(XR), Inches(y), Inches(CW), Inches(CH), cd)
gf.name = "Time chart"
ch = gf.chart
ch.has_legend = False; ch.has_title = False
ch.font.size = Pt(30); ch.font.name = FONT; ch.font.color.rgb = INK
pl = ch.plots[0]; pl.gap_width = 40
pl.has_data_labels = True
dl = pl.data_labels; dl.number_format = '0.0'; dl.number_format_is_linked = False
dl.position = XL_LABEL_POSITION.OUTSIDE_END; dl.font.size = Pt(30); dl.font.bold = True
ser = pl.series[0]
for i, d in enumerate(reversed(data)):
    pt = ser.points[i]; pt.format.fill.solid()
    pt.format.fill.fore_color.rgb = AMBER if d[2] else GREEN
va = ch.value_axis; va.maximum_scale = 80; va.minimum_scale = 0; va.major_unit = 20
va.has_major_gridlines = True; va.major_gridlines.format.line.color.rgb = LINE
va.format.line.fill.background(); va.tick_labels.font.size = Pt(26)
ca = ch.category_axis; ca.format.line.color.rgb = LINE; ca.tick_labels.font.size = Pt(30)
y += CH + 0.05
rect(XR + 0.2, y + 0.1, 0.5, 0.4, fill=GREEN)
text(XR + 0.9, y, 5.6, 0.6, "no violation", size=SMALL - 4, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
rect(XR + 6.4, y + 0.1, 0.5, 0.4, fill=AMBER)
text(XR + 7.1, y, 9.0, 0.6, "counterexample (stops at 1st violation)", size=SMALL - 4, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
y += 1.1

y = heading2(XR, y, 5, "What the 3 counterexamples were")
cases = [
    ("GPS uBlox", "Out-of-bounds copy in the model; unreachable in production.", "false positive (model)"),
    ("BMI088 FIFO", "The harness omitted 3 guards present in the driver.", "false positive"),
    ("BMI088 temperature", "86.25 °C above the 85 °C datasheet limit; PX4 declined it.", "disputed specification"),
]
for t, d, verdict in cases:
    hh = 2.75
    b = rect(XR, y, CW, hh, fill=PANEL, shape=MSO_SHAPE.ROUNDED_RECTANGLE); b.adjustments[0] = 0.05
    rect(XR, y + 0.25, 0.18, hh - 0.5, fill=AMBER)
    text(XR + 0.6, y + 0.15, CW - 1.0, 0.8, t, size=40, bold=True, color=NAVY)
    text(XR + 0.6, y + 0.92, CW - 1.0, 1.2, d, size=33)
    text(XR + 0.6, y + 2.0, CW - 1.0, 0.6,
         [[("Outcome: ", {"color": MUTE}), (verdict, {"bold": True, "color": AMBER})]], size=33)
    y += hh + 0.25
y += 0.25

y = heading2(XR, y, 6, "Take-aways")
y = bullets2(XR, y, [
    "BMC with datasheet-derived assertions is feasible on production PX4 code.",
    "10 of 12 harnesses are clean within the bound; counterexamples need triage against the real code.",
    [("Model-level evidence, not a proof. ", {"bold": True}), ("Next: k-induction, failsafe and geofence modules, HIL.", {})],
], size=40)
y += 0.2

YR_END = y

# ---------------------------------------------------------------- rodape
FY = H - M - 2.35
rect(M, FY, W - 2 * M, 0.06, fill=LINE)
groups = [("Organization", ["l01", "l02"]), ("Promotion", ["l11"]), ("Funding Agencies", ["l04", "l05"]),
          ("Support", ["l08", "l07"]), ("Sponsors", ["l06", "hwit"]), ("Author’s institution", ["ufam"])]
LH = 1.45
hmap = {"l04": 0.8, "l07": 0.75, "l06": 1.3}
widths = []
for g, ls in groups:
    ws = []
    for l in ls:
        im = Image.open(f"{P}/logos/{l}.png"); hh = hmap.get(l, LH); ws.append(hh * im.width / im.height)
    widths.append(ws)
inner = 0.45
gw = [max(sum(ws) + inner * (len(ws) - 1), 0.5) for ws in widths]
space = (W - 2 * M - sum(gw)) / (len(groups) - 1)
x = M
for (g, ls), ws, gwid in zip(groups, widths, gw):
    text(x - 1.0, FY + 0.18, gwid + 2.0, 0.4, g, size=19, bold=True, color=MUTE, align=PP_ALIGN.CENTER)
    xx = x
    for l, w_ in zip(ls, ws):
        hh = hmap.get(l, LH)
        pic(f"{P}/logos/{l}.png", xx, FY + 0.7 + (LH - hh) / 2, h=hh)
        xx += w_ + inner
    x += gwid + space

print("fim esquerda %.2f | fim direita %.2f | rodape %.2f | pagina %.2f" % (YL_END, YR_END, FY, H))
prs.save(OUT)
