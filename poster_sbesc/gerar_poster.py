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


def bullets(x, y, items, size=28, w=CW, sa=10):
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


# ---------------------------------------------------------------- header
pic(f"{P}/logos/ufam.png", M, M, h=4.0)
pic(f"{P}/logos/l10.png", W - M - 5.0, M + 0.35, w=5.0)
text(W - M - 5.0, M + 2.75, 5.0, 1.2,
     ["XVI Symposium on Computing", "Systems Engineering", "Nov 24–27, 2026 · Salvador, BA"],
     size=17, color=MUTE, align=PP_ALIGN.CENTER, line_spacing=1.05)
tx, tw = M + 4.5, W - 2 * M - 4.5 - 5.4
text(tx, M - 0.05, tw, 3.1,
     "Formal Verification of Critical Modules in the PX4 Autopilot using Bounded Model Checking",
     size=66, bold=True, color=NAVY, align=PP_ALIGN.CENTER, line_spacing=1.0, name="Title")
text(tx, M + 3.2, tw, 0.75,
     "Marcela Alves Rodrigues · Lucas C. Cordeiro · José Reginaldo H. de Carvalho",
     size=34, bold=True, color=INK, align=PP_ALIGN.CENTER, name="Authors")
text(tx, M + 4.0, tw, 1.1,
     ["Graduate Program in Electrical Engineering (PPGEE) · Institute of Computing",
      "Federal University of Amazonas (UFAM), Manaus, Brazil · marcela.rodrigues@ufam.edu.br"],
     size=24, color=MUTE, align=PP_ALIGN.CENTER, name="Affiliation")
rect(M, M + 5.35, W - 2 * M, 0.09, fill=TEAL)

TOP = M + 5.85
# ---------------------------------------------------------------- left column
y = heading(XL, TOP, 1, "Introduction")
y = bullets(XL, y, [
    [("Autopilot firmware is safety-critical. ", {"bold": True}),
     ("PX4’s source tree has ≈710k lines of C/C++ and runs on microcontrollers without a memory management unit (MMU).", {})],
    "All modules share one address space, so a memory defect in any driver can corrupt flight-critical state.",
    "Testing can miss faults triggered by extreme sensor values or unsigned-arithmetic edge cases.",
    [("Gap: ", {"bold": True}), ("production autopilot driver code has not been examined with software verifiers.", {})],
])
y += 0.45

y = heading(XL, y, 2, "Proposal")
y = bullets(XL, y, [
    [("Apply ", {}), ("bounded model checking (BMC)", {"bold": True}),
     (" to harnesses derived from twelve PX4 modules, using ESBMC and CBMC.", {})],
    "Replace hardware interfaces (UART, I2C, SPI, uORB) with nondeterministic stubs and add assertions derived from datasheets.",
])
y += 0.15
tw3 = (CW - 0.8) / 3
tile(XL, y, tw3, 2.5, "12", "PX4 modules covered by a harness")
tile(XL + tw3 + 0.4, y, tw3, 2.5, "≈2,800", "lines of hand-written harness code", TEAL)
tile(XL + 2 * (tw3 + 0.4), y, tw3, 2.5, "≈10,400", "lines of target PX4 code")
y += 2.5 + 0.35
rq = [
    [("RQ1 ", {"bold": True, "color": TEAL}), ("What is the per-module verification latency on commodity hardware?", {})],
    [("RQ2 ", {"bold": True, "color": TEAL}), ("Does BMC expose model-reachable violations that existing PX4 unit tests do not exercise?", {})],
    [("RQ3 ", {"bold": True, "color": TEAL}), ("Which per-module configurations are needed to make each module tractable?", {})],
]
h = est(rq, 26, CW, space_after=6)
text(XL, y, CW, h, rq, size=26, space_after=6)
y += h + 0.45

y = heading(XL, y, 3, "Development")
stages = [
    ("Module selection", "by safety impact on the sensing-to-actuation path and stub feasibility", False),
    ("Hardware-abstraction stubs", "UART, I2C, SPI and uORB calls return nondeterministic values", True),
    ("Safety-property specification", "automatic checks plus datasheet-derived assertions", True),
    ("ESBMC execution", "unwind bound k = 10, overflow check, 120 s timeout", False),
    ("Counterexample analysis", "each trace is checked against the unmodified driver source", False),
    ("Community reporting", "findings are filed with the PX4 developers", False),
    ("Cross-tool validation and patch verification", "CBMC 5.95.1 as an independent verifier", True),
    ("Cross-autopilot check and CI/CD", "ArduPilot comparison; harnesses run in GitHub Actions", True),
]
RH, RG = 1.26, 0.3
for i, (t, d, new) in enumerate(stages):
    b = rect(XL, y, CW, RH, fill=HILITE if new else PANEL, line=TEAL if new else LINE,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, lw=2.25 if new else 1.25, name=f"Stage {i+1}")
    b.adjustments[0] = 0.18
    c = rect(XL + 0.22, y + 0.23, 0.8, 0.8, fill=TEAL if new else NAVY, shape=MSO_SHAPE.OVAL)
    text(XL + 0.22, y + 0.23, 0.8, 0.8, str(i + 1), size=30, bold=True, color=WHITE,
         align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(XL + 1.25, y + 0.1, CW - 1.45, RH - 0.2,
         [[(t, {"bold": True, "size": 27, "color": NAVY})], [(d, {"size": 22, "color": MUTE})]],
         size=24, anchor=MSO_ANCHOR.MIDDLE, line_spacing=1.0)
    y += RH
    if i < len(stages) - 1:
        rect(XL + 0.47, y + 0.03, 0.3, RG - 0.06, fill=NAVY, shape=MSO_SHAPE.DOWN_ARROW)
        y += RG
y += 0.2
rect(XL, y + 0.07, 0.5, 0.36, fill=HILITE, line=TEAL, lw=2)
text(XL + 0.7, y, CW - 0.7, 0.9,
     "Eight-stage verification pipeline. Highlighted stages (2, 3, 7, 8) are the additions of this work; "
     "stages 1 and 4–6 follow the workflow of Sousa et al. [1].", size=21, color=MUTE)
y += 1.05

text(XL, y, CW, 0.6, "Modules covered, by input class", size=28, bold=True, color=NAVY)
y += 0.65
classes = [
    ("Serial sensor", "GPS uBlox · IMU BMI088 · Magnetometer · Barometer"),
    ("Radio input", "Expo RC · RC Update"),
    ("Actuation", "PWM Output · VTOL"),
    ("Telemetry link", "MAVLink"),
    ("Power sensor", "Battery Monitor"),
    ("State estimation", "EKF2 Covariance"),
    ("Mission logic", "Navigator"),
]
rh = 0.66
for i, (c, m) in enumerate(classes):
    if i % 2 == 0: rect(XL, y, CW, rh, fill=PANEL)
    text(XL + 0.2, y, 4.2, rh, c, size=24, bold=True, color=TEAL, anchor=MSO_ANCHOR.MIDDLE)
    text(XL + 4.5, y, CW - 4.7, rh, m, size=24, anchor=MSO_ANCHOR.MIDDLE)
    y += rh
y += 0.3
setup = [[("Setup: ", {"bold": True, "color": NAVY}),
          ("ESBMC v8.1.0 with Boolector 3.2.4; CBMC 5.95.1; PX4 main branch, commit 8be2590c15 (2026-06-04); "
           "Intel i7-13700K, 64 GB, Ubuntu 24.04. Times are medians of three runs.", {})]]
h = est(setup, 24, CW)
text(XL, y, CW, h, setup, size=24, color=MUTE)
YL_END = y + h

# ---------------------------------------------------------------- right column
y = heading(XR, TOP, 4, "Results")
tile(XR, y, tw3, 2.5, "10 of 12", "harnesses verify with no violation up to the bound", GREEN)
tile(XR + tw3 + 0.4, y, tw3, 2.5, "2", "harnesses with a counterexample (GPS, BMI088)", AMBER)
tile(XR + 2 * (tw3 + 0.4), y, tw3, 2.5, "0.1–64.9 s", "verification time per harness")
y += 2.5 + 0.4

text(XR, y, CW, 0.6, "Verification time per harness (seconds, median of three runs)", size=26, bold=True, color=NAVY)
y += 0.7
data = [("GPS uBlox", 64.9, 1), ("MAVLink Parser", 49.2, 0), ("EKF2 Covariance", 12.2, 0), ("RC Update", 9.5, 0),
        ("Battery Monitor", 6.7, 0), ("Magnetometer", 6.6, 0), ("Expo RC", 4.9, 0), ("VTOL", 4.0, 0),
        ("Barometer", 0.4, 0), ("PWM Output", 0.2, 0), ("IMU BMI088", 0.1, 1), ("Navigator/Mission", 0.1, 0)]
cd = CategoryChartData()
cd.categories = [d[0] for d in reversed(data)]
cd.add_series("Time (s)", [d[1] for d in reversed(data)])
CH = 7.3
gf = sl.shapes.add_chart(XL_CHART_TYPE.BAR_CLUSTERED, Inches(XR), Inches(y), Inches(CW), Inches(CH), cd)
gf.name = "Time chart"
ch = gf.chart
ch.has_legend = False; ch.has_title = False
ch.font.size = Pt(22); ch.font.name = FONT; ch.font.color.rgb = INK
pl = ch.plots[0]; pl.gap_width = 45
pl.has_data_labels = True
dl = pl.data_labels; dl.number_format = '0.0'; dl.number_format_is_linked = False
dl.position = XL_LABEL_POSITION.OUTSIDE_END; dl.font.size = Pt(22); dl.font.bold = True
ser = pl.series[0]
for i, d in enumerate(reversed(data)):
    pt = ser.points[i]; pt.format.fill.solid()
    pt.format.fill.fore_color.rgb = AMBER if d[2] else GREEN
va = ch.value_axis; va.maximum_scale = 75; va.minimum_scale = 0; va.major_unit = 25
va.has_major_gridlines = True; va.major_gridlines.format.line.color.rgb = LINE
va.format.line.fill.background(); va.tick_labels.font.size = Pt(20)
ca = ch.category_axis; ca.format.line.color.rgb = LINE; ca.tick_labels.font.size = Pt(22)
y += CH + 0.05
rect(XR + 0.2, y + 0.08, 0.4, 0.34, fill=GREEN)
text(XR + 0.75, y, 6.2, 0.5, "no violation up to the bound", size=21, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
rect(XR + 7.2, y + 0.08, 0.4, 0.34, fill=AMBER)
text(XR + 7.75, y, 8.4, 0.5, "counterexample found (stops at first violation)", size=21, color=MUTE, anchor=MSO_ANCHOR.MIDDLE)
y += 0.85

text(XR, y, CW, 0.6, "Triage of the three counterexamples against production code", size=26, bold=True, color=NAVY)
y += 0.72
cases = [
    ("GPS uBlox — out-of-bounds memcpy",
     "The harness reaches input_len = 10 with dump_buffer.len = 207. CBMC reproduces it at a reduced bound. "
     "The maintainer’s review and source analysis show the production write loop clamps the length, so the state "
     "does not arise there. A defensive guard was proposed upstream.",
     "Reachable in the model, not in production"),
    ("IMU BMI088 — FIFO byte counter",
     "ESBMC produced count = 9088 for a 1,024-byte FIFO. The released driver guards this read on three levels; "
     "the harness had omitted those guards.",
     "False positive"),
    ("IMU BMI088 — temperature range",
     "A reading of 86.25 °C exceeds the datasheet operating range of −40 to +85 °C without validation. "
     "The proposed fix was declined by the PX4 developers.",
     "Design note, declined upstream"),
]
for t, d, verdict in cases:
    dp = [d]
    hh = 0.62 + est(dp, 23, CW - 0.9) + 0.72
    b = rect(XR, y, CW, hh, fill=PANEL, shape=MSO_SHAPE.ROUNDED_RECTANGLE); b.adjustments[0] = 0.05
    rect(XR, y + 0.2, 0.14, hh - 0.4, fill=AMBER)
    text(XR + 0.45, y + 0.14, CW - 0.9, 0.5, t, size=26, bold=True, color=NAVY)
    text(XR + 0.45, y + 0.68, CW - 0.9, hh - 1.3, dp, size=23)
    text(XR + 0.45, y + hh - 0.62, CW - 0.9, 0.5,
         [[("Outcome: ", {"color": MUTE}), (verdict, {"bold": True, "color": AMBER})]], size=23)
    y += hh + 0.22
y += 0.1
ans = [
    [("RQ1 ", {"bold": True, "color": TEAL}), ("Every timed harness verifies in under ninety seconds: per-module feasibility, not whole-system scaling.", {})],
    [("RQ2 ", {"bold": True, "color": TEAL}), ("Yes for one module: no PX4 unit test references the GPS function where the violation was found.", {})],
    [("RQ3 ", {"bold": True, "color": TEAL}), ("Three adjustments suffice: MAVLink --unwind 300, Navigator --unwind 15, VTOL --no-library.", {})],
]
h = est(ans, 24, CW, space_after=6)
text(XR, y, CW, h, ans, size=24, space_after=6)
y += h + 0.45

y = heading(XR, y, 5, "Conclusions and Future Work")
y = bullets(XR, y, [
    "BMC with datasheet-derived assertions is feasible for production autopilot code and reaches paths that unit tests do not cover.",
    "Ten harnesses give bounded, model-level evidence of memory safety and absence of overflow.",
    "A formally valid counterexample must be checked against production code before it is called a defect.",
    [("Future work: ", {"bold": True}), ("k-induction; commander, failsafe and geofence modules; PX4 CI with hardware-in-the-loop evaluation.", {})],
], size=26, sa=8)
y += 0.45

y = heading(XR, y, 6, "References and Acknowledgment")
refs = [
    "[1] J. O. de Sousa et al., “Finding software vulnerabilities in large C projects via bounded model checking,” Int. J. Softw. Tools Technol. Transf., 2026.",
    "[2] L. Cordeiro, B. Fischer and J. Marques-Silva, “SMT-based bounded model checking for embedded ANSI-C software,” IEEE Trans. Softw. Eng., 2012.",
    "[3] D. Kroening and M. Tautschnig, “CBMC – C bounded model checker,” TACAS 2014.",
    "[4] S. Khatiri, S. Panichella and P. Tonella, “Simulation-based testing of unmanned aerial vehicles with Aerialist,” ICSE-Companion 2024.",
]
QR = 3.0
rw = CW - QR - 0.5
h = est(refs, 19, rw, space_after=4)
text(XR, y, rw, h, refs, size=19, color=INK, space_after=4)
img = qrcode.make("https://doi.org/10.5281/zenodo.23282236", box_size=20, border=2)
img.save(f"{P}/build/qr.png")
pic(f"{P}/build/qr.png", XR + CW - QR, y, w=QR)
text(XR + CW - QR - 0.3, y + QR + 0.05, QR + 0.6, 0.8, ["Artifact: harnesses and logs", "doi.org/10.5281/zenodo.23282236"],
     size=14, color=MUTE, align=PP_ALIGN.CENTER)
y += max(h, QR + 0.9) + 0.1
ack = [[("Acknowledgment. ", {"bold": True, "color": NAVY}),
        ("Part of the SAGRES Project, funded by CAPES Pr\u00f3-Defesa V, carried out at UFAM with the support of CAPES "
         "(AUXPE-CAPES-PROEX, Finance Code 001) and FAPEAM (PDPG-CAPES and POSGRAD projects). "
         "The authors thank the PX4 development community.", {})]]
h = est(ack, 20, CW)
text(XR, y, CW, h, ack, size=20, color=MUTE)
YR_END = y + h

# ---------------------------------------------------------------- footer
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
gw = [sum(ws) + inner * (len(ws) - 1) for ws in widths]
gw = [max(g, 0.5) for g in gw]
space = (W - 2 * M - sum(gw)) / (len(groups) - 1)
x = M
for (g, ls), ws, gwid in zip(groups, widths, gw):
    text(x - 1.0, FY + 0.18, gwid + 2.0, 0.4, g, size=17, bold=True, color=MUTE, align=PP_ALIGN.CENTER)
    xx = x
    for l, w_ in zip(ls, ws):
        hh = hmap.get(l, LH)
        pic(f"{P}/logos/{l}.png", xx, FY + 0.7 + (LH - hh) / 2, h=hh)
        xx += w_ + inner
    x += gwid + space

print("left end %.2f right end %.2f footer %.2f page %.2f" % (YL_END, YR_END, FY, H))
prs.save(OUT)
