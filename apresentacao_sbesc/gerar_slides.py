import sys
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE as SH
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.dml import MSO_LINE
from PIL import Image

PS = "../poster_sbesc"          # imagens ja geradas para o poster
OUT = sys.argv[1]
NAVY = RGBColor(0x0B, 0x3A, 0x63); BLUE = RGBColor(0x1F, 0x6F, 0xB2); TURQ = RGBColor(0x00, 0xB3, 0xBE)
INK = RGBColor(0x1F, 0x29, 0x33); GRAY = RGBColor(0x5B, 0x66, 0x73); LGRAY = RGBColor(0xF1, 0xF4, 0xF7)
LBLUE = RGBColor(0xE6, 0xF0, 0xF8); WHITE = RGBColor(0xFF, 0xFF, 0xFF); LINE = RGBColor(0xC8, 0xD1, 0xDA)
FONT = "Arial"
W, H = 13.333, 7.5

prs = Presentation()
prs.slide_width, prs.slide_height = Inches(W), Inches(H)


def rect(sl, x, y, w, h, fill=None, line=None, shape=SH.RECTANGLE, lw=2):
    s = sl.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    if fill is None: s.fill.background()
    else: s.fill.solid(); s.fill.fore_color.rgb = fill
    if line is None: s.line.fill.background()
    else: s.line.color.rgb = line; s.line.width = Pt(lw)
    s.shadow.inherit = False
    st = s._element.find('{http://schemas.openxmlformats.org/presentationml/2006/main}style')
    if st is not None: s._element.remove(st)
    return s


def text(sl, x, y, w, h, paras, size=24, color=INK, bold=False, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, ls=1.1, sa=0):
    tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame; tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = anchor
    if isinstance(paras, str): paras = [paras]
    for i, p in enumerate(paras):
        para = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        para.alignment = align; para.line_spacing = ls; para.space_after = Pt(sa)
        for t, o in (p if isinstance(p, list) else [(p, {})]):
            r = para.add_run(); r.text = t
            r.font.name = FONT; r.font.size = Pt(o.get("size", size)); r.font.bold = o.get("bold", bold)
            r.font.color.rgb = o.get("color", color)
    return tb


def pic(sl, path, x, y, w=None, h=None):
    im = Image.open(path); r = im.width / im.height
    if w is None: w = h * r
    if h is None: h = w / r
    sl.shapes.add_picture(path, Inches(x), Inches(y), Inches(w), Inches(h))
    return w, h


n = [0]
def slide(title, notes=None):
    sl = prs.slides.add_slide(prs.slide_layouts[6]); n[0] += 1
    text(sl, 0.7, 0.45, 11.9, 1.0, title, size=36, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE)
    rect(sl, 0.7, 1.5, 1.2, 0.07, fill=TURQ)
    text(sl, 12.2, 6.95, 0.6, 0.35, str(n[0]), size=18, color=GRAY, align=PP_ALIGN.RIGHT)
    if notes: sl.notes_slide.notes_text_frame.text = notes
    return sl


def box(sl, x, y, w, h, label, fill=LBLUE, line=BLUE, color=INK, size=22, bold=True):
    b = rect(sl, x, y, w, h, fill=fill, line=line, shape=SH.ROUNDED_RECTANGLE, lw=2); b.adjustments[0] = 0.15
    text(sl, x + 0.1, y, w - 0.2, h, label.split("\n"), size=size, bold=bold, color=color, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, ls=1.0)


def arrow(sl, x, cy, w=0.45, h=0.4):
    rect(sl, x, cy - h / 2, w, h, fill=NAVY, shape=SH.RIGHT_ARROW)


# ------------------------------------------------------------------ 1 titulo
sl = prs.slides.add_slide(prs.slide_layouts[6]); n[0] += 1
rect(sl, 0, 0, W, H, fill=NAVY)
text(sl, 0.9, 1.3, 11.5, 2.2, "Formal Verification of Critical Modules in the PX4 Autopilot using Bounded Model Checking",
     size=44, bold=True, color=WHITE, ls=1.05)
rect(sl, 0.9, 3.65, 1.3, 0.08, fill=TURQ)
text(sl, 0.9, 4.0, 12.0, 0.6, "Marcela Alves Rodrigues¹ · Lucas C. Cordeiro² · José Reginaldo H. de Carvalho³",
     size=21, bold=True, color=WHITE)
text(sl, 0.9, 4.75, 11.5, 1.2, ["\u00b9PPGEE \u00b7 \u00b2Dept. of Electrical and Computer Engineering \u00b7 \u00b3Institute of Computing",
                                "Federal University of Amazonas (UFAM), Manaus, Brazil"],
     size=20, color=RGBColor(0xCF, 0xE0, 0xF0), sa=4)
rect(sl, 0, 6.15, W, 1.35, fill=WHITE)
pic(sl, f"{PS}/logos/l10.png", 0.9, 6.3, h=1.05)
text(sl, 4.4, 6.35, 6.0, 1.0, ["XVI Symposium on Computing Systems Engineering", "November 24–27, 2026 · Salvador, BA"],
     size=20, color=NAVY, anchor=MSO_ANCHOR.MIDDLE, sa=2)
pic(sl, f"{PS}/logos/ufam_round.png", 11.5, 6.25, h=1.15)
sl.notes_slide.notes_text_frame.text = "Apresentar-se e dizer em uma frase: verificamos módulos do PX4 com model checking limitado e analisamos cada contraexemplo."

# ------------------------------------------------------------------ 2 motivacao
sl = slide("Why PX4 needs more than unit tests",
           "PX4 roda em microcontroladores sem MMU: todos os módulos dividem o mesmo espaço de endereços. Um erro num driver pode afetar o estado de voo.")
text(sl, 0.9, 1.95, 11.5, 2.6, [
    [("About 710,000 lines", {"bold": True, "color": NAVY}), (" of C/C++ in the source tree", {})],
    [("No MMU: ", {"bold": True, "color": NAVY}), ("all modules share one address space", {})],
    [("One faulty driver", {"bold": True, "color": NAVY}), (" can corrupt flight-critical state", {})]],
    size=28, sa=12)
bw, gp = 1.95, 0.55
x0 = 0.9; yb = 5.0
rb = rect(sl, x0 + bw + gp - 0.2, yb - 0.55, 3 * bw + 2 * gp + 0.4, 1.95, line=BLUE, lw=3, shape=SH.ROUNDED_RECTANGLE)
rb.adjustments[0] = 0.08; rb.line.dash_style = MSO_LINE.DASH
text(sl, x0 + bw + gp - 0.2, yb - 0.5, 3 * bw + 2 * gp + 0.4, 0.4, "one shared address space", size=20, bold=True, color=BLUE, align=PP_ALIGN.CENTER)
for k, lab in enumerate(["Sensors", "PX4\ndrivers", "uORB\nbus", "Flight\ncontrol", "Vehicle\n(motors)"]):
    bx = x0 + k * (bw + gp); edge = k in (0, 4)
    box(sl, bx, yb, bw, 1.3, lab, fill=LGRAY if edge else LBLUE, line=LINE if edge else BLUE, size=22)
    if k < 4: arrow(sl, bx + bw + 0.06, yb + 0.65, w=gp - 0.12)
rect(sl, x0 + 4 * (bw + gp) + bw / 2 - 0.02, yb + 1.3, 0.04, 0.42, fill=NAVY)
rect(sl, x0 + bw / 2 - 0.02, yb + 1.7, 4 * (bw + gp) + 0.04, 0.04, fill=NAVY)
rect(sl, x0 + bw / 2 - 0.02, yb + 1.45, 0.04, 0.27, fill=NAVY)
rect(sl, x0 + bw / 2 - 0.14, yb + 1.3, 0.28, 0.22, fill=NAVY, shape=SH.ISOSCELES_TRIANGLE)

# ------------------------------------------------------------------ 3 pergunta
sl = slide("Our question",
           "Deixar claro o escopo: harnesses feitos a partir de trechos dos módulos, não o autopiloto inteiro.")
text(sl, 0.9, 2.0, 11.5, 2.0, "Can bounded model checking find safety violations in real PX4 driver code?", size=40, bold=True, color=INK, ls=1.05)
for k, t in enumerate(["12 PX4 modules", "ESBMC and CBMC", "Datasheet assertions"]):
    xx = 0.9 + k * 4.0
    box(sl, xx, 4.4, 3.6, 1.1, t, size=24)
text(sl, 0.9, 6.0, 11.5, 0.8, "Scope: harnesses built from excerpts of the modules, not the whole autopilot.", size=22, color=GRAY)

# ------------------------------------------------------------------ 4 BMC
sl = slide("How bounded model checking works",
           "O verificador codifica o programa e a propriedade como problema SAT/SMT até a profundidade k. Se houver violação, o solver devolve um contraexemplo concreto.")
steps = ["C/C++\nharness", "Symbolic\nexecution", "SAT / SMT\nsolver", "Result"]
sw = 2.5; sg = 0.55
for k, t in enumerate(steps):
    bx = 0.9 + k * (sw + sg)
    box(sl, bx, 2.2, sw, 1.5, t, fill=NAVY if k == 3 else LBLUE, line=NAVY if k == 3 else BLUE, color=WHITE if k == 3 else INK, size=26)
    if k < 3: arrow(sl, bx + sw + 0.07, 2.95, w=sg - 0.14)
box(sl, 9.4, 4.0, 3.2, 1.0, "Counterexample", fill=LGRAY, line=GRAY, color=GRAY, size=22)
box(sl, 5.9, 4.0, 3.2, 1.0, "No violation up to k", fill=LGRAY, line=NAVY, color=NAVY, size=22)
text(sl, 0.9, 5.5, 11.5, 1.2, [[("Important: ", {"bold": True, "color": NAVY}), ("no violation up to k is evidence, not a proof.", {})]], size=28)

# ------------------------------------------------------------------ 5 oito etapas
sl = slide("Eight stages, from selection to CI",
           "As etapas 1, 4, 5 e 6 seguem o fluxo de Sousa et al. As etapas 2, 3, 7 e 8 são o que acrescentamos para firmware.")
stages = ["Select modules", "Hardware stubs", "Datasheet properties", "Run ESBMC",
          "Analyze counterexamples", "Report to PX4", "Cross-check with CBMC", "ArduPilot and CI"]
cw, cg = 2.85, 0.2
for k, st in enumerate(stages):
    r_, c_ = divmod(k, 4)
    cx = 0.9 + c_ * (cw + cg); cy = 2.0 + r_ * 2.0
    added = k in (1, 2, 6, 7)
    b = rect(sl, cx, cy, cw, 1.7, fill=NAVY if added else LBLUE, line=NAVY, shape=SH.ROUNDED_RECTANGLE, lw=2); b.adjustments[0] = 0.12
    text(sl, cx, cy + 0.12, cw, 0.5, str(k + 1), size=26, bold=True, color=WHITE if added else BLUE, align=PP_ALIGN.CENTER)
    text(sl, cx + 0.15, cy + 0.62, cw - 0.3, 1.0, st, size=20, bold=True, color=WHITE if added else NAVY, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE, ls=1.0)
rect(sl, 0.9, 6.25, 0.4, 0.3, fill=NAVY)
text(sl, 1.45, 6.18, 10, 0.5, "our additions to the adopted workflow", size=20, color=GRAY, anchor=MSO_ANCHOR.MIDDLE)

# ------------------------------------------------------------------ 6 modulos
sl = slide("Twelve modules, eight input classes",
           "Escolhidos por impacto de segurança no caminho sensor-atuador e pela viabilidade de isolar com stubs. Isso introduz viés de seleção, tratado nas limitações.")
mods = [("GPS uBlox", "Serial sensor"), ("IMU BMI088", "Serial sensor"), ("MAVLink parser", "Telemetry"), ("Expo RC", "Radio"),
        ("Battery monitor", "Power sensor"), ("PWM output", "Actuation"), ("RC update", "Radio"), ("Navigator / Mission", "Mission logic"),
        ("EKF2 covariance", "State estimation"), ("Magnetometer", "Sensor aggregation"), ("Barometer", "Sensor aggregation"), ("VTOL", "Actuation")]
for k, (m, c) in enumerate(mods):
    col, row = divmod(k, 6)
    x = 0.9 + col * 6.1; y = 1.95 + row * 0.82
    rect(sl, x, y + 0.72, 5.6, 0.02, fill=LINE)
    text(sl, x, y, 3.0, 0.7, m, size=22, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE)
    text(sl, x + 3.0, y, 2.6, 0.7, c, size=20, color=GRAY, anchor=MSO_ANCHOR.MIDDLE)

# ------------------------------------------------------------------ 7 setup
sl = slide("Experimental setup",
           "Tempos são a mediana de três execuções. Com --no-unwinding-assertions, laços além do limite são truncados em silêncio; isso está nas ameaças à validade.")
for k, (lab, big, small) in enumerate([("Machine", "Intel Core i7-13700K", "64 GB · Ubuntu 24.04"),
                                        ("Verifier", "ESBMC v8.1.0", "Boolector 3.2.4"),
                                        ("Default bound", "k = 10", "120 s timeout · median of 3 runs")]):
    x = 0.9 + k * 4.0
    rect(sl, x, 2.0, 3.7, 2.6, fill=LGRAY, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.06
    rect(sl, x, 2.2, 0.12, 2.2, fill=TURQ)
    text(sl, x + 0.4, 2.2, 3.2, 0.4, lab.upper(), size=18, bold=True, color=GRAY)
    text(sl, x + 0.4, 2.75, 3.2, 0.9, big, size=24, bold=True, color=NAVY, ls=1.0)
    text(sl, x + 0.4, 3.65, 3.2, 0.9, small, size=20, color=INK)
text(sl, 0.9, 5.1, 11.5, 1.6, [
    [("Adjusted per module: ", {"bold": True, "color": NAVY}), ("MAVLink k = 300 · Navigator k = 15 · VTOL --no-library", {})],
    [("With --no-unwinding-assertions, loops beyond the bound are silently truncated.", {"color": GRAY})]],
    size=22, sa=10)

# ------------------------------------------------------------------ 8 numeros
sl = slide("What came out",
           "Dez dos doze harnesses não têm violação até o limite. As violações estão em dois harnesses: GPS uBlox e IMU BMI088 (três achados no total).")
tiles = [("10 of 12", "harnesses with no violation up to the bound", NAVY),
         ("3 findings", "in 2 harnesses: GPS uBlox and IMU BMI088", GRAY),
         ("0.1–64.9 s", "verification time per harness", NAVY),
         ("≈ 94 s", "total for the ten completed harnesses (≈ 159 s with GPS)", NAVY)]
for k, (big, small, col) in enumerate(tiles):
    r_, c_ = divmod(k, 2)
    x = 0.9 + c_ * 6.1; y = 2.0 + r_ * 2.35
    rect(sl, x, y, 5.7, 2.1, fill=LGRAY, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.07
    rect(sl, x, y + 0.25, 0.12, 1.6, fill=col)
    text(sl, x + 0.45, y + 0.15, 5.0, 0.95, big, size=44, bold=True, color=col, anchor=MSO_ANCHOR.MIDDLE)
    text(sl, x + 0.45, y + 1.1, 5.0, 0.9, small, size=21, color=INK, ls=1.05)

# ------------------------------------------------------------------ 9 grafico
sl = slide("Time per harness",
           "Os harnesses com violação (GPS e IMU) param na primeira violação, então o tempo mostrado é o de decisão.")
data = [("GPS uBlox", 64.9, 1), ("MAVLink parser", 49.2, 0), ("EKF2 covariance", 12.2, 0), ("RC update", 9.5, 0),
        ("Battery monitor", 6.7, 0), ("Magnetometer", 6.6, 0), ("Expo RC", 4.9, 0), ("VTOL", 4.0, 0),
        ("Barometer", 0.4, 0), ("PWM output", 0.2, 0), ("IMU BMI088", 0.1, 1), ("Navigator / Mission", 0.1, 0)]
LW = 3.2; BX = 0.9 + LW + 0.2; BW = 7.2; RH = 0.38; y0 = 1.85
for i, (nm, v, ce) in enumerate(data):
    y = y0 + i * RH
    if i % 2 == 0: rect(sl, 0.9, y, 11.7, RH, fill=LGRAY)
    text(sl, 0.9, y, LW, RH, nm, size=18, color=INK, align=PP_ALIGN.RIGHT, anchor=MSO_ANCHOR.MIDDLE)
    bw = max(BW * v / 65.0, 0.1)
    rect(sl, BX, y + 0.06, bw, RH - 0.12, fill=GRAY if ce else NAVY)
    text(sl, BX + bw + 0.15, y, 1.2, RH, f"{v:.1f}", size=18, bold=True, color=INK, anchor=MSO_ANCHOR.MIDDLE)
yl = y0 + 12 * RH + 0.2
rect(sl, 5.0, yl + 0.1, 0.35, 0.3, fill=NAVY); text(sl, 5.5, yl, 2.2, 0.5, "no violation", size=18, color=GRAY, anchor=MSO_ANCHOR.MIDDLE)
rect(sl, 8.0, yl + 0.1, 0.35, 0.3, fill=GRAY); text(sl, 8.5, yl, 2.5, 0.5, "counterexample", size=18, color=GRAY, anchor=MSO_ANCHOR.MIDDLE)
text(sl, 0.9, yl, 3.5, 0.5, "seconds, median of 3 runs", size=18, color=GRAY, anchor=MSO_ANCHOR.MIDDLE)

# ------------------------------------------------------------------ 10 achados
sl = slide("A counterexample is where the analysis starts",
           "Cada contraexemplo foi conferido contra o código de produção. Nenhum dos três é um defeito confirmado.")
cards = [("GPS uBlox", "Model-only", "Out-of-bounds memcpy in the harness. The production driver clamps the length, so the state is not reachable there."),
         ("BMI088 FIFO", "False positive", "The harness left out three guards that the real driver has."),
         ("BMI088 temperature", "Contested", "86.25 °C against an 85 °C datasheet limit. The upstream change was declined.")]
for k, (t, tag, body) in enumerate(cards):
    x = 0.9 + k * 4.0
    rect(sl, x, 1.95, 3.7, 4.0, fill=LGRAY, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.05
    text(sl, x + 0.3, 2.15, 3.2, 0.8, t, size=26, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE, ls=1.0)
    pl = rect(sl, x + 0.3, 3.05, 2.2, 0.6, fill=GRAY, shape=SH.ROUNDED_RECTANGLE); pl.adjustments[0] = 0.4
    text(sl, x + 0.3, 3.05, 2.2, 0.6, tag, size=20, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(sl, x + 0.3, 3.9, 3.2, 2.0, body, size=20, color=INK, ls=1.1)
text(sl, 0.9, 6.25, 11.5, 0.6, "None of the three is a confirmed production bug.", size=24, bold=True, color=NAVY)

# ------------------------------------------------------------------ 11 GPS
sl = slide("GPS uBlox: an unsigned underflow",
           "No harness a capacidade é 200 e len=207; 200-207 dá -7 em int e vira um size_t enorme na comparação. No driver real o laço de escrita limita o tamanho. A proposta de guarda defensiva é o PR #27576.")
w_, h_ = pic(sl, f"{PS}/build/terminal_gps.png", 0.9, 1.95, w=6.4)
im = Image.open(f"{PS}/build/eq2_t.png")
pic(sl, f"{PS}/build/eq2_t.png", 0.9, 1.95 + h_ + 0.5, w=6.4)
text(sl, 7.8, 1.95, 4.8, 4.8, [
    [("In the harness", {"bold": True, "color": NAVY})],
    "the length check is always false, so memcpy writes out of bounds.",
    [("In production", {"bold": True, "color": NAVY})],
    "the write loop clamps the length. The state is not reachable.",
    [("Outcome", {"bold": True, "color": NAVY})],
    "A defensive guard was proposed upstream (PX4 PR #27576)."],
    size=22, sa=6, ls=1.08)

# ------------------------------------------------------------------ 12 limites
sl = slide("What these results do not show",
           "Resultado de assurance limitado e em nível de modelo; não é prova sobre o PX4 inteiro.")
rows = [("Bounded depth", "k = 10 by default; deeper paths may be missed."),
        ("Stubs", "Nondeterministic stubs can admit states the real driver rejects."),
        ("Selected modules", "Stub feasibility under-represents tightly coupled code."),
        ("Solver", "Some outcomes changed with solver or configuration.")]
for k, (a, b) in enumerate(rows):
    y = 1.95 + k * 1.05
    rect(sl, 0.9, y + 0.95, 11.7, 0.02, fill=LINE)
    text(sl, 0.9, y, 3.4, 0.9, a, size=26, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE)
    text(sl, 4.4, y, 8.2, 0.9, b, size=24, color=INK, anchor=MSO_ANCHOR.MIDDLE)
b = rect(sl, 0.9, 6.25, 11.7, 0.65, fill=LBLUE, shape=SH.ROUNDED_RECTANGLE); b.adjustments[0] = 0.2
text(sl, 1.2, 6.25, 11.2, 0.65, "Bounded, model-level evidence. Not a proof about the whole of PX4.", size=24, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE)

# ------------------------------------------------------------------ 13 conclusao
sl = slide("Conclusion",
           "Mensagem final: o resultado importante não é achar um contraexemplo, é saber se ele existe no código de produção.")
text(sl, 0.9, 1.95, 5.8, 0.5, "WHAT WE LEARNED", size=18, bold=True, color=GRAY)
text(sl, 0.9, 2.5, 5.7, 4.3, [
    "BMC is feasible for harnesses of selected PX4 modules.",
    "10 of 12 harnesses are clean within the bound, each in at most 65 s.",
    [("The key step is checking each counterexample against production code.", {"bold": True, "color": NAVY})]],
    size=26, sa=16, ls=1.1)
rect(sl, 6.95, 1.95, 0.03, 4.6, fill=LINE)
text(sl, 7.4, 1.95, 5.2, 0.5, "NEXT", size=18, bold=True, color=GRAY)
for k, t in enumerate(["k-induction for stronger evidence", "Commander, failsafe and geofence modules", "PX4 CI with hardware-in-the-loop"]):
    y = 2.6 + k * 1.3
    c = rect(sl, 7.4, y + 0.1, 0.7, 0.7, fill=BLUE, shape=SH.OVAL)
    text(sl, 7.4, y + 0.1, 0.7, 0.7, str(k + 1), size=22, bold=True, color=WHITE, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    text(sl, 8.35, y, 4.3, 0.95, t, size=24, color=INK, anchor=MSO_ANCHOR.MIDDLE, ls=1.05)

# ------------------------------------------------------------------ 14 obrigado
sl = prs.slides.add_slide(prs.slide_layouts[6]); n[0] += 1
rect(sl, 0, 0, W, H, fill=NAVY)
text(sl, 0.9, 1.6, 7, 1.2, "Thank you", size=60, bold=True, color=WHITE)
text(sl, 0.9, 2.9, 7, 0.8, "Questions?", size=32, color=RGBColor(0xCF, 0xE0, 0xF0))
rect(sl, 0.9, 3.9, 1.3, 0.08, fill=TURQ)
text(sl, 0.9, 4.3, 7, 1.0, ["Harnesses and logs:", "doi.org/10.5281/zenodo.23282236"], size=22, color=WHITE, sa=4)
text(sl, 0.9, 5.7, 8, 1.2, "Part of the SAGRES Project. Supported by CAPES (Finance Code 001) and FAPEAM.", size=20,
     color=RGBColor(0xCF, 0xE0, 0xF0))
rect(sl, 9.0, 1.9, 3.5, 3.5, fill=WHITE, shape=SH.ROUNDED_RECTANGLE).adjustments[0] = 0.05
pic(sl, f"{PS}/build/qr.png", 9.2, 2.1, w=3.1)
text(sl, 9.0, 5.55, 3.5, 0.5, "Scan for the artifact", size=20, color=WHITE, align=PP_ALIGN.CENTER)
prs.save(OUT)
print("slides:", n[0])
