import sys
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE as SH
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION
from PIL import Image

PS = "../poster_sbesc"
OUT = sys.argv[1]
NAVY = RGBColor(0x0B, 0x3A, 0x63); BLUE = RGBColor(0x1F, 0x6F, 0xB2)
INK = RGBColor(0x22, 0x2B, 0x35); GRAY = RGBColor(0x66, 0x70, 0x7C); LGRAY = RGBColor(0xEE, 0xF1, 0xF4)
WHITE = RGBColor(0xFF, 0xFF, 0xFF); MID = RGBColor(0x7A, 0x86, 0x93)
FONT = "Arial"
W, H = 13.333, 7.5
prs = Presentation(); prs.slide_width, prs.slide_height = Inches(W), Inches(H)
cnt = [0]


def rect(sl, x, y, w, h, fill):
    s = sl.shapes.add_shape(SH.RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    s.fill.solid(); s.fill.fore_color.rgb = fill; s.line.fill.background(); s.shadow.inherit = False
    st = s._element.find('{http://schemas.openxmlformats.org/presentationml/2006/main}style')
    if st is not None: s._element.remove(st)
    return s


def text(sl, x, y, w, h, paras, size=26, color=INK, bold=False, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, sa=10, ls=1.12):
    tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame; tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = anchor
    if isinstance(paras, str): paras = [paras]
    for i, p in enumerate(paras):
        para = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        para.alignment = align; para.space_after = Pt(sa); para.line_spacing = ls
        for t, o in (p if isinstance(p, list) else [(p, {})]):
            r = para.add_run(); r.text = t
            r.font.name = o.get("font", FONT); r.font.size = Pt(o.get("size", size))
            r.font.bold = o.get("bold", bold); r.font.color.rgb = o.get("color", color)
    return tb


def pic(sl, path, x, y, w=None, h=None):
    im = Image.open(path); r = im.width / im.height
    if w is None: w = h * r
    if h is None: h = w / r
    sl.shapes.add_picture(path, Inches(x), Inches(y), Inches(w), Inches(h)); return w, h


def listing(sl, x, y, w, lines, size=20):
    h = 0.42 * len(lines) + 0.3
    b = sl.shapes.add_shape(SH.RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    b.fill.solid(); b.fill.fore_color.rgb = RGBColor(0xF7, 0xF8, 0xFA); b.line.color.rgb = RGBColor(0x9A, 0xA4, 0xAF)
    b.shadow.inherit = False
    st = b._element.find('{http://schemas.openxmlformats.org/presentationml/2006/main}style')
    if st is not None: b._element.remove(st)
    text(sl, x + 0.25, y + 0.15, w - 0.4, h - 0.2, [[(l, {"font": "Courier New", "size": size, "bold": l.startswith("VERIF")})] for l in lines],
         size=size, sa=0, ls=1.15)
    return h


def slide(title, notes=None):
    sl = prs.slides.add_slide(prs.slide_layouts[6]); cnt[0] += 1
    text(sl, 0.8, 0.45, 11.8, 0.9, title, size=36, bold=True, color=NAVY, anchor=MSO_ANCHOR.MIDDLE)
    pic(sl, f"{PS}/logos/l10.png", 0.8, 6.85, h=0.4)
    text(sl, 11.8, 6.9, 0.8, 0.4, str(cnt[0]), size=18, color=GRAY, align=PP_ALIGN.RIGHT)
    if notes: sl.notes_slide.notes_text_frame.text = notes
    return sl


def table(sl, x, y, colw, rows, size=20, rowh=0.4, bold_first=True):
    gf = sl.shapes.add_table(len(rows), len(colw), Inches(x), Inches(y), Inches(sum(colw)), Inches(rowh * len(rows)))
    t = gf.table
    tblPr = gf._element.graphic.graphicData.tbl.tblPr
    tblPr.set('bandRow', '0'); tblPr.set('firstRow', '0')
    for c, w in enumerate(colw): t.columns[c].width = Inches(w)
    for r, row in enumerate(rows):
        t.rows[r].height = Inches(rowh)
        for c, val in enumerate(row):
            cell = t.cell(r, c); cell.fill.solid()
            cell.fill.fore_color.rgb = NAVY if r == 0 else (LGRAY if r % 2 == 0 else WHITE)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.margin_left = cell.margin_right = Inches(0.12); cell.margin_top = cell.margin_bottom = Inches(0.02)
            p = cell.text_frame.paragraphs[0]; run = p.add_run(); run.text = str(val)
            run.font.name = FONT; run.font.size = Pt(size)
            run.font.bold = (r == 0) or (c == 0 and bold_first)
            run.font.color.rgb = WHITE if r == 0 else INK
    return gf


# ---------------------------------------------------------------- 1 titulo
sl = prs.slides.add_slide(prs.slide_layouts[6]); cnt[0] += 1
pic(sl, f"{PS}/logos/ufam_round.png", 0.8, 0.6, h=1.3)
pic(sl, f"{PS}/logos/l10.png", 9.9, 0.7, w=2.7)
text(sl, 0.8, 2.3, 11.8, 2.0, "Formal Verification of Critical Modules in the PX4 Autopilot using Bounded Model Checking",
     size=42, bold=True, color=NAVY, ls=1.05)
text(sl, 0.8, 4.75, 11.8, 0.6, "Marcela Alves Rodrigues¹, Lucas C. Cordeiro², José Reginaldo H. de Carvalho³", size=22, bold=True, color=INK)
text(sl, 0.8, 5.4, 11.8, 1.0, ["¹PPGEE  ²Dept. of Electrical and Computer Engineering  ³Institute of Computing",
                               "Federal University of Amazonas (UFAM), Manaus, Brazil"], size=20, color=GRAY, sa=4)
text(sl, 0.8, 6.6, 11.8, 0.5, "SBESC 2026 · Salvador, November 24–27", size=20, color=GRAY)
sl.notes_slide.notes_text_frame.text = "Apresentação em uma frase: verificamos módulos do PX4 com model checking limitado e conferimos cada contraexemplo no código de produção."

# ---------------------------------------------------------------- 2 PX4
sl = slide("Why PX4?", "PX4 roda em microcontroladores sem MMU: todos os módulos compartilham o espaço de endereços. Os módulos em verde são os cobertos por harnesses.")
text(sl, 0.8, 1.8, 5.2, 4.8, [
    [("About 710k lines", {"bold": True}), (" of C/C++", {})],
    [("No MMU", {"bold": True}), (": every module shares one address space", {})],
    [("So a defect in one driver can corrupt flight-critical state", {})]], size=28, sa=18)
w_, h_ = pic(sl, "fig/fig1_uorb.png", 6.3, 1.9, w=6.3)
text(sl, 6.3, 1.9 + h_ + 0.15, 6.3, 0.8, "Green: modules covered by a harness (Fig. 1 of the paper)", size=20, color=GRAY)

# ---------------------------------------------------------------- 3 RQs
sl = slide("What we asked", "As três perguntas de pesquisa do artigo.")
for k, (rq, q) in enumerate([("RQ1", "What is the per-module verification latency on commodity hardware?"),
                             ("RQ2", "Does bounded model checking expose violations that the existing PX4 unit tests do not exercise?"),
                             ("RQ3", "Which per-module configurations are needed to make each module tractable?")]):
    y = 1.9 + k * 1.6
    text(sl, 0.8, y, 1.4, 0.8, rq, size=32, bold=True, color=BLUE)
    text(sl, 2.3, y, 10.2, 1.3, q, size=28)

# ---------------------------------------------------------------- 4 BMC
sl = slide("Bounded model checking in one slide", "A fórmula é a Eq. (1) do artigo. Ausência de violação até k não é uma prova.")
pic(sl, f"{PS}/build/eq1_t.png", 1.2, 1.8, w=10.9)
text(sl, 0.8, 4.2, 11.8, 2.0, [
    "The program is unrolled up to depth k and encoded as an SMT problem",
    "A satisfying assignment is a concrete counterexample",
    [("No violation up to k is evidence, not a proof", {"bold": True, "color": NAVY})]], size=26, sa=12)

# ---------------------------------------------------------------- 5 metodologia
sl = slide("Eight-stage methodology", "Etapas 1 e 4 a 6 seguem Sousa et al. Em verde, as etapas 7 e 8; as etapas 2 e 3 também são adições do trabalho.")
w_, h_ = pic(sl, "fig/fig2_pipeline.png", 0.8, 1.6, h=4.4)
text(sl, 0.8 + w_ + 0.5, 1.8, 12.6 - (0.8 + w_ + 0.5), 4.5, [
    "We add stages 2, 3, 7 and 8 to the workflow of Sousa et al.",
    "Each counterexample is checked against the unmodified driver"], size=22, sa=16)

# ---------------------------------------------------------------- 6 modulos
sl = slide("The twelve modules", "Selecionados por impacto de segurança e viabilidade de isolar com stubs (viés de seleção, nas limitações). Os tamanhos são linhas de código.")
rows = [("Module", "Input class", "Target LoC", "Harness LoC"),
        ("GPS uBlox", "Serial sensor", "1712", "294"), ("IMU BMI088", "Serial sensor", "700", "319"),
        ("MAVLink", "Telemetry link", "3624", "82"), ("Expo RC", "Radio input", "303", "129"),
        ("Battery monitor", "Power sensor", "449", "242"), ("PWM output", "Actuation", "337", "208"),
        ("RC update", "Radio input", "769", "253"), ("Navigator / Mission", "Mission logic", "497", "291"),
        ("EKF2 covariance", "State estimation", "385", "296"), ("Magnetometer", "Serial sensor", "792", "239"),
        ("Barometer", "Serial sensor", "246", "278"), ("VTOL", "Actuation", "591", "146")]
table(sl, 0.8, 1.45, [3.6, 3.4, 2.2, 2.4], rows, size=20, rowh=0.4)

# ---------------------------------------------------------------- 7 setup
sl = slide("Setup", "Tempos são a mediana de três execuções. Com --no-unwinding-assertions, laços além de k são truncados em silêncio.")
text(sl, 0.8, 1.8, 11.8, 4.8, [
    [("Machine: ", {"bold": True, "color": NAVY}), ("Intel i7-13700K, 64 GB, Ubuntu 24.04", {})],
    [("Verifier: ", {"bold": True, "color": NAVY}), ("ESBMC v8.1.0 with Boolector 3.2.4", {})],
    [("Code: ", {"bold": True, "color": NAVY}), ("PX4 main, commit 8be2590c15", {})],
    [("Bound: ", {"bold": True, "color": NAVY}), ("k = 10, 120 s timeout, median of 3 runs", {})],
    [("Exceptions: ", {"bold": True, "color": NAVY}), ("MAVLink k = 300, Navigator k = 15, VTOL with --no-library", {})]],
    size=28, sa=20)

# ---------------------------------------------------------------- 8 resultados
sl = slide("10 of 12 harnesses verify clean", "RQ1: todo harness verifica em menos de 90 s. Os dois com violação (GPS e IMU) param na primeira violação.")
data = [("GPS uBlox", 64.9, 1), ("MAVLink Parser", 49.2, 0), ("EKF2 Covariance", 12.2, 0), ("RC Update", 9.5, 0),
        ("Battery Monitor", 6.7, 0), ("Magnetometer", 6.6, 0), ("Expo RC", 4.9, 0), ("VTOL", 4.0, 0),
        ("Barometer", 0.4, 0), ("PWM Output", 0.2, 0), ("IMU BMI088", 0.1, 1), ("Navigator/Mission", 0.1, 0)]
cd = CategoryChartData(); cd.categories = [d[0] for d in reversed(data)]; cd.add_series("Time (s)", [d[1] for d in reversed(data)])
gf = sl.shapes.add_chart(XL_CHART_TYPE.BAR_CLUSTERED, Inches(0.8), Inches(1.4), Inches(7.6), Inches(5.3), cd)
ch = gf.chart; ch.has_legend = False; ch.has_title = False
ch.font.size = Pt(18); ch.font.name = FONT; ch.font.color.rgb = INK
pl = ch.plots[0]; pl.gap_width = 40; pl.has_data_labels = True
dl = pl.data_labels; dl.number_format = '0.0'; dl.number_format_is_linked = False
dl.position = XL_LABEL_POSITION.OUTSIDE_END; dl.font.size = Pt(18); dl.font.bold = True
for i, d in enumerate(reversed(data)):
    pt = pl.series[0].points[i]; pt.format.fill.solid(); pt.format.fill.fore_color.rgb = MID if d[2] else NAVY
va = ch.value_axis; va.maximum_scale = 80; va.minimum_scale = 0; va.major_unit = 20
va.has_major_gridlines = False; va.visible = False
ch.category_axis.format.line.color.rgb = RGBColor(0xC8, 0xD1, 0xDA); ch.category_axis.tick_labels.font.size = Pt(18)
text(sl, 8.8, 1.7, 3.9, 5.0, [
    [("0.1 to 64.9 s", {"bold": True, "size": 32, "color": NAVY})], "per harness, median of 3 runs",
    [("≈94 s", {"bold": True, "size": 32, "color": NAVY})], "for the ten completed harnesses",
    [("≈159 s", {"bold": True, "size": 32, "color": NAVY})], "with GPS (64.9 s)"], size=22, sa=6)
text(sl, 8.8, 6.25, 3.9, 0.5, "gray = violation found", size=20, color=GRAY)

# ---------------------------------------------------------------- 9 achados
sl = slide("Three findings, none a confirmed bug", "Cada um foi conferido contra o código de produção. O GPS só existe no modelo; o FIFO é falso positivo; a temperatura é contestada.")
text(sl, 0.8, 1.7, 11.8, 5.0, [
    [("GPS uBlox", {"bold": True, "color": NAVY}), ("  violation in the harness, not reachable in the production driver", {})],
    [("BMI088 FIFO", {"bold": True, "color": NAVY}), ("  false positive: the harness left out three guards", {})],
    [("BMI088 temperature", {"bold": True, "color": NAVY}), ("  86.25 °C above the 85 °C limit; contested, PX4 declined it", {})]],
    size=28, sa=26)
text(sl, 0.8, 5.7, 11.8, 0.8, "Two harnesses, three candidates; the counterexample is where the analysis starts.", size=24, color=GRAY)

# ---------------------------------------------------------------- 10 GPS
sl = slide("GPS uBlox: an unsigned underflow", "No harness a capacidade é 200 e len = 207. 200 - 207 = -7 vira um size_t enorme na comparação. No driver real o laço de escrita limita write_len. O PR #27576 propõe uma guarda defensiva.")
h_ = listing(sl, 0.8, 1.6, 6.9, ["[Counterexample]", "State 3:  input_len = 10", "State 8:  dump_buffer.len = 207", "State 21: __memcpy_impl, line 280", "Violated property:", "  dereference failure: access to", "  object out of bounds", "VERIFICATION FAILED"])
pic(sl, f"{PS}/build/eq2_t.png", 0.8, 1.6 + h_ + 0.4, w=6.9)
text(sl, 8.0, 1.6, 4.6, 5.0, [
    [("In the harness", {"bold": True, "color": NAVY})], "the length check is always false, so memcpy writes out of bounds",
    [("In production", {"bold": True, "color": NAVY})], "the write loop clamps the length; the state is not reachable",
    [("Upstream", {"bold": True, "color": NAVY})], "we proposed a defensive guard (PR #27576)"], size=22, sa=6)

# ---------------------------------------------------------------- 11 temperatura
sl = slide("BMI088 temperature: 86.25 °C", "O revisor do PX4 objetou a limitar o valor (\"faking the number\") e perguntou o que o sensor reporta fora da faixa; o PR foi fechado e a issue encerrada como not planned.")
h_ = listing(sl, 0.8, 1.6, 6.9, ["[Counterexample]", "State 2:  temp_msb = 63, temp_lsb = 95", "State 7:  Temp_uint11 = 506", "State 11: temperature = 86.250000f", "Violated property: assertion", "  temperature >= -40.0f &&", "  temperature <= 85.0f", "VERIFICATION FAILED"])
text(sl, 8.0, 1.6, 4.6, 5.0, [
    "The driver does not validate temperature; the datasheet range is −40 to +85 °C",
    "A PX4 reviewer objected to clamping the value",
    "The PR was closed and the issue closed as not planned",
    [("We keep it as a design note", {"bold": True, "color": NAVY})]], size=24, sa=14)

# ---------------------------------------------------------------- 12 ferramentas
sl = slide("Which tool did what", "ESBMC é a ferramenta principal. CBMC foi segunda opinião em dois harnesses e a ferramenta usada no ArduPilot.")
text(sl, 0.8, 1.7, 11.8, 5.0, [
    [("ESBMC", {"bold": True, "color": NAVY}), ("  all 12 harnesses; every time and counterexample", {})],
    [("CBMC", {"bold": True, "color": NAVY}), ("  cross-check on IMU BMI088 (agreed) and GPS uBlox (only at reduced bounds)", {})],
    [("ArduPilot", {"bold": True, "color": NAVY}), ("  CBMC on its BMI088 driver reproduces the missing range check; the maintainer declined it", {})],
    [("CI", {"bold": True, "color": NAVY}), ("  a GitHub Actions workflow in a PX4 fork runs the IMU and GPS harnesses on every push", {})]],
    size=26, sa=22)

# ---------------------------------------------------------------- 13 limites
sl = slide("What the results do not show", "Evidência limitada e em nível de modelo.")
text(sl, 0.8, 1.8, 11.8, 4.5, [
    "Bounded at k = 10; with no unwinding assertions, longer loops are silently truncated",
    "Stubs over-approximate the hardware, so a harness can admit states the real driver rejects",
    "Modules were chosen by stub feasibility, so the sample is not representative of PX4",
    "Some results depend on the solver: Z3 gave no verdict on the GPS harness"], size=24, sa=12)
text(sl, 0.8, 6.2, 11.8, 0.6, "Bounded, model-level evidence. Not a proof about PX4.", size=26, bold=True, color=NAVY)

# ---------------------------------------------------------------- 14 conclusao
sl = slide("Conclusion", "O que importa é saber se o contraexemplo existe no código de produção.")
text(sl, 0.8, 1.8, 6.0, 4.8, [
    "Bounded model checking is feasible for harnesses of selected PX4 modules",
    "10 of 12 harnesses are clean up to the bound",
    [("Each counterexample must be checked against production code", {"bold": True, "color": NAVY})]], size=28, sa=20)
text(sl, 7.3, 1.8, 5.3, 0.6, "Next", size=28, bold=True, color=BLUE)
text(sl, 7.3, 2.5, 5.3, 4.0, ["k-induction", "commander, failsafe and geofence modules", "PX4 CI with hardware-in-the-loop"], size=28, sa=16)

# ---------------------------------------------------------------- 15 obrigado
sl = prs.slides.add_slide(prs.slide_layouts[6]); cnt[0] += 1
text(sl, 0.8, 1.6, 7.0, 1.2, "Thank you", size=54, bold=True, color=NAVY)
text(sl, 0.8, 2.9, 7.0, 0.8, "Questions?", size=32, color=GRAY)
text(sl, 0.8, 4.2, 7.5, 1.4, ["Harnesses and logs", "doi.org/10.5281/zenodo.23282236"], size=24, sa=6)
text(sl, 0.8, 5.6, 7.5, 1.0, "Part of the SAGRES Project. Supported by CAPES (Finance Code 001) and FAPEAM.", size=20, color=GRAY)
pic(sl, f"{PS}/build/qr.png", 9.0, 1.8, w=3.4)
pic(sl, f"{PS}/logos/l10.png", 0.8, 6.8, h=0.45)
prs.save(OUT); print("slides:", cnt[0])
