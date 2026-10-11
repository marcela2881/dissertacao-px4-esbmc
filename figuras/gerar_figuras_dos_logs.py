#!/usr/bin/env python3
"""Gera as figuras dos contraexemplos (temperatura do BMI088 e GPS uBlox)
DIRETAMENTE dos logs versionados do ESBMC 8.1.0 + Boolector 3.2.4.

Nenhum valor e digitado a mao: cada linha da figura e extraida do log por
expressao regular. A unica edicao e de formato: "State N file F line L column C
function G thread 0" vira "State N -- F:L -- G", e cadeias binarias longas sao
encurtadas com "...". Caminhos do ESBMC (/home/runner/...) sao reduzidos ao nome
do arquivo.

Uso (na raiz do repositorio):  python3 figuras/gerar_figuras_dos_logs.py
Saida: figuras/ce_temperatura_bmi088.png e figuras/ce_gps_ublox.png
"""
import os
import re
import sys

from PIL import Image, ImageDraw, ImageFont

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FONTE = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
FONTE_B = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"

FUNDO, TEXTO, APAGADO = (30, 32, 44), (205, 210, 220), (140, 146, 160)
DESTAQUE, ERRO, BORDA = (240, 200, 110), (235, 100, 100), (240, 200, 110)

STATE = re.compile(r"^State (\d+) file (\S+) line (\d+) column \d+ function (\S+) thread \d+")


def ler(rel):
    with open(os.path.join(RAIZ, rel), encoding="utf-8", errors="replace") as f:
        return [l.rstrip("\n") for l in f]


def cabecalho(linhas):
    versao = next(l for l in linhas if l.startswith("ESBMC version"))
    solver = next((l for l in linhas if l.startswith("Solving with solver")), "")
    return versao.split(" 64-bit")[0], solver.replace("Solving with solver ", "")


def contraexemplo(linhas, variaveis):
    """Lista de (texto, estilo) com os estados que atribuem as variaveis pedidas,
    a propriedade violada e o veredito."""
    ini = linhas.index("[Counterexample]")
    out, i = [("[Counterexample]", "erro_b")], ini + 1
    while i < len(linhas):
        m = STATE.match(linhas[i])
        if m and i + 2 < len(linhas):
            n, arq, lin, fun = m.groups()
            k = i + 1                       # pula a linha separadora "-----"
            while k < len(linhas) and set(linhas[k].strip()) <= {"-"}:
                k += 1
            valor = linhas[k].strip()
            nome = valor.split(" = ")[0]
            if nome in variaveis:
                out.append((f"State {n} -- {os.path.basename(arq)}:{lin} -- {fun}", "apagado"))
                out.append(("  " + variaveis[nome](valor), "destaque"))
        if linhas[i].startswith("Violated property:"):
            out.append(("Violated property:", "erro_b"))
            j = i + 1
            while j < len(linhas) and linhas[j].strip() and not linhas[j].startswith("VERIFICATION"):
                t = linhas[j].strip()
                t = re.sub(r"file \S*/", "file ", t)
                t = re.sub(r" column \d+", "", t)
                out.append(("  " + t, "erro"))
                j += 1
        if linhas[i].startswith("VERIFICATION FAILED"):
            out.append(("VERIFICATION FAILED", "erro_b"))
            break
        i += 1
    return out


def curto(valor):  # encurta cadeias binarias longas
    return re.sub(r"\(([01 ]{20,})\)", lambda m: "(..." + m.group(1).replace(" ", "")[-11:] + ")", valor)


def desenhar(linhas, rodape, nota, destino):
    f = ImageFont.truetype(FONTE, 22)
    fb = ImageFont.truetype(FONTE_B, 22)
    fr = ImageFont.truetype(FONTE, 17)
    lh, pad = 34, 40
    larg = max(f.getlength(t) for t, _ in linhas) + 2 * pad + 40
    larg = max(larg, max(fr.getlength(t) for t in rodape + [nota]) + 2 * pad)
    alt = pad * 2 + lh * len(linhas) + 30 + 26 * (len(rodape) + 1) + 20
    img = Image.new("RGB", (int(larg), int(alt)), FUNDO)
    d = ImageDraw.Draw(img)
    y = pad
    for t, est in linhas:
        cor = {"apagado": APAGADO, "destaque": DESTAQUE, "erro": ERRO, "erro_b": ERRO}.get(est, TEXTO)
        fonte = fb if est in ("erro_b", "destaque") else f
        if est == "destaque":
            w = fonte.getlength(t.strip())
            x0 = pad + fonte.getlength("  ")
            d.rounded_rectangle([x0 - 8, y - 4, x0 + w + 8, y + lh - 6], 6, outline=BORDA, width=2)
        d.text((pad, y), t, font=fonte, fill=cor)
        y += lh
    y += 14
    d.line([pad, y, larg - pad, y], fill=(70, 74, 90), width=1)
    y += 14
    d.text((pad, y), nota, font=fr, fill=DESTAQUE)
    y += 26
    for t in rodape:
        d.text((pad, y), t, font=fr, fill=APAGADO)
        y += 26
    img.save(destino)
    print("gerada:", os.path.relpath(destino, RAIZ))


def main():
    # --- Temperatura do BMI088
    log_t = "resultados/IMU_BMI088/resultado_temperatura_8.1.log"
    L = ler(log_t)
    ver, sol = cabecalho(L)
    ident = lambda v: curto(v)
    linhas = contraexemplo(L, {"temp_msb": ident, "temp_lsb": ident, "Temp_uint11": ident,
                               "temperature": ident})
    # o estado 12 repete "temperature" na funcao de teste; mantem so o de updateTemperature
    linhas = [x for k, x in enumerate(linhas)
              if not (x[1] == "apagado" and "test_temperature_calculation" in x[0]
                      and k + 1 < len(linhas) and "temperature =" in linhas[k + 1][0])
              and not (x[1] == "destaque" and "temperature =" in x[0]
                       and "test_temperature_calculation" in linhas[k - 1][0])]
    desenhar(linhas,
             [f"{ver} + {sol} -- log: {log_t}",
              "PX4 main, commit 8be2590c15 -- Issue #26865 (fechada) / PR #27575 (fechado)"],
             "8.625000e+1f = 86,25 °C > 85 °C (limite operacional do datasheet BMI088)",
             os.path.join(RAIZ, "figuras", "ce_temperatura_bmi088.png"))

    # --- GPS uBlox
    log_g = "medianas_logs/GPS_run1.log"
    L = ler(log_g)
    ver, sol = cabecalho(L)
    dump = lambda v: "dump_buffer.len = { .data={...}, .len=" + re.search(r"\.len=(\d+)", v).group(1) + ", ... }"
    linhas = contraexemplo(L, {"test_choice": curto, "input_len": curto, "dump_buffer.len": dump})
    desenhar(linhas,
             [f"{ver} + {sol} -- log: {log_g} (igual em run2 e run3)",
              "PX4 main, commit 8be2590c15 -- Issue #26866 / PR #27576 (aberto)"],
             "harness: 200 - 207 = -7 (int) -> size_t enorme na comparacao; guarda sempre falsa",
             os.path.join(RAIZ, "figuras", "ce_gps_ublox.png"))


if __name__ == "__main__":
    sys.exit(main())
