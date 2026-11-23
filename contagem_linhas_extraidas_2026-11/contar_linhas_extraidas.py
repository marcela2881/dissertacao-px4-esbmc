#!/usr/bin/env python3
"""Conta, para cada harness da primeira campanha, quantas linhas de codigo
coincidem com linhas do codigo-fonte do PX4 (src/), e quantas sao codigo de modelo.

Criterio (declarado no texto da dissertacao):
  - So entram linhas de codigo: sem comentarios, sem linhas em branco.
  - Linhas triviais (menos de 12 caracteres sem espacos, como "{", "}", "return;",
    "break;") ficam FORA do numerador e do denominador, e sao contadas a parte.
  - Uma linha do harness e "extraida do PX4" se, normalizada (espacos removidos),
    for identica a alguma linha nao trivial de algum arquivo de src/.
  - Para evitar coincidencias (linhas genericas que existem em qualquer codigo C),
    so contam arquivos de src/ com pelo menos 5 linhas coincidentes com o harness,
    e bibliotecas de terceiros (submodulos) ficam fora do indice.
  - O numero e um LIMITE INFERIOR: linhas adaptadas (tipo, nome ou constante
    alterados) contam como modelo.
Uso:  python3 contar_linhas_extraidas.py CAMINHO_DO_PX4 [CAMINHO_DOS_HARNESSES]
"""
import os, re, sys, glob, collections

px4 = sys.argv[1]
hdir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'resultados')
MIN = 12
MIN_ARQ = 5
TERCEIROS = ('lib/cdrstream', 'zenoh', 'libtommath', 'monocypher', 'rl_tools', 'cyclonedds', 'lib/crypto', 'cyphal', 'iq-module', 'public_regulated', 'legacy_data')

def strip_comments(txt):
    txt = re.sub(r'/\*.*?\*/', lambda m: '\n' * m.group(0).count('\n'), txt, flags=re.S)
    return [re.sub(r'//.*', '', l) for l in txt.split('\n')]

def norm(l):
    return re.sub(r'\s+', '', l)

src = os.path.join(px4, 'src')
if not os.path.isdir(src):
    sys.exit('Nao achei %s' % src)
index = collections.defaultdict(set)   # linha normalizada -> arquivos
nfiles = 0
for root, _, files in os.walk(src):
    for f in files:
        if f.endswith(('.cpp', '.hpp', '.h', '.c', '.cc')):
            p = os.path.join(root, f)
            try:
                txt = open(p, encoding='utf-8', errors='replace').read()
            except OSError:
                continue
            nfiles += 1
            rel = os.path.relpath(p, src)
            if any(t in rel for t in TERCEIROS):
                continue
            for l in strip_comments(txt):
                n = norm(l)
                if len(n) >= MIN:
                    index[n].add(rel)

print('PX4: %s | arquivos indexados: %d' % (px4, nfiles))
print('%-28s %7s %9s %9s %9s  %s' % ('harness', 'codigo', 'triviais', 'do PX4', 'modelo', 'arquivos que mais casam'))
tot = [0, 0, 0, 0]
for h in sorted(glob.glob(os.path.join(hdir, '*', 'test_*.cpp'))):
    name = os.path.basename(h)
    if 'prop_limpa' in name:
        continue
    lines = [norm(l) for l in strip_comments(open(h, encoding='utf-8', errors='replace').read())]
    lines = [l for l in lines if l]
    triv = [l for l in lines if len(l) < MIN]
    nt = [l for l in lines if len(l) >= MIN]
    cnt = collections.Counter()
    for l in nt:
        for f in index.get(l, ()):
            cnt[f] += 1
    apoio = {f for f, c in cnt.items() if c >= MIN_ARQ}
    hit = [l for l in nt if index.get(l, set()) & apoio]
    top = ', '.join('%s(%d)' % (f, c) for f, c in cnt.most_common(3) if c >= MIN_ARQ)
    print('%-28s %7d %9d %9d %9d  %s' % (name, len(lines), len(triv), len(hit), len(nt) - len(hit), top))
    for i, v in enumerate((len(lines), len(triv), len(hit), len(nt) - len(hit))):
        tot[i] += v
print('%-28s %7d %9d %9d %9d' % ('TOTAL', *tot))
