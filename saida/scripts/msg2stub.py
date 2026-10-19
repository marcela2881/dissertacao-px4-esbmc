#!/usr/bin/env python3
"""
Gera os headers de tópicos uORB usados pelos stubs (saida/stubs/uORB/topics/<nome>.h)
a partir das definições ORIGINAIS do PX4 (msg/*.msg e msg/versioned/*.msg).

Só os campos e as constantes; sem serialização. Mesmo layout de nomes do gerador do
PX4 (struct <nome>_s, constantes como static constexpr dentro da struct).

Uso: msg2stub.py <pasta msg do PX4> <pasta de saída> NomeMsg [NomeMsg ...]
  ex.: msg2stub.py px4/msg stubs/uORB/topics ActuatorArmed ActuatorOutputs
"""
import os
import re
import sys

TIPOS = {
    'bool': 'bool', 'char': 'char', 'byte': 'uint8_t',
    'int8': 'int8_t', 'int16': 'int16_t', 'int32': 'int32_t', 'int64': 'int64_t',
    'uint8': 'uint8_t', 'uint16': 'uint16_t', 'uint32': 'uint32_t', 'uint64': 'uint64_t',
    'float32': 'float', 'float64': 'double',
}


def snake(nome):
    return re.sub(r'(?<!^)(?=[A-Z])', '_', nome).lower()


def acha(pasta, nome):
    for sub in ('', 'versioned'):
        p = os.path.join(pasta, sub, nome + '.msg')
        if os.path.isfile(p):
            return p
    raise SystemExit('nao achei %s.msg em %s' % (nome, pasta))


def gera(pasta, nome, saida):
    campos, consts, incl = [], [], []
    for linha in open(acha(pasta, nome)):
        linha = linha.split('#', 1)[0].strip()
        if not linha:
            continue
        m = re.match(r'^(\w+)(\[(\d+)\])?\s+(\w+)\s*(=\s*(.+))?$', linha)
        if not m:
            continue
        tipo, _, n, campo, _, valor = m.groups()
        if tipo in TIPOS:
            ctipo = TIPOS[tipo]
        else:  # mensagem aninhada
            ctipo = snake(tipo) + '_s'
            incl.append(snake(tipo))
        if valor is not None:
            consts.append('\tstatic constexpr %s %s = %s;' % (ctipo, campo, valor.strip()))
        else:
            campos.append('\t%s %s%s;' % (ctipo, campo, '[%s]' % n if n else ''))
    arq = snake(nome)
    with open(os.path.join(saida, arq + '.h'), 'w') as f:
        f.write('/* GERADO por saida/scripts/msg2stub.py a partir de msg/%s.msg (PX4 8be2590). */\n' % nome)
        f.write('#pragma once\n#include <cstdint>\n')
        for i in incl:
            f.write('#include "%s.h"\n' % i)
        f.write('struct %s_s {\n%s\n%s\n};\n' % (arq, '\n'.join(campos), '\n'.join(consts)))
    print('  %s.h (%d campos, %d constantes)' % (arq, len(campos), len(consts)))


if __name__ == '__main__':
    if len(sys.argv) < 4:
        raise SystemExit(__doc__)
    for n in sys.argv[3:]:
        gera(sys.argv[1], n, sys.argv[2])
