#!/usr/bin/env bash
# ESBMC 8.5 com o solver Boolector, o mesmo das outras verificações da dissertação.
# Sem isto, o rodar_tudo.sh usa o esbmc do PATH (na máquina oficial, o 8.2) e o solver
# padrão (Z3), com o qual o teste do sumd estoura o tempo-limite de 1 h (19/10/2026).
#   ESBMC=$PWD/esbmc_boolector.sh ./rodar_tudo.sh
E="${ESBMC85:-$(find "$HOME/esbmc-8.5" -name esbmc -type f 2>/dev/null | head -1)}"
[ -x "$E" ] || { echo "ESBMC 8.5 não encontrado (defina ESBMC85=/caminho/esbmc)" >&2; exit 127; }
case " $* " in
  *" --version "*) exec "$E" "$@" ;;
  *) exec "$E" --default-solver boolector "$@" ;;
esac
