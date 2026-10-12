# Retorno sobre a reprodução dos 8 candidatos

Marcela — 19/08/2026

Professor, rodei tudo no servidor do laboratório (iartes, ESBMC 8.1.0, CBMC
5.95.1, Ubuntu 24.04) e segue o que deu. Fiz os três ajustes que o senhor
pediu e no caminho apareceram outras seis coisas que acho que valem a sua
opinião — três delas mexem no texto da dissertação, não só no artefato.

---

## 1. Os três ajustes que o senhor pediu

### Candidato 4 (sumd.cpp) — ASan e o timeout

Confirmei os dois pontos. O ASan realmente não detecta nada, e o UBSan pega:

```
replay_sumd.cpp:37:29: runtime error: index 64 out of bounds
                       for type 'unsigned char [64]'
```

Usei `-fsanitize=undefined` sozinho, sem o `array-bounds` que estava no seu
comando, porque no g++ o `array-bounds` não é aceito como opção separada —
já vem dentro do `undefined`. A saída é a mesma que a sua.

O timeout também sumiu com `--unwind 70`:

```
_rxlen = 64
array bounds violated: array upper bound
(signed long int)_rxlen < 64
test_sumd_decode.cpp:162
VERIFICATION FAILED
```

Corrigi a tabela e o comando no guia.

### Candidato 5 (logger.cpp) — o log que faltava

O senhor tinha razão, não havia log nenhum pro logger no pacote. Rodei com
`--max-k-step 320` e gravei. Mas o resultado veio melhor do que eu esperava:

```
VERIFICATION SUCCESSFUL
Solution found by the forward condition; all states are reachable (k = 300)
```

Não é só "não achei violação até o limite". A condição de avanço convergiu, o
que quer dizer que o ESBMC provou que percorreu todos os estados alcançáveis.
Pro candidato 5 a refutação deixa de ser limitada ao bound.

**Isso muda como eu devo escrever o resultado do logger na dissertação?** (Resposta v4: não generalizar. Escrever "sem violação de memória para os comprimentos modelados e qualquer key_len 0–255".)
Hoje o texto trata como refutação normal, sem mencionar a convergência.

### Candidato 7 (bst.cpp) — o replay que não chegava no probe()

Criei o `replay_bst_probe_only.cpp` preservando o arquivo inteiro até o
`main()` (conferi com `diff`, as 145 primeiras linhas são idênticas byte a
byte). Rodou aqui no Linux e deu o mesmo que no Mac do senhor:

```
#3 bst_probe_real ... replay_bst_probe_only.cpp:136
#4 main            ... replay_bst_probe_only.cpp:176
dev_info_reply em [48, 90) — acesso no offset 90
```

O frame é o `bst_probe_real()`, não o `main`. O probe() verbatim executou.

Uma diferença pequena: o senhor registrou `READ of size 37` e aqui deu
`READ of size 34`. Acho que é layout de pilha diferente entre as plataformas,
a violação é a mesma. Deixei isso documentado num LEIA-ME junto dos dois logs.

Mantive o `replay_bst_real.cpp` no pacote em vez de apagar, porque ele já está
no Zenodo e citado no artigo — achei melhor deixar o registro do erro e da
correção do que sumir com o arquivo. **Se o senhor preferir remover, eu tiro.**

---

## 2. O candidato 8 — aqui eu discordo em parte

O senhor escreveu que a asserção é uma tautologia e o texto parou aí. Fui
olhar o harness:

```c
assert(PORT_SIZE <= PORT_SIZE - 1);   // linha 47
```

`PORT_SIZE` é 128, então isso é `128 <= 127`. Não é tautologia, é
**contradição** — falsa sempre. Achei que fosse pior que tautologia (uma
tautologia daria SUCCESSFUL sem provar nada; uma contradição dá FAILED sem
provar nada, e FAILED parece achado).

Só que fui conferir de qual propriedade veio o FAILED e me enganei também:

```
Violated property:
  file test_microstrain_port_v2.cpp line 38 ...
  array bounds violated: array `_port' upper bound
  (signed long int)max_len < 128
```

Veio da verificação automática de limites na linha 38, não da asserção. O
ESBMC para na primeira violação e nunca chegou no `assert` do main.

**Mas o problema de verdade é outro, e é pior:**

```c
const size_t max_len = PORT_SIZE;   // 128, atribuído direto
_port[max_len] = '\0';
```

O harness fixava `max_len = 128` por constante. Não tinha `uart_port`, não
tinha `strnlen`, não tinha `min`. Ou seja, ele provava que "se max_len for
128, _port[128] estoura" — o que é conta, não verificação. A pergunta que
interessa é se existe entrada que leve o driver a max_len = 128, e o harness
assumia a resposta.

Reescrevi (`test_microstrain_port_v3.cpp`): deixei o `uart_port` não
determinístico byte a byte, com o `strnlen` e o `min` calculados como no
driver. Aí o solver decide se o estado é alcançável. E é:

```
uart_port[126] = 4, uart_port[127] = 64, uart_port[128] = 0
max_len = 128
Violated property: assertion max_len < PORT_SIZE
VERIFICATION FAILED
```

O CBMC concorda (2 de 8 propriedades) e o replay novo confirma no ASan:

```
WRITE of size 1
0 bytes after 128-byte region
heap-buffer-overflow
```

Então o candidato 8 sai de "achado duvidoso" para achado com contraexemplo
construído a partir de entrada livre, e com as três ferramentas. Antes de
hoje eu ia tirar ele do conjunto.

---

## 3. Coisas que apareceram e que mexem na dissertação

### 3.1 O `n/a` da Tabela 5.1 não é limitação da ferramenta

Descobri que o ESBMC tem `--multi-property`, que faz ele continuar depois da
primeira violação. Rodei no BMI088:

```
Properties: 120 verified — 118 passed, 2 failed
```

As duas que falham são a temperatura e o FIFO, exatamente os dois candidatos
que a dissertação documenta.

Isso me incomodou porque a Tabela 5.1 põe `n/a` nas VCCs do BMI088 e do GPS, e
o rodapé diz que é porque "o BMC para na primeira violação". Não é bem isso —
é a configuração `--incremental-bmc` que não combina com enumeração completa
de propriedades. Com `--unwind` fixo o ESBMC enumera tudo.

**O senhor acha que vale trocar o `n/a` por 120 na tabela?** Isso também
responderia aquele seu comentário sobre a linha do BMI088 ficar confusa com
"violação" e VAM = 0 na mesma linha. Com a contagem completa dá pra mostrar
118 verificadas e 2 violadas, as duas triadas como não-defeito.

O GPS ainda está rodando enquanto escrevo isso — são 1172 VCCs contra 120 do
BMI088, então pode não fechar em tempo razoável. Se não fechar, aí sim o `n/a`
tem justificativa, só que outra: enumerar tudo é caro demais nesse módulo.

### 3.2 O CBMC do IridiumSBD não dava UNKNOWN pelo motivo que a gente achava

O guia diz que o CBMC dá UNKNOWN "por causa do loop interno do memcpy". Fui
rodar e nem chegava lá:

```
file test_iridiumsbd_write.cpp line 97: symbol 'ssize_t' is unknown
CONVERSION ERROR
```

O CBMC 5.95.1 não conhece `ssize_t`. Com `-D'ssize_t=long'` ele converte e
decide:

```
[iridiumsbd_write.precondition_instance.3] line 130
    memcpy destination region writeable: FAILURE
[main.assertion.1] line 155
    assertion _tx_buf_write_idx <= SATCOM_TX_BUF_LEN: FAILURE
** 2 of 10 failed
```

Mesmas linhas e mesmas propriedades que o senhor obteve no CBMC 6.8. Isso
responde aquilo que o senhor escreveu que não dava pra separar: **foi o
unwind, não a versão.** O 5.95.1 decide igual, desde que receba `--unwind 400`
e saiba o que é `ssize_t`.

### 3.3 Sobre a segunda ressalva

Professor, o senhor escreveu "duas ressalvas que permanecem" e o texto termina
no meio da primeira. Como o PDF tem seis páginas e é o retorno completo, imagino
que a segunda tenha ficado pra depois. **Se lembrar qual era, me diz que eu
verifico.** Deixei anotado no guia como pendência.

---

## 4. O que ficou pendente

- GPS com `--multi-property` (rodando, 1172 VCCs)
- Decidir se reporto os módulos ao PX4 e em que ordem
- O `issue_bst.md` que está no pacote foi escrito quando o replay ainda não
  alcançava o `probe()`. Preciso reescrever com o log novo antes de abrir
  qualquer issue.

## 5. Como estão os oito hoje

| # | Módulo | Situação |
|---|---|---|
| 1 | Common.cpp | achado, três ferramentas |
| 2 | externalChecks.cpp | achado, três ferramentas |
| 3 | ft7_technologies.cpp | achado, três ferramentas — é o que liga direto no SAGRES |
| 4 | sumd.cpp | achado, três ferramentas; severidade prática baixa (escrita intra-objeto) |
| 5 | logger.cpp | sem violação para os comprimentos modelados (k = 300 vale para o harness) |
| 6 | IridiumSBD.cpp | achado, três ferramentas; precisa documentar o `-D'ssize_t=long'` |
| 7 | bst.cpp | achado, agora com o probe() real executando |
| 8 | MicroStrain.cpp | achado, depois do harness v3 |

Sete achados e uma refutação.

Sobre o reporte ao PX4: escrevi o rascunho das sete issues e mandei junto
(`issues_px4_rascunho.md`). Dessa vez separei o achado da proposta de correção
— relato o que o verificador encontrou, digo o que é alcançável no modelo e o
que eu não estabeleci, e pergunto aos mantenedores se o estado é atingível na
prática, em vez de já chegar com patch. Foi a lição que ficou do PR da
temperatura.

Minha sugestão de ordem seria começar pelo `ft7_technologies.cpp`, que é o mais
forte e amarra no SAGRES, e depois `Common.cpp` e `externalChecks.cpp`. Fico no
aguardo do seu feedback.

Todos os logs estão no pacote atualizado, na mesma estrutura de antes.
