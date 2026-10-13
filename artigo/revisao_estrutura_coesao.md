# Revisão de estrutura, coesão e conteúdo do artigo (versão em português)

Esta revisão segue as diretrizes do PPGEE para uso de LLMs (Cordeiro, Dantas e Lima Filho, 03/08/2026), itens 2 e 3 do "pode e deve": revisão de estrutura e coesão e revisão crítica do conteúdo. **Ela não reescreve o texto.** Aponta problemas e sugere como resolvê-los. A reescrita é sua.

Arquivo revisado: `artigo/artigo_pt.tex`, uma tradução fiel do original em inglês. Os marcadores `\entra{}` e `\corr{}` foram removidos e o conteúdo deles foi mantido.

---

## 1. Diagnóstico geral

O problema central de coesão é que **o texto foi corrigido por acréscimo**. A cada revisão ou resposta dos mantenedores, entrou uma ressalva nova (marcadores `\corr`), mas a narrativa original não foi refeita. Por isso:

- o artigo começa prometendo "defeitos latentes" e "evidência de ausência" e termina dizendo que nenhum dos três achados é um defeito confirmado;
- quase toda frase longa carrega uma ressalva entre parênteses (versão da ferramenta, data do harness, número do PR, quem fechou a issue);
- a mesma informação se repete em 4 ou 5 lugares (o caso do GPS aparece no resumo, na introdução, na metodologia, na avaliação e na conclusão, sempre com detalhes diferentes).

**Pergunta para você responder antes de reescrever:** qual é a tese do artigo hoje, depois do que os mantenedores disseram? Pelo que os resultados sustentam, parecem ser duas:
1. BMC com harnesses escritos à mão é **viável** por módulo no PX4 (QP1 e QP3);
2. contraexemplos de harness **precisam ser confrontados com o código de produção e com a comunidade**, e o artigo documenta três casos disso.

Se a tese for essa, a introdução, as contribuições e as QPs devem ser construídas em torno dela, e não em torno de "encontrar defeitos".

---

## 2. Comentários por seção

### Resumo
- Está longo e mistura detalhes de implementação (Z3 sem veredito, harness semeado do CBMC, *unwind* reduzido) que pertencem à Seção V.
- A ordem lógica esperada (contexto → lacuna → objetivo → método → resultados → conclusão) está quebrada: a lacuna aparece, depois o método, depois dois "resultados", depois uma lição ("harnesses expõem estados...") e por último um detalhe de ferramenta.
- A frase do GPS tem quatro orações encadeadas ("underflow ... propagando ... proposto ... o mantenedor mostrou"). Quebre em frases curtas.
- "≈94 s no total, sem a execução do GPS": o valor também exclui o BMI088 (93,8 s = soma dos dez módulos sem violação). Ajuste o texto ou o número.

### I. Introdução
- **1º parágrafo:** a última frase junta três ideias com ponto e vírgula (testes não acham, testes não provam ausência, o verificador precisa separar artefatos). Cada uma merece uma frase.
- **2º parágrafo:** "Uma extensão da BMC original pode fechar essa lacuna" não se liga à frase anterior. Qual extensão? Por que o ESBMC e não o CBMC? A justificativa da escolha da ferramenta está faltando.
- **2º parágrafo:** objetivo e QPs estão espremidos em uma frase. Considere separar: lacuna → objetivo → QPs (em lista ou destacadas).
- **Falta uma hipótese explícita.** As diretrizes destacam a "criação de hipótese" como parte do método científico que é sua. Hoje o artigo tem QPs, mas nenhuma hipótese que os resultados confirmem ou refutem.
- **Contribuições (iii) e (iv):** estão muito longas e cheias de números de PR e issue. Uma contribuição deve dizer *o que* o leitor ganha. O histórico de PRs vai para a Seção V.
- **Contribuição (iii):** apresentar como contribuição uma violação que o próprio texto mostra ser inalcançável enfraquece o artigo. Talvez a contribuição seja "a análise de três contraexemplos e o que eles mostram sobre a validade de harnesses".
- "Até onde sabemos, esta é uma das primeiras aplicações...": uma afirmação de originalidade exige revisão bibliográfica que a sustente (ver Seção III).

### II. Fundamentação Teórica
- Está coesa e no tamanho certo.
- **Possível imprecisão técnica** (verifique na literatura, não confie em mim): a frase "a BMC é *correta* e *completa* até o limite k" tem sentido específico. Confira em Biere et al. (1999) e Clarke et al. qual formulação usar. Com `--no-unwinding-assertions` ligado (Seção V-A), a completude até k deixa de valer para laços maiores que k, o que contradiz esta frase. A Seção VI reconhece isso, mas a Seção II não avisa.

### III. Trabalhos Relacionados
- **É a seção mais fraca para um artigo científico.** Tem dois parágrafos e cerca de seis trabalhos. Não há comparação sistemática (por exemplo, uma tabela com alvo, técnica, nível de código e ferramenta).
- Faltam categorias que um parecerista vai perguntar: verificação de drivers e firmware embarcado com BMC (fora de UAV), *fuzzing* de PX4/ArduPilot, análise estática de autopilotos, verificação de parsers MAVLink.
- **Isso é trabalho seu** (diretriz 3): buscar em IEEE Xplore, ACM DL, Springer e Scopus, ler e decidir o que entra. Eu não vou sugerir referências.
- **Confira cada referência atual.** Para cada entrada do `references.bib` (Wanner2024, Khatiri2024, Pollien2023, Bernardeschi2023, Sousa2026, Luckcuck2019, Siddiqi2022, Menezes2024, Monteiro2022, Beyer2022, Godefroid2005, Kroening2014, Bosch2022…): verifique se existe, se você leu e se diz o que o texto afirma. O protocolo exige o link de todas as referências.

### IV. Metodologia
- **O parágrafo "Na prática:..."** repete a lista das oito etapas com detalhes de execução e remissões para seções futuras. Isso quebra o fluxo. A lista de etapas deveria vir uma vez, e o "como" de cada uma nas subseções.
- **O exemplo do GPS** no fim da introdução da seção antecipa o resultado ("mostram que o estado violador não ocorre em produção") antes de o leitor saber o que é o harness. Na Metodologia, descreva o método; o resultado fica para a Seção V.
- **Fig. 2:** o fluxo tem uma saída "Bug confirmado" que nenhum caso alcançou. Não é errado, mas a legenda precisa justificar.
- **Etapa (viii)** (outro piloto automático + CI/CD): no texto ela se resume a uma issue no ArduPilot e a um workflow em um fork. É honesto chamar isso de "etapa da metodologia"? Um parecerista pode questionar.
- **IV-A:** o parágrafo longo sobre o harness do GPS (assume, tautologia da asserção, valores do contraexemplo) é detalhe de implementação. A informação de que **a asserção final é uma tautologia** é importante e está escondida entre parênteses. Ela mostra que a propriedade φ_gps da Eq. 3 não é a que detectou a violação. Isso deveria estar em destaque, provavelmente em Ameaças à Validade.
- **"Caracterizamos a cobertura por categoria de entrada (Seção V-B)":** a Seção V-B não apresenta nenhuma métrica de cobertura, só uma lista de classes. A promessa não é cumprida.

### V. Avaliação Experimental
- **V-A, o ponto mais sério para a validade:** "os harnesses são cópias de trechos, não compilados contra o PX4, e algumas constantes diferem (200 vs. 79 bytes)". Isso afeta a afirmação do título ("Verificação formal de módulos críticos do PX4"). O que foi verificado são modelos escritos à mão, inspirados no PX4. O título, o resumo e as contribuições precisam ser coerentes com isso. Essa informação deveria estar em Ameaças à Validade e não escondida na configuração experimental.
- **Tabela II:**
  - a linha "Total" diz "2 viol.", mas a coluna VAM soma 1; o leitor fica confuso. O BMI088 está como "viol." com 0 VAM.
  - **Conferi os tempos com os logs de `medianas_logs/`.** As medianas batem com a tabela (Barômetro 0,368; Bateria 6,738; EKF2 12,213; Expo 4,870; MAVLink 49,176; Magnetômetro 6,630; Navigator 0,100; PWM 0,161; RC 9,468; VTOL 3,966; GPS 64,857; IMU 0,122 s). A soma exata dos dez é 93,7 s; o 93,8 vem da soma dos valores já arredondados. Os 158,7 s "com GPS" também incluem o IMU (0,1 s), o que a nota não diz.
- **QP1:** "todo harness cronometrado é verificado em menos de noventa segundos". O limite de 90 s foi definido antes ou depois dos resultados? Se não há critério prévio, a resposta é descritiva, não um teste.
- **QP2:** a pergunta foi formulada de modo que a resposta seja sempre "sim" ("violações alcançáveis no modelo que os testes não exercitam"). Como a violação não é alcançável em produção, a resposta "afirmativa" soa forçada. Um parecerista vai notar. Considere reformular a QP2 ou a resposta.
- **QP3:** a resposta ("três configurações bastam") está na Discussão, longe dos dados que a sustentam (V-A). Ela poderia vir logo após a configuração experimental.
- **Parágrafo do CBMC (V-B):** é a frase mais difícil do artigo. Tem harness original, harness posterior, data, pré-condição, diferença de −5 para −10, três unwinds e timeout de 30 min. Considere uma pequena tabela (ferramenta × harness × unwind × resultado).
- **"Verificação da correção":** o próprio parágrafo conclui que a comparação não isola o efeito da correção e que não há log do ESBMC. Se não há evidência, o parágrafo enfraquece a contribuição (ii) ("validação de correções"). Decida se fica, sai ou vai para Ameaças.
- **Estudos de caso:** a ordem IMU → GPS está boa, mas o GPS é o caso principal e aparece por último, depois de uma QP2 respondida no meio do estudo de caso. Avalie a ordem.

### VI. Ameaças à Validade
- Boa estrutura (interna, externa, construto, stubs).
- **Falta:** (a) harness não compilado contra o PX4 e com constantes diferentes; (b) asserção do GPS tautológica; (c) execuções originais com outra versão do ESBMC (7.10) e a troca para 8.1.0; (d) os autores que construíram os harnesses também fizeram a triagem (viés do pesquisador).
- "Seus logs não estão no artefato" (MAVLink e EKF2): um parecerista pode pedir esses logs.

### VII. Conclusão
- Repete o resumo quase literalmente. A conclusão deveria responder: o que aprendemos? A hipótese se confirmou? Quais são as lições para quem for aplicar BMC em firmware?
- Os trabalhos futuros estão em uma linha só. Ligue cada um a uma limitação da Seção VI (por exemplo, k-indução para o problema do limite k).

### Agradecimentos e declaração de IA
- O texto atual diz que a IA foi usada **"para a redação"** (*drafting*) do texto. **Isso contraria a diretriz 1** ("não delegue a escrita a LLMs"). Ao reescrever o artigo com suas palavras, atualize a declaração para descrever o uso real (por exemplo: análise de código, revisão gramatical, revisão de estrutura e coesão, tradução).
- O protocolo pede uma **seção** de declaração de uso de IA, e não uma frase nos agradecimentos. Verifique o formato exigido pelo evento.
- Lembre-se do protocolo: **laudos de duas ou mais ferramentas com menos de 20% de conteúdo de IA**. Um texto reescrito por você, e não por mim, é o caminho para isso.

---

## 3. Inconsistências que conferi com o repositório

| Ponto | No artigo | No repositório | Ação |
|---|---|---|---|
| Tempos medianos (12 módulos) | Tabela II | `medianas_logs/`: todos batem | nenhuma |
| Total dos 10 módulos | 93,8 s | 93,7 s (soma exata) | escolher um critério e declarar |
| "158,7 s com GPS" | nota da Tabela II | inclui também o IMU (0,1 s) | ajustar a nota |
| "≈94 s sem a execução do GPS" | resumo | também exclui o IMU | ajustar o texto |
| Versão e solver | ESBMC 8.1.0 + Boolector 3.2.4 | confirmado em `LOGS.md` | nenhuma |
| README do artefato | citado como guia de reprodução | o `README.md` do `main` foi substituído por um tutorial sobre o IBGE no commit `b535829` | **restaurar antes de submeter** (o artigo aponta para esse repositório e para o DOI do Zenodo) |

---

## 4. Ordem de trabalho sugerida

1. **Definir a tese** (seção 1 deste documento) e escrever, com suas palavras, uma frase-tese, a hipótese e as QPs revisadas.
2. **Revisão bibliográfica** (sua): ler e fichar os trabalhos; reescrever a Seção III.
3. **Reescrever a Introdução** a partir da tese (contexto → lacuna → hipótese/objetivo → QPs → contribuições curtas).
4. **Reorganizar Metodologia e Avaliação**: tirar resultados da Metodologia; mover limitações para Ameaças; decidir o destino de "Verificação da correção".
5. **Conclusão e Resumo por último**, quando o corpo estiver estável.
6. A cada seção reescrita, me mande o texto para revisão de coesão e gramática.
7. Só então, a versão em inglês.

Quando receber o PDF da dissertação do Lennon, eu acrescento aqui a comparação estrutural (ordem dos capítulos e função de cada seção).
