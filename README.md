# 🌎 Conversando com o IBGE pela internet: teste de integração

> Hoje vocês vão escrever um programa que **conversa pela internet** com um
> servidor do IBGE (sim, aquele do Censo!), pega informações de lá e
> **verifica** se chegou o que a gente esperava. Isso se chama
> **teste de integração**. 🧪
>
> Siga este tutorial na ordem, **sem pular passo**. Se alguma coisa
> aparecer diferente, levante a mão e chame a professora. 🙋

---

## 🤔 O que é um "teste de integração"?

Na aula passada vocês fizeram um programa que **funcionava sozinho** — só somava números. Não dependia de ninguém pra nada.

Hoje é diferente. O programa de hoje precisa **falar com outro computador** (um servidor do IBGE, do outro lado do Brasil). Quando o seu programa depende de outro sistema, a gente precisa testar essa **conversa** — ou seja, essa **integração**.

Um teste de integração responde a perguntas como:

- ✅ O servidor está respondendo?
- ✅ Ele devolveu o que a gente pediu?
- ✅ O formato da resposta é o que a gente esperava?
- ✅ Os dados fazem sentido? (Tem mesmo 5 regiões? Tem mesmo 27 estados?)

Se qualquer uma dessas coisas falhar, o teste **dispara um erro** e a gente fica sabendo na hora. Vocês vão ver isso funcionando hoje.

---

## 🛠️ PARTE 0 — Antes de começar: instalando tudo

A gente vai precisar de **Python**, **VS Code** e uma biblioteca extra chamada **`requests`**, que serve pra Python conseguir falar pela internet.

> 💡 **Se o computador da escola já tem tudo instalado, pulem direto pra PARTE 1!**

### 🔍 Como saber se já está tudo instalado

**0.1.** Abra o **Prompt de Comando**: aperte a tecla **Windows**, digite `cmd` e aperte **Enter**.

**0.2.** Digite e aperte Enter:

```
python --version
```

- ✅ Mostrou uma versão (tipo `Python 3.12.0`)? Pode pular pra **0.6** (instalar a biblioteca `requests`).
- ❌ Disse `não é reconhecido como comando interno...`? Continuem o tutorial.

**Pra VS Code:** procurem "Visual Studio Code" no menu Iniciar. Se aparecer, está instalado.

### 🐍 Instalando o Python (só se ainda não tem)

**0.3.** Aperte **Windows**, digite **`powershell`** e aperte **Enter**.

**0.4.** Cole este comando e aperte Enter:

```
winget install Python.Python.3.12
```

**0.5.** Autorize a janela de permissão ("Sim") e espere o `Successfully installed`. ⏱️ Demora uns 2-3 minutos.

> ⚠️ **Importante:** depois de instalar, **feche o PowerShell** e abra um **novo** Prompt de Comando. Terminais não enxergam programas que foram instalados enquanto estavam abertos.

### 📦 Instalando a biblioteca `requests`

A biblioteca `requests` é o que vai permitir que o Python **converse com a internet**. Ela não vem com o Python por padrão, então a gente instala separado.

**0.6.** Em um Prompt de Comando **novo** (depois de instalar o Python), cole:

```
pip install requests
```

> 💡 `pip` é o "instalador de bibliotecas" do Python. Funciona parecido com o `winget` mas só pra coisas do Python.

**0.7.** Vai aparecer uma barra de progresso e depois algo tipo:

```
Successfully installed requests-2.32.0 ...
```

✅ **Apareceu `Successfully installed`?** Pode seguir pra PARTE 1!

### 💻 Instalando o Visual Studio Code (só se ainda não tem)

**0.8.** No PowerShell:

```
winget install Microsoft.VisualStudioCode
```

Autorize e espere terminar.

### 🆘 Problemas comuns na instalação

| Problema | Solução |
|---|---|
| `pip não é reconhecido` | O Python não foi instalado direito ou vocês esqueceram de fechar e abrir o terminal. |
| `pip install requests` dá erro de rede | Internet da escola pode estar bloqueando. Chame a professora. |
| `python não é reconhecido` mesmo depois de instalar | Esqueceram de **fechar e abrir o terminal**. |
| Pediu senha de administrador | Chame a professora. |

---

## 📥 PARTE 1 — Pegar o material no Google Classroom

**1.1.** Abra o **Google Classroom** no navegador.

**1.2.** Entre na turma e procure a atividade de hoje.

**1.3.** Clique no arquivo `.zip` anexado pra baixar.

**1.4.** O arquivo vai cair em **Downloads**.

---

## 📂 PARTE 2 — Preparar a pasta no computador

**2.1.** Abra o **Explorador de Arquivos** (pastinha amarela na barra de baixo).

**2.2.** Vá em **Downloads**.

**2.3.** Encontre o `.zip` baixado.

**2.4.** Clique com o **botão direito** → **"Extrair Tudo..."** → **"Extrair"**.

**2.5.** Entre na pasta extraída. Você deve ver **diretamente estes 2 arquivos**:

```
📄 README.md
📄 teste_api.py
```

> ⚠️ **Se aparecer uma pasta dentro da pasta** (em vez dos arquivos), entre nessa pasta de dentro.

---

## 💻 PARTE 3 — Abrir a pasta no VS Code

**3.1.** Abra o **Visual Studio Code**.

**3.2.** Menu **"File"** → **"Open Folder..."**.

**3.3.** Navegue até **Downloads** → clique uma vez na pasta que você extraiu.

**3.4.** Clique em **"Selecionar Pasta"**.

**3.5.** Se perguntar *"Do you trust the authors..."*, clique em **"Yes, I trust the authors"**.

**3.6.** Você deve ver no canto **esquerdo**:

```
📁 PASTA-DA-AULA
   📄 README.md
   📄 teste_api.py
```

---

## ⌨️ PARTE 4 — Abrir o terminal dentro do VS Code

**4.1.** Menu **"Terminal"** → **"New Terminal"**.

> 💡 *Atalho: `Ctrl` + `'`*

**4.2.** Um painel preto abre embaixo, mostrando o caminho da pasta. **Se aparecer o nome da pasta no final, está certo!** ✅

---

## 👀 PARTE 5 — Olhar o código com calma

**5.1.** Na lista da esquerda, clique em **`teste_api.py`**.

**5.2.** Reparem nas coisas novas (que a aula passada não tinha):

- 🆕 `import requests` → a gente está usando uma biblioteca pra falar com a internet
- 🆕 `requests.get(url)` → essa linha **faz a chamada na internet** e espera a resposta
- 🆕 `resposta.status_code` → um número que o servidor devolve. `200` = "deu tudo certo"
- 🆕 `resposta.json()` → transforma a resposta (que vem em texto) em uma lista do Python
- 🆕 `assert ...` → essa palavrinha é **a alma do teste de integração**

> 🔑 **`assert` significa "afirme que isto é verdade"**. Se for verdade, o programa continua tranquilo. Se não for, o programa **PARA na hora** e mostra o erro.
>
> Exemplo:
> ```python
> assert len(regioes) == 5, "Esperava 5 regiões!"
> ```
> Quer dizer: "afirme que recebemos 5 regiões. Se não recebeu, grita pra mim."

**5.3.** Reparem que o programa faz **3 testes diferentes**:

1. Buscar as **5 regiões** do Brasil (resposta bem pequena)
2. Buscar os **27 estados** (resposta média)
3. Buscar **todos os ~5570 municípios** (resposta GRANDE)

A gente vai descobrir uma coisa interessante sobre essa diferença de tamanho. 👀

---

## 🚀 PARTE 6 — Rodar o programa

**6.1.** Voltem pro terminal.

**6.2.** ✅ **Confirmação rápida:** digite `dir` e aperte Enter. Os arquivos `README.md` e `teste_api.py` precisam aparecer.

- ❌ Não apareceu? Está na pasta errada — **chame a professora**. 🙋

**6.3.** Agora rode:

```
python teste_api.py
```

**6.4.** Vai aparecer várias linhas no terminal. No final, algo parecido com:

```
=======================================================
TESTE: Buscar as 5 regioes do Brasil
URL : https://servicodados.ibge.gov.br/api/v1/localidades/regioes
=======================================================
Tempo gasto..: 187.45 ms
Status code..: 200
Quantidade...: 5 regioes
Nomes........: ['Norte', 'Nordeste', 'Sudeste', 'Sul', 'Centro-Oeste']
Todos os asserts passaram! :)

[... e por aí vai, com os outros 2 testes ...]

=======================================================
RESUMO DOS TEMPOS
=======================================================
  5 regioes.........:   187.45 ms
  27 estados........:   234.12 ms
  5570 municipios...:  1845.30 ms

Buscar municipios foi 9.8x mais lento que buscar regioes.

PARABENS! Todos os testes de integracao passaram. :)
```

**6.5.** **Anotem os 3 tempos** que apareceram no SEU computador (no caderno ou num papel).

**6.6.** Rodem o programa **mais 2 vezes** (seta ↑ no terminal e Enter) e anotem os tempos das outras tentativas.

> 💡 Os tempos variam **muito** entre uma execução e outra. A gente vai falar sobre isso já já.

---

## 🤔 PARTE 7 — Mas e os asserts? Como saber que estão funcionando?

Apareceu "Todos os asserts passaram!" — beleza, mas como provar que eles **realmente verificam** alguma coisa? Vamos quebrar o teste **de propósito** pra ver.

**7.1.** Abram o `teste_api.py` no VS Code.

**7.2.** Encontrem esta linha (ali no Teste 1):

```python
assert len(regioes) == 5, f"Esperava 5 regioes, recebeu {len(regioes)}"
```

**7.3.** Mudem o `5` pra `7` (uma mentira de propósito):

```python
assert len(regioes) == 7, f"Esperava 5 regioes, recebeu {len(regioes)}"
```

**7.4.** **Salvem** (`Ctrl + S`).

**7.5.** Rodem de novo:

```
python teste_api.py
```

**7.6.** O programa **vai quebrar** mostrando algo tipo:

```
AssertionError: Esperava 5 regioes, recebeu 5
```

🎉 **Isso é ótimo!** Quer dizer que o teste **está funcionando**. Ele percebeu que a realidade (5 regiões) não bateu com o que a gente afirmou (7).

**7.7.** Voltem o `7` pro `5` e salvem de novo. Confirmem rodando — tem que voltar a passar tudo.

> 🔑 **Lição:** um teste só vale se ele **consegue falhar** quando alguma coisa está errada. Um teste que sempre passa, não importa o que aconteça, não testa nada.

---

## ⏱️ PARTE 8 — Por que o tempo varia tanto?

Olhem os tempos que vocês anotaram. Provavelmente algo assim:

| Endpoint | Tentativa 1 | Tentativa 2 | Tentativa 3 |
|---|---|---|---|
| 5 regiões | ____ ms | ____ ms | ____ ms |
| 27 estados | ____ ms | ____ ms | ____ ms |
| ~5570 municípios | ____ ms | ____ ms | ____ ms |

### Perguntas pra discutir

1. **Qual foi mais lento, sempre?** (Provavelmente municípios.)
2. **Por quê?** O Python ficou "mais lento" no terceiro caso? 🤔
3. **Os tempos variaram bastante entre as 3 tentativas?**

### A resposta

Na aula passada, o computador estava fazendo a conta **sozinho**. Tudo o que importava era o quanto a linguagem era rápida.

Hoje é **completamente diferente**. Olhem o que acontece quando vocês rodam `python teste_api.py`:

```
[seu computador]  →  rede da escola  →  internet do Brasil  →  
                                              ↓
                                       servidor do IBGE
                                              ↓
                                      ele procura os dados
                                              ↓
[seu computador]  ←  rede da escola  ←  internet do Brasil  ←
```

O Python só fica **esperando** essa volta toda. Praticamente o tempo todo, o **gargalo não é a linguagem** — é a internet e o tamanho da resposta.

**Por isso:**
- 🟢 **5 regiões** = resposta com pouca coisa = volta rapidinho
- 🟡 **27 estados** = resposta um pouquinho maior = um pouco mais lento
- 🔴 **5570 municípios** = resposta ENORME (alguns megabytes!) = bem mais lento

### Por que os tempos variam entre tentativas?

Várias coisas mudam de uma execução pra outra:

- 🌐 A internet pode estar mais ou menos congestionada
- 🖥️ O servidor do IBGE pode estar atendendo muita gente naquele segundo
- 💾 Pode ser que da segunda vez a resposta esteja "cacheada" (guardada em algum lugar do caminho) e venha mais rápido

> 🔑 **Lição:** quando seu programa depende de **outras coisas pela internet**, o tempo não é só "culpa" da linguagem. Tem muita coisa fora do seu controle.

---

## 💡 PARTE 9 — Recapitulando: o que é um teste de integração?

Hoje vocês não fizeram só "um programa que chama uma API". Vocês fizeram um **teste de integração** porque:

| Ingrediente | Como apareceu no nosso programa |
|---|---|
| Conversa com um sistema externo | `requests.get(...)` chama o servidor do IBGE |
| Verifica o que voltou | `assert resposta.status_code == 200` |
| Verifica o formato | `assert len(regioes) == 5` |
| Verifica que os dados fazem sentido | `assert "Sudeste" in nomes_regioes` |
| Falha de forma clara quando algo dá errado | Vocês viram isso na PARTE 7 |

Empresas grandes (Nubank, iFood, Magalu...) rodam **milhares** desses testes automaticamente, várias vezes por dia, pra garantir que os sistemas continuam conversando direito uns com os outros.

---

## 🎯 PARTE 10 — Desafios extras (se sobrar tempo)

### 🎯 Desafio 1 — Buscar só os estados do Nordeste

Olhem a documentação:
👉 https://servicodados.ibge.gov.br/api/docs/localidades

Existe um endpoint pra buscar só os estados de uma região específica. Tentem descobrir qual é, e adicionem um quarto teste no programa que:

1. Busca só os estados do Nordeste
2. Verifica com `assert` que recebeu **exatamente 9 estados**
3. Verifica que **"Pernambuco"** está na lista

### 🎯 Desafio 2 — Quebrar mais asserts pra ver o que acontece

Inventem 3 mentiras diferentes nos asserts (tipo: "esperava 100 municípios", "Sudeste virou Sudoeste") e rodem cada uma. Anotem que mensagem de erro cada uma gera.

> 💡 Saber **ler mensagens de erro** é uma das habilidades mais úteis que vocês podem desenvolver em programação.

### 🎯 Desafio 3 — Testar uma URL que não existe

Mudem uma das URLs pra algo errado de propósito, tipo:

```python
url = f"{URL_BASE}/cidades-fantasmas"
```

O que acontece? O `status_code` vira `200` ou outro número? Vejam qual.

> 🔍 Códigos que vocês podem encontrar: `200` (OK), `404` (não existe), `500` (erro do servidor), `503` (servidor fora do ar)

---

## 🆘 Problemas comuns

| Problema | Solução |
|---|---|
| `ModuleNotFoundError: No module named 'requests'` | Esqueceram de rodar `pip install requests`. Voltem ao passo 0.6. |
| `python não é reconhecido` | Tentem `python3 teste_api.py`. |
| `ConnectionError` ou `Timeout` | Internet da escola está com problema, ou o IBGE está fora do ar. Tentem de novo daqui a pouco. |
| Um `assert` falhou que não era pra falhar | Pode ser que o IBGE mudou alguma coisa (raro, mas acontece). **Chame a professora.** |
| `can't open file 'teste_api.py'` | O terminal está na pasta errada. Digite `dir` pra conferir. |
| Tempos super diferentes do que aparece nos exemplos | Normal! Depende muito da internet da escola. O importante é a **proporção** entre os 3 testes no SEU computador. |

---

## ✅ Resumo

Hoje vocês:

- 🌎 Fizeram um programa que **conversa com um servidor pela internet**
- ✅ Aprenderam a **verificar** se a resposta veio certinha (com `assert`)
- 💥 Quebraram um assert de propósito pra ter certeza que o teste funciona
- ⏱️ Mediram os tempos e descobriram que **o tamanho da resposta importa MUITO**
- 🧠 Entenderam o conceito de **teste de integração**: testar a conversa entre seu programa e outro sistema

Bom trabalho! 🚀
