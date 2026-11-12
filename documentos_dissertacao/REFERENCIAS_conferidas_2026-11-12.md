# Referências conferidas na fonte em 2026-11-12

## Datasheet do BMI088 (Bosch Sensortec)

Fonte: PDF oficial em
`https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf`,
baixado em 2026-11-12 (62 páginas).

- Identificação na capa: "BMI088: Data Sheet", "Document revision 1.9",
  "Document release date January 2024", "Document number BST-BMI088-DS000-19".
- Faixa operacional: Seção 1 "Specification", 1.1 "Electrical Specifications",
  "Table 1: Electrical parameter specification", **p. 7**: linha
  "Operating Temperature | TA | Min -40 | Max +85 | °C".
- Sensor de temperatura: Seção 1.4 "Temperature Sensor Specifications",
  "Table 6: Temperature sensor specifications", **p. 11**: faixa de medição
  TS de -104 a 150 °C; inclinação dTS 0.125 K/LSB; erro de offset OTS ±1 K a 25 °C.
- Conversão: Seção 5.3.7 "Register 0x22 – 0x23: Temperature sensor data",
  **p. 28**: valor de 11 bits em complemento de dois;
  `Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32)`; se maior que 1023, subtrai
  2048; `Temperature = Temp_int11 * 0,125°C/LSB + 23°C`. A tabela de exemplos
  vai de -40 °C (`0xC1 0x00`) a 85 °C (`0x3E 0x00`), e `0x80` é "Invalid".
  "The temperature sensor data is updated every 1.28 s."

Atenção: a chave usada no texto é `Bosch2021`, mas o documento é de janeiro de
2024. Conferir o ano na entrada do `referencias.bib`.

## ICMJE

Fonte: `https://www.icmje.org/recommendations/` e
`https://www.icmje.org/icmje-recommendations.pdf` (21 páginas), em 2026-11-12.

- Título: "Recommendations for the Conduct, Reporting, Editing, and Publication
  of Scholarly Work in Medical Journals".
- Versão vigente: "**Updated January 2026**" (na página e na primeira página do PDF).
- Orientação sobre IA: Seção II.A.4, "Artificial Intelligence (AI)–Assisted
  Technology".
- O texto antigo da Declaração dizia "ICMJE (2025)"; não foi localizada uma
  versão de 2025 nesta conferência. A entrada preparada cita a de janeiro de 2026.

## WAME

Dados obtidos por busca na web (não pela página da WAME aberta diretamente):
Zielinski C, Winker MA, Aggarwal R, et al., "Chatbots, Generative AI, and
Scholarly Manuscripts: WAME Recommendations on Chatbots and Generative
Artificial Intelligence in Relation to Scholarly Publications", revisado em
31 de maio de 2023. Conferir a lista completa de autores antes do depósito.
