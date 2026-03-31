# espFoC: pipeline completo em Q16.16 (opção B)

**Status:** **Fase A** (código de produção) em grande parte implementada; **Fase B** (testes + renomes) pendente. Ponte IQ31↔Q16 no LPF dos observers: [`esp_foc_iq31_q16_bridge.h`](../include/espFoC/utils/esp_foc_iq31_q16_bridge.h).

**Relação com 3.0:** O merge canónico actual usa **IQ31 (Q1.31)** no laço e tipos `*_iq31_t` em [`esp_foc_units_iq31.h`](../include/espFoC/esp_foc_units_iq31.h). Este milestone **substitui essa representação interna** por **Q16.16** (`int32_t` escalar: ~±32768 com resolução ~1/65536), alinhado com valores de engenharia (ganhos, pares de polos como escalar fixo, R/L, etc.) **sem** ficar preso a [-1, +1).

**Fonte de estado:** [`PLAN_STATUS.md`](PLAN_STATUS.md) — secção “Q16.16 full pipeline”.

---

## Decisão (opção B)

- **Um único formato fixo** no caminho de controlo e na API pública de grandezas escalares: **Q16.16** para sinais e parâmetros que precisem de **faixa > 1** e **fracções** úteis.
- **Inteiros puros** onde a semântica for contagem (ex.: CPR, índices): manter **`uint32_t` / `int`** explícitos — não forçar Q16.16 desnecessariamente.
- **Acumuladores e produtos:** usar **`int64_t`** onde o produto de dois Q16.16 ou misturas com escalas excederem 32 bits; documentar padrão **mul + shift + saturate** (equivalente ao “i31q32” discutido: largura 64-bit para intermediários, resultado final Q16.16 ou saturado).

---

## Riscos e mitigação

| Risco | Mitigação |
|-------|-----------|
| Regressão numérica em Park/Clarke/SVM | Reimplementar com **testes de referência float** temporários **só** na fase de testes (ou golden Q16.16 fixos após baseline); **não** bloquear a primeira onda de código |
| Saturação em cadeia | Política explícita: **onde** satura (por bloco), e uso de **int64** em nós críticos |
| LUT sin/cos | Reescalar entradas/ saídas para **ângulo e amplitude** em Q16.16 consistentes com o resto do pipeline |
| PID / LPF | Estado e coeficientes em **fixo**; integrador preferencialmente **int64** |

---

## Ordem de trabalho (obrigatória)

1. **Fase A — Implementação (código de produção)**  
   - Introduzir camada **`esp_foc_q16.h`** (nome final a alinhar): tipo `q16_t`, constantes, `q16_mul` → int64, `q16_div`, saturadores, conversões opcionais desde/para inteiro.  
   - Migrar **por subsistema**, numa ordem que mantenha build possível (mesmo que temporariamente coexistam `iq31_t` e `q16_t` atrás de typedefs — ver “Estratégia de migração”).  
   - **Substituir** [`esp_foc_units_iq31.h`](../include/espFoC/esp_foc_units_iq31.h) por unidades **Q16.16** (novo header ou rename planeado).  
   - **Core + sensored + drivers + utils** (foc_math, modulator, PID, LPF) até **zero `float` no hot path** e **mínimo float em init** (idealmente só conversão desde Kconfig/user literal se ainda existir).  
   - **Observers:** portar estado e parâmetros para Q16.16; intermediários 64-bit onde necessário.

2. **Fase B — Refactor de testes**  
   - Actualizar `test_*` para asserções em **Q16.16** e tolerâncias; remover ou isolar **paridade float** residual.  
   - Renomear ficheiros/tags onde ainda digam `iq31` no nome se a API deixar de ser IQ31-first.  
   - Actualizar [`test/README.md`](../test/README.md).

**Não inverter** a ordem: evita manter dois grandes conjuntos de testes verdes durante a migração de tipos.

---

## Estratégia de migração técnica (sugestão)

1. Definir **`typedef int32_t q16_t`** (ou nome de projeto) e **contrato global de escala** (o que é “1.0” em Q16.16 para tensão per-unit, corrente, ângulo eléctrico, etc.).  
2. Migrar **foc_math** primeiro (Clarke, Park, normalize, limit_voltage) — é dependência de todo o resto.  
3. **Modulador + SVM** em seguida.  
4. **PID + LPF** (estado 64-bit onde preciso).  
5. **Axis + core + sensored**.  
6. **Drivers** (wire format Q16.16).  
7. **Observers** por último ou em paralelo após foc_math estável.

Onde **IQ31** for apenas “per-unit [-1,1]”, o mapeamento natural é **Q16.16 com escala 1.0 = 32768** (ou constante única `Q16_ONE`) — validar uma vez e usar em todo o lado para não haver drift.

---

## Verificação

- `idf.py -D TEST_COMPONENTS=espFoC build` verde **após Fase B** (Fase A pode usar builds parciais ou testes desligados temporariamente **só se** o equipa aceitar — preferível manter componente a compilar sempre).  
- Documentar quebras de API em [`PLAN_STATUS.md`](PLAN_STATUS.md) quando o milestone fechar.

---

## Documentação utilizador

- **Depois** da Fase B (alinhado ao plano 3.0): tabela **grandeza → unidade → tipo → faixa** no README ou `docs/`.
