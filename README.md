# TMS_SLAVE_AMP_226 — Firmware de Monitoramento Térmico (Slave)

## Visão Geral

O **TMS_SLAVE_AMP_226** é o firmware embarcado de uma placa escrava do sistema de **Thermal Management System (TMS)** do veículo elétrico da equipe de Fórmula SAE. Ele roda em um microcontrolador **STM32G4** com **FreeRTOS** (CMSIS-RTOS v2) e é responsável por:

1. **Ler** 16 termistores conectados ao acumulador (bateria de alta tensão) via ADC + DMA.
2. **Filtrar** as leituras brutas com filtros de mediana e IIR para eliminar ruído.
3. **Converter** os valores filtrados em temperatura (°C) via polinômio de 4ª ordem.
4. **Transmitir** as temperaturas para a placa Master via barramento **FDCAN** (CAN 2.0 clássico).
5. **Receber** mensagens CAN da Master (ou de loopback interno para testes) via **FreeRTOS Message Queue**.
6. **Detectar falhas** de comunicação CAN (timeout com contagem de falhas consecutivas) e **acionar o Shutdown Circuit** caso a rede fique inoperante.

---

## Arquitetura do Software

```
┌──────────────────────────────────────────────────────────┐
│                        HARDWARE                          │
│  16x Termistores NTC → ADC2 (16ch) → DMA1               │
│  FDCAN1 → Barramento CAN interno (para Master)          │
│  GPIO PC7 = User LED | GPIO PC8 = Shutdown Pin           │
└──────────────────────────────────────────────────────────┘
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
     ┌─────────────┐ ┌──────────┐ ┌──────────────┐
     │ ISR: ADC    │ │ ISR: CAN │ │  FreeRTOS    │
     │ DMA Cplt    │ │ RxFifo0  │ │  Kernel      │
     │  Callback   │ │ Callback │ │              │
     └──────┬──────┘ └────┬─────┘ └──────────────┘
            │             │
   Notify   │    Queue    │
   (Give)   │    (Send)   │
            ▼             ▼
     ┌─────────────┐ ┌──────────────┐
     │ xReadTemp   │ │ xSendCAN     │
     │ (Normal)    │ │ (AboveNorm.) │
     │             │ │              │
     │ - Filtra    │ │ - Drena Queue│
     │ - Converte  │ │ - Envia CAN  │
     │ - Mutex     │ │ - Mutex      │
     └─────────────┘ └──────────────┘
```

---

## Estrutura de Arquivos

| Arquivo | Descrição |
|---------|-----------|
| `Core/Inc/adc.h` | Constantes do ADC, coeficientes do polinômio T(V), protótipos dos filtros |
| `Core/Src/adc.c` | Implementação dos filtros (mediana, IIR), conversão bits→tensão→temperatura |
| `Core/Inc/can.h` | IDs CAN, defines de seleção de slave, struct `CAN_RxMsg_t`, protótipos |
| `Core/Src/can.c` | Funções de transmissão CAN: envio de temperaturas e erros de leitura |
| `Core/Src/main.c` | Inicialização do hardware, callbacks de interrupção, threads FreeRTOS |

---

## Detalhamento das Funções

### Arquivo: `adc.c` — Processamento de Sinais Analógicos

#### `void initTemperatureFilters(uint16_t rawAdcBuffer[])`
Inicializa os buffers internos dos filtros com a primeira leitura real do ADC. Deve ser chamada apenas uma vez na primeira iteração do loop de leitura. Preenche o buffer circular da mediana com o valor inicial e seta o acumulador do filtro IIR.

#### `uint16_t median3(uint16_t a, uint16_t b, uint16_t c)`
Retorna a mediana de 3 valores usando comparações e swaps manuais. É a base do filtro de mediana que elimina picos espúrios (spikes) de ruído eletromagnético sem introduzir atraso significativo.

#### `uint16_t applyMedianFilter(uint16_t newReading, uint8_t index)`
Mantém um buffer circular de 3 amostras por canal. A cada nova leitura, insere o valor no buffer, calcula a mediana das 3 últimas amostras e retorna o resultado. O `index` identifica qual dos 16 termistores está sendo processado.

#### `uint16_t applyIIRFilter(uint16_t newReading, uint8_t index)`
Filtro IIR (Infinite Impulse Response) de primeira ordem com coeficiente α = 1/8. Implementado com shift de bits (`>> 3`) para evitar operações de ponto flutuante. Suaviza as oscilações residuais que o filtro de mediana não captura completamente.

**Cadeia completa de filtragem:** `ADC Bruto → Mediana (3 amostras) → IIR (α=1/8) → Valor Filtrado`

#### `float convertBitsToVoltage(uint16_t rawAdcVal)`
Converte o valor bruto do ADC (0–4095) para tensão real (0–3.3V) com a fórmula:
```
V = (rawAdcVal × 3.3) / 4095
```

#### `float convertVoltageToTemperature(float voltage)`
Converte tensão em temperatura (°C) usando um polinômio de 4ª ordem calibrado para o modelo de termistor NTC utilizado:
```
T(V) = C0 + C1·V + C2·V² + C3·V³ + C4·V⁴
```
Coeficientes: `C0=134.25`, `C1=-156.45`, `C2=105.64`, `C3=-40.82`, `C4=6.03`

#### `thermStatus checkThermistorConnection(uint16_t rawAdcVal)`
Diagnóstico de integridade do sensor. Retorna:
- `OK` → Leitura dentro da faixa válida.
- `THERM_SHORT` → ADC < 100 (curto-circuito no termistor).
- `THERM_OPEN` → ADC > 3995 (circuito aberto / cabo rompido).

---

### Arquivo: `can.c` — Comunicação CAN (Transmissão)

#### `static CAN_TxStatus_t sendSingleFrame(uint16_t identifier, uint8_t *data)`
Função auxiliar interna (`static`) que encapsula o envio de **um único quadro CAN** com toda a lógica de robustez:

1. **Verificação de estado:** Antes de tentar enviar, checa se o periférico FDCAN está em estado operacional (`HAL_FDCAN_STATE_BUSY`). Se não estiver (bus-off, erro de init), conta como falha imediata sem perder tempo no retry.
2. **Retry com backoff:** Tenta inserir o quadro na fila de TX do hardware até `CAN_TX_RETRY_MAX` vezes (padrão: 20), com 1ms de intervalo entre tentativas (`osDelay(1)`).
3. **Contador de falhas persistente:** A variável `canConsecutiveFailures` é `static` no escopo do arquivo (`can.c`), compartilhada entre todas as funções de transmissão. Qualquer envio bem-sucedido zera o contador.
4. **Retorno tipado:** Retorna `CAN_TX_OK`, `CAN_TX_FAIL` (quadro descartado, sistema segue) ou `CAN_TX_FATAL` (limiar atingido, chamador deve acionar Shutdown).

#### `CAN_TxStatus_t sendTemperatureToMaster(float buffer[], uint16_t baseID)`
Envia as 16 temperaturas para a Master em **8 quadros CAN** (2 floats por quadro = 8 bytes). Os IDs são sequenciais a partir do `baseID` (ex: `0x010`, `0x011`, ..., `0x017` para o Slave 1).

**Características de nível industrial:**
- Usa `memcpy` para copiar os floats para o buffer de TX, evitando violação de **strict aliasing** (cast `float*` sobre `uint8_t*` é *undefined behavior* no padrão C e pode gerar código incorreto em otimizações `-O2`/`-O3`).
- Delega cada quadro individualmente para `sendSingleFrame()`.
- Rastreia o "pior resultado" do burst inteiro e retorna para a thread chamadora.
- Se `sendSingleFrame` retornar `CAN_TX_FATAL`, chama `Error_Handler()` imediatamente (Shutdown).

**Mecanismo de Segurança (Timeout de Rede):**
- Cada quadro tenta ser enviado até **20 vezes** (`CAN_TX_RETRY_MAX`, configurável no `can.h`).
- Se falhar, incrementa `canConsecutiveFailures`.
- Se atingir **10 falhas consecutivas** (`CAN_TX_FAULT_THRESHOLD`) sem nenhum sucesso intermediário, aciona o pino de **Shutdown** e trava o processador (proteção crítica da bateria).
- Qualquer envio bem-sucedido **zera** o contador de falhas.

#### `void sendReadingErrorInfoIntoCAN(void)`
Envia um quadro CAN de erro para a Master informando que um termistor falhou (curto ou aberto). O ID utilizado depende da identidade do slave (definida por `#ifdef slaveN` no `can.h`). Reutiliza a mesma `sendSingleFrame()` para manter a lógica de retry e contagem de falhas unificada.

---

### Arquivo: `can.h` — Definições e Configuração CAN

#### Seleção de Identidade do Slave
```c
#define slave1          // Descomente apenas UM para cada placa física
//#define slave2
//#define slave3
//#define slave4
```

#### Constantes de Robustez
```c
#define CAN_TX_RETRY_MAX       20   // Tentativas por quadro antes de desistir (20ms)
#define CAN_TX_FAULT_THRESHOLD 10   // Falhas consecutivas antes de acionar Shutdown
#define CAN_RX_QUEUE_SIZE      16   // Capacidade da fila de recepção
```

#### `CAN_TxStatus_t` (enum)
Tipo de retorno das funções de transmissão: `CAN_TX_OK`, `CAN_TX_FAIL`, `CAN_TX_FATAL`.

#### Mapa de IDs CAN

| ID | Descrição |
|----|-----------|
| `0x00A` | Mensagem da Master (resumo do sistema) |
| `0x010–0x017` | Burst de temperaturas do Slave 1 (8 quadros) |
| `0x020–0x027` | Burst de temperaturas do Slave 2 |
| `0x030–0x037` | Burst de temperaturas do Slave 3 |
| `0x040–0x047` | Burst de temperaturas do Slave 4 |
| `0x050` | Erro de termistor do Slave 1 |
| `0x051` | Erro de termistor do Slave 2 |
| `0x052` | Erro de termistor do Slave 3 |
| `0x053` | Erro de termistor do Slave 4 |

#### `CAN_RxMsg_t` (struct)
Estrutura que encapsula um quadro CAN completo (header + 8 bytes de dados) para ser enfileirado na Message Queue do FreeRTOS.

---

### Arquivo: `main.c` — Núcleo do Firmware

#### Callbacks de Interrupção (ISR)

##### `void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)`
Chamada automaticamente pelo hardware quando o DMA termina de transferir todas as 16 leituras do ADC para a RAM. Envia uma **Task Notification** (`vTaskNotifyGiveFromISR`) para acordar a thread `xReadTemp`, que estava dormindo esperando os dados ficarem prontos.

##### `void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)`
Chamada automaticamente quando uma mensagem CAN chega na FIFO 0 do FDCAN1. Usa o padrão **Deferred Interrupt Processing**:
1. Lê a mensagem do hardware para uma **struct local na stack** (`CAN_RxMsg_t rxMsg`).
2. Empurra a struct inteira para a **Message Queue** do FreeRTOS (`xQueueSendFromISR`).
3. Se a fila estiver cheia (16 mensagens), a mensagem é descartada silenciosamente (sem travar).
4. Se a leitura falhar (quadro corrompido), ignora e retorna.

**Tempo gasto na ISR:** ~2–5 µs (apenas cópia de memória, sem processamento pesado).

---

#### Threads FreeRTOS

##### `void xReadTempFunction(void *argument)` — Prioridade: Normal
Responsável pela aquisição e processamento dos 16 canais de temperatura.

**Fluxo a cada iteração (100ms):**
1. Inicia uma conversão DMA do ADC2 com `HAL_ADC_Start_DMA()`.
2. **Dorme** no `ulTaskNotifyTake()` até o DMA completar a transferência.
3. Para o DMA com `HAL_ADC_Stop_DMA()`.
4. Na primeira execução, inicializa os filtros (`initTemperatureFilters`).
5. Para cada um dos 16 canais:
   - Aplica filtro de mediana (3 amostras).
   - Aplica filtro IIR (α = 1/8).
   - Verifica integridade do sensor (`checkThermistorConnection`).
   - Adquire o **Mutex** `tempBufferMutex` e grava a temperatura convertida no buffer compartilhado.
6. Espera 100ms (`osDelay(100)`) e repete.

##### `void xSendCANFunction(void *argument)` — Prioridade: Acima do Normal
Responsável pelo envio periódico das temperaturas e pelo processamento de mensagens recebidas.

**Fluxo a cada iteração (100ms):**
1. **(Modo testLoopback):** Drena todas as mensagens pendentes da Message Queue CAN e copia a última mensagem recebida para `lastRxMsg` (visível no Live Expressions do debugger).
2. Adquire o **Mutex** e copia o buffer de temperaturas para um buffer local.
3. Chama `sendTemperatureToMaster()` com o ID do slave ativo, enviando 8 quadros CAN com as 16 temperaturas.
4. Alterna o LED do usuário (GPIO PC7) como heartbeat visual.
5. Espera 100ms (`osDelay(100)`) e repete.

---

#### `void Error_Handler(void)`
Ação de último recurso (falha crítica):
1. Seta o pino **GPIO PC8** (conectado ao Shutdown Circuit do carro).
2. Desabilita todas as interrupções (`__disable_irq()`).
3. Entra em loop infinito (`while(1)`).

**Resultado:** O carro desliga imediatamente via abertura dos contatores.

---

## Mecanismos de Proteção

| Proteção | Implementação | Localização |
|----------|--------------|-------------|
| **Ruído do ADC** | Filtro de mediana + IIR em cascata | `adc.c` |
| **Termistor aberto/curto** | Verificação de faixa do ADC | `checkThermistorConnection()` |
| **Falha de rede CAN (TX)** | Contador de 10 falhas consecutivas → Shutdown | `sendTemperatureToMaster()` |
| **Race condition no buffer** | Mutex (`tempBufferMutex`) entre threads | `xReadTemp` / `xSendCAN` |
| **Perda de msgs CAN (RX)** | Message Queue com 16 slots de buffer | `HAL_FDCAN_RxFifo0Callback()` |
| **Travamento na ISR** | Deferred Interrupt Processing (ISR mínima) | Callbacks em `main.c` |

---

## Configuração de Hardware (CubeMX / .ioc)

| Periférico | Configuração |
|------------|-------------|
| **ADC2** | 16 canais, Scan Mode, DMA contínuo, 640.5 ciclos de amostragem |
| **FDCAN1** | Modo Normal, 500 kbps (Prescaler=10, Seg1=22, Seg2=11), Filtro global aceita tudo no FIFO0 |
| **DMA1 Ch1** | Ligado ao ADC2, prioridade 5 (compatível FreeRTOS) |
| **FreeRTOS** | CMSIS-RTOS v2, 2 threads + 1 mutex + 1 message queue |
| **GPIO PC7** | Saída — LED do usuário (heartbeat) |
| **GPIO PC8** | Saída — Pino de Shutdown (falha crítica) |

> **Nota:** Verificar que `TOTAL_HEAP_SIZE` ≥ 4096 bytes e `StdFiltersNbr = 1` no .ioc.

---

## Flags de Compilação Condicional

| Define | Efeito |
|--------|--------|
| `slave1` / `slave2` / `slave3` / `slave4` | Seleciona a identidade (IDs CAN) da placa física |
| `testLoopback` | Habilita a drenagem da Message Queue de RX para debug em Internal Loopback |

---

*Autor: Guilherme Lettmann — Fórmula SAE / AMP-226*
*Última atualização: Abril 2026*
