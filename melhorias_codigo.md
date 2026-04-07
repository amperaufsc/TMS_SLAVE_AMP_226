# Proposta de Melhorias no Código (TMS_SLAVE_AMP_226)

Abaixo estão os exemplos em código C demonstrando como aplicar cada uma das quatro melhorias sugeridas.

## 1. Sincronização Correta da Tarefa (ADC -> FreeRTOS)

O uso de `vTaskNotifyGiveFromISR` somado ao `ulTaskNotifyTake` garante que a *Thread* durma enquanto o ADC trabalha, poupando processamento (ao invés de fazer varredura contínua com `osDelay`).

```c
// Dentro de main.c ou app_freertos.c

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    // Esse callback funciona exatamente do jeito que está no seu código:
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xReadTempHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void xReadTempFunction(void *argument)
{
    static bool filtersInitialized = false;

    // xReadTempHandle OBRIGATÓRIAMENTE precisa receber o ID da tarefa corrente 
    // se não houver sido setado na inicialização
    // xReadTempHandle = xTaskGetCurrentTaskHandle();

    for(;;)
    {
        // 1. A thread fica DORMIDINHA aqui aguardando infinitamente (portMAX_DELAY) a interrupção do ADC.
        // Assim que a interrupção rodar, ela libera para continuar a execução.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(!filtersInitialized)
        {
            initTemperatureFilters(rawAdcBuffer);
            filtersInitialized = true;
        }

        for(int i = 0; i < numberOfThermistors; i++){
            uint16_t medianADC = applyMedianFilter(rawAdcBuffer[i], i);
            filteredAdcBuffer[i] = applyIIRFilter(medianADC, i);
            readStatus = checkThermistorConnection(filteredAdcBuffer[i]);

            if(readStatus == OK){
                // *** Aqui colocaremos a proteção com Mutex da próxima dica ***
                tempBuffer[i] = convertVoltageToTemperature(convertBitsToVoltage(filteredAdcBuffer[i]));
            }
            else{
                thermistorFault = 1;
                sendReadingErrorInfoIntoCAN();
                Error_Handler();
            }
        }
        
        // 2. Removemos o osDelay(1). O controle de tempo agora é de responsabilidade puramente da interrupção/DMA do ADC.
    }
}
```

---

## 2. Prevenção de Race Condition usando Mutex

Como `tempBuffer` é compartilhado por duas *Threads* (uma escreve, a outra lê e envia pro CAN), precisamos de um `Mutex` para que elas não tentem mexer no array simultaneamente.

Primeiro, crie seu mutex no STM32CubeIDE ou inicialize ele no código:

```c
// Declarado nas variáveis globais do `main.c`:
osMutexId_t tempBufferMutexHandle;
const osMutexAttr_t tempBufferMutex_attributes = {
  .name = "tempBufferMutex"
};

// No `main()`, dentro de /* USER CODE BEGIN RTOS_MUTEX */
tempBufferMutexHandle = osMutexNew(&tempBufferMutex_attributes);
```

Modificando as *Threads* para usarem o Mutex:

**Thread que Escreve as Temperaturas (xReadTemp):**
```c
// Dentro do for loop da xReadTempFunction
if(readStatus == OK){
    // Lock do Mutex antes de escrever
    if (osMutexAcquire(tempBufferMutexHandle, osWaitForever) == osOK) {
        tempBuffer[i] = convertVoltageToTemperature(convertBitsToVoltage(filteredAdcBuffer[i]));
        // Unlock do Mutex após escrever
        osMutexRelease(tempBufferMutexHandle);
    }
}
```

**Thread que Lê para Mandar pro CAN (xSendCAN):**
```c
void xSendCANFunction(void *argument)
{
    float localTempBuffer[numberOfThermistors]; // Variável local protetora

    for(;;)
    {
        // Pega os dados com proteção bem rápido
        if (osMutexAcquire(tempBufferMutexHandle, osWaitForever) == osOK) {
            // Copia o buffer global inteiro para uma versão local rapidamente
            memcpy(localTempBuffer, tempBuffer, sizeof(localTempBuffer));
            osMutexRelease(tempBufferMutexHandle);
        }

        // Agora nós lidamos com o envio CAN usando a cópia LOCAL (localTempBuffer) 
        // e não travamos a CPU e memórias da tarefa do ADC e evitamos bugs!
#ifdef slave1
        sendTemperatureToMaster(localTempBuffer, idSlave1Burst0);
#endif
        // ... repete para o restante ...

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        osDelay(100);
    }
}
```

---

## 3. Removendo Lógicas Pesadas do `main.c`

Para mover o código de forma elegante, você apenas recorta as funções do `main.c` e cola no `app_freertos.c`. As variáveis e diretivas precisam ser chamadas usando `extern`.

**No arquivo `app_freertos.c`**:
```c
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "adc.h" // Incluindo para os cálculos
#include "can.h" // Incluindo para os pacotes

// Acessando as variáveis exportadas por outro arquivo (Exemplo: main.c)
extern uint16_t rawAdcBuffer[];
extern float tempBuffer[];
extern osThreadId_t xReadTempHandle;

// Cria a função de Thread aqui dentro
void xReadTempFunction(void *argument)
{
     // Toda aquela lógica do ADC já explicada vem pra cá...
}

void xSendCANFunction(void *argument)
{
    // Toda aquela lógica do CAN já explicada vem pra cá...
}
```

*Lembrete:* As assinaturas destas funções não mudam; você só limpa a tela principal (`main.c`) da pesada lógica visual.

---

## 4. Hardware ID em substituição aos `#ifdef` (Opcional - Arquitetura Universal)

Caso queira ter um Hex unificado compilado, você criaria 2 pinos novos lidos na inicialização da sua C, que agiriam como *Hardware Dip Switch*. 

**Configuração Lógica:**
```c
// Dentro de main.c
uint8_t hardwareSlaveID = 0;

void readHardwareID() {
    // Digamos que você usou PA0 e PA1 no projeto para pinos com PULL-UP lendo para o GND.
    uint8_t bit0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    uint8_t bit1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
    
    // Calcula ID de 1 a 4.
    // Lógica Inversa sendo executada (pinos LOW em caso de Jumper/Dip-switch fechado).
    hardwareSlaveID = ((!bit1 << 1) | !bit0) + 1; 
}
```

No **`can.c`**, você remove os `#ifdef` do envio CAN e usa o ID da variável global via `extern` ou passando o parâmetro:

```c
extern uint8_t hardwareSlaveID;

void xSendCANFunction(void *argument)
{
    uint32_t currentBurstID = 0;
            
    switch(hardwareSlaveID){
        case 1: currentBurstID = idSlave1Burst0; break;
        case 2: currentBurstID = idSlave2Burst0; break;
        case 3: currentBurstID = idSlave3Burst0; break;
        case 4: currentBurstID = idSlave4Burst0; break;
        default: currentBurstID = idSlave1Burst0; break;
    }

    sendTemperatureToMaster(localTempBuffer, currentBurstID);
}
```

Isso garante que um mesmo ".hex" ou ".bin" possa ser gravado nas quatro placas satélite. A diferenciação acontece ligando pequenos *jumpers* mecânicos na board final.
