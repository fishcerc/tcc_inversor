/*
 * INVERSOR TCC - V9
 * ESP32 classico (ESP32-WROOM/DevKit) + Arduino-ESP32 Core 2.x
 * MCPWM legacy do ESP-IDF 4.x
 *
 * FUNCOES:
 * - PWM trifasico com tres pares A/B (U, V, W)
 * - Portadora alvo: 10 kHz (MCPWM_UP_DOWN, config 20 kHz)
 * - SPWM com injecao de terceira harmonica
 * - Rampa V/F 0..60 Hz
 * - AP Wi-Fi + painel web
 * - STOP por rampa
 * - Sem entrada de emergencia fisica; RUN/STOP pelo painel web
 * - Atualizacao de duty em tarefa FreeRTOS, nao dentro da ISR
 * - Timer 1 e 2 sincronizados a partir do Timer 0
 *
 * ATENCAO:
 * Este firmware e uma base de ENSAIO DE SINAIS. Nao e firmware certificado
 * de seguranca nem esta validado para acionar o IPM com o barramento de 311 Vcc.
 * Use fonte de baixa tensao/limitada e osciloscopio, com o barramento CC de
 * potencia fisicamente desconectado. O circuito externo deve manter os gates
 * desabilitados durante boot, reset, falha e ausencia de alimentacao do ESP32.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "soc/mcpwm_struct.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// 1. CONFIGURACAO DO PROJETO
// ============================================================

static const char *AP_SSID = "INVERSOR_TCC";
static const char *AP_PASSWORD = "12345678";
WebServer server(80);

// ESP32 classico: U_H, U_L, V_H, V_L, W_H, W_L
static const int PIN_UH = 19;
static const int PIN_UL = 18;
static const int PIN_VH = 17;
static const int PIN_VL = 16;
static const int PIN_WH = 32; // GPIO32: pino de saida sem funcao de strapping no ESP32 classico.
static const int PIN_WL = 14;
static const int PWM_PINS[6] = {PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL};

const int PIN_RELE = 23;

// LCD 16x2 I2C: pinos padrao do ESP32 classico.
static const uint8_t LCD_SDA_PIN = 21;
static const uint8_t LCD_SCL_PIN = 22;
#define LCD_I2C_ADDRESS 0x27 // Se necessario, troque para 0x3F conforme o modulo I2C.
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);

static const uint32_t PWM_OUTPUT_HZ = 10000UL;
static const uint32_t MCPWM_CONFIG_HZ = PWM_OUTPUT_HZ * 2UL; // UP_DOWN: metade da config
static const uint32_t DEADTIME_TICKS = 20UL;                 // 20 x 100 ns = 2 us nominais
static const uint32_t PWM_ISR_TICK_BIT = (1UL << 3);          // Timer 0 TEZ (contador em zero)

static const float MOTOR_FREQ_MAX_HZ = 60.0f;
static const float RAMP_HZ_PER_SECOND = 5.0f;
static const float MODULATION_BOOST = 0.10f;
static const float MODULATION_MAX = 1.00f;

// A tabela e interpolada para reduzir erro de fase/amplitude.
static const uint16_t SINE_TABLE_SIZE = 1024;
static float sine3Table[SINE_TABLE_SIZE];

// IMPORTANTE: selecoes abaixo NAO podem ser consideradas corretas sem medir
// os sinais nos pinos do ESP32 e nas entradas do IKCM apos os 6N137.
// Esta configuracao inicial assume comandos PWM ativos em nivel alto no ESP32
// e saida HIGH como estado de repouso/desligado do conjunto opto + IPM.
#define PWM_DUTY_MODE          MCPWM_DUTY_MODE_0
#define PWM_DEADTIME_MODE      MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE
#define PWM_IDLE_IS_HIGH       1

static const uint32_t CONTROL_PERIOD_MS = 10;
static const uint32_t PWM_TASK_STACK = 4096;
static const UBaseType_t PWM_TASK_PRIORITY = 22;
static const BaseType_t PWM_TASK_CORE = 1;

// ============================================================
// 2. ESTADO COMPARTILHADO
// ============================================================

static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
static bool mcpwmInitFailed = false;                // inicializacao/infraestrutura
static bool pwmTimingFault = false;                 // atraso/erro de atualizacao
static bool runCommand = false;
static bool inverterActive = false;
static float frequencyActual = 0.0f;
static bool currentDirectionForward = true;
static uint32_t pwmUpdateOverruns = 0;

// Variaveis usadas somente pelo loop/WebServer.
static float frequencySetpoint = 0.0f;
static bool targetDirectionForward = true;
static uint32_t lastControlMillis = 0;

static TaskHandle_t pwmTaskHandle = nullptr;
static intr_handle_t mcpwmIntrHandle = nullptr;
static bool pwmHardwareReady = false;
static float phaseU_deg = 0.0f; // usada somente pela tarefa PWM

// Atualiza o LCD somente quando o conteudo muda, evitando trafego I2C desnecessario.
static void updateLcdDisplay() {
    static uint32_t lastCheckMs = 0;
    static char previousLine1[17] = "";
    static char previousLine2[17] = "";

    const uint32_t now = millis();
    if ((uint32_t)(now - lastCheckMs) < 200U) return;
    lastCheckMs = now;

    const float displayedFrequency = frequencySetpoint;
    const bool displayedDirectionForward = targetDirectionForward;

    bool displayedRun;
    portENTER_CRITICAL(&stateMux);
    displayedRun = runCommand;
    portEXIT_CRITICAL(&stateMux);

    char line1[17];
    char line2[17];
    char formatted[24];

    // Exemplo: "FREQ 36Hz  STOP" (com espacos finais ate completar 16 colunas).
    memset(line1, ' ', 16);
    line1[16] = '\0';
    snprintf(formatted, sizeof(formatted), "FREQ %2.0fHz  %-4s",
             displayedFrequency, displayedRun ? "RUN" : "STOP");
    size_t len1 = strlen(formatted);
    if (len1 > 16U) len1 = 16U;
    memcpy(line1, formatted, len1);

    // Exibe o sentido selecionado no painel web.
    memset(line2, ' ', 16);
    line2[16] = '\0';
    snprintf(formatted, sizeof(formatted), "Sentido %-6s",
             displayedDirectionForward ? "DIRETO" : "REV");
    size_t len2 = strlen(formatted);
    if (len2 > 16U) len2 = 16U;
    memcpy(line2, formatted, len2);

    if (strcmp(line1, previousLine1) != 0) {
        lcd.setCursor(0, 0);
        lcd.print(line1);
        memcpy(previousLine1, line1, sizeof(previousLine1));
    }
    if (strcmp(line2, previousLine2) != 0) {
        lcd.setCursor(0, 1);
        lcd.print(line2);
        memcpy(previousLine2, line2, sizeof(previousLine2));
    }
}

// ============================================================
// 3. PAGINA WEB
// ============================================================

static const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Inversor TCC V9</title>
<style>
*{box-sizing:border-box}body{margin:0;padding:18px;background:#101827;color:#f1f5f9;font-family:Arial,sans-serif}
.container{max-width:620px;margin:auto}.card{background:#1e293b;padding:20px;margin-bottom:15px;border-radius:14px}
h1{font-size:23px;margin:0}.freq{font-size:42px;font-weight:bold;color:#38bdf8;text-align:center;margin:14px 0}
.freq-caption{text-align:center;color:#94a3b8;font-size:12px;margin-top:-8px}
.freq-control{margin-top:20px;padding:15px;background:#0f172a;border-radius:12px}
.freq-setpoint{text-align:center;font-size:34px;font-weight:bold;color:#f8fafc;margin-bottom:12px}
.freq-setpoint span{color:#38bdf8}.freq-unit{font-size:16px;color:#94a3b8;margin-left:5px}
.freq-adjust{display:flex;align-items:center;gap:12px}
.freq-step{flex:0 0 52px;width:52px;height:52px;padding:0;margin:0;border-radius:12px;background:#334155;font-size:29px;line-height:1}
input[type=range]{flex:1;min-width:0;width:100%;padding:0;margin:0;background:transparent;accent-color:#38bdf8;cursor:pointer}
button{width:100%;padding:15px;margin-top:10px;border:0;border-radius:9px;color:white;font-size:17px;font-weight:bold;cursor:pointer}
button:disabled{opacity:.4;cursor:not-allowed}.row{display:flex;gap:10px}.row button{flex:1}.run{background:#15803d}.stop{background:#dc2626}.dir{background:#334155}
.status{padding:12px;text-align:center;border-radius:8px;background:#334155;color:#fbbf24;font-weight:bold}
.small{font-size:12px;color:#94a3b8;line-height:1.5}.fault{background:#7f1d1d}.reset{background:#991b1b;display:none}
</style>
</head>
<body><div class="container">
<div class="card"><h1>Inversor trifásico - TCC V9</h1><p class="small">Controle de bancada. Não conecte o barramento CC durante os testes de PWM.</p></div>
<div class="card">
 <div id="status" class="status">INICIALIZANDO</div>
 <div class="freq"><span id="actual">0.0</span> Hz</div>
 <div class="freq-caption">Frequência atual de saída</div>
 <div class="freq-control">
  <div class="small" style="text-align:center;margin-bottom:6px">FREQUÊNCIA DE REFERÊNCIA</div>
  <div class="freq-setpoint"><span id="setpoint">0</span><span class="freq-unit">Hz</span></div>
  <div class="freq-adjust">
   <button id="freqDown" class="freq-step" type="button" aria-label="Diminuir frequência em 1 Hz">−</button>
   <input id="slider" type="range" min="0" max="60" step="1" value="0" aria-label="Ajustar frequência de 0 a 60 Hz">
   <button id="freqUp" class="freq-step" type="button" aria-label="Aumentar frequência em 1 Hz">+</button>
  </div>
  <div class="small" style="text-align:center;margin-top:9px">Ajuste de 1 Hz · faixa de 0 a 60 Hz</div>
 </div>
 <div class="row"><button id="run" class="run">RUN</button><button id="stop" class="stop">STOP</button></div>
 <button id="reset" class="reset">REINICIAR APÓS FALHA</button>
</div>
<div class="card">
 <h3>Sentido de rotação</h3>
 <div class="row"><button id="forward" class="dir">Direto</button><button id="reverse" class="dir">Reverso</button></div>
 <p class="small">A troca de sentido só é aceita com a saída PWM desabilitada. Sem sensor de velocidade, confirme visualmente que o rotor parou antes de inverter.</p>
</div>
<div class="card"><div class="small" id="diagnostic">Diagnóstico: aguardando...</div></div>
</div>
<script>
const $=id=>document.getElementById(id);
let targetFrequency=0;
let sliderDragging=false;
let localEditUntil=0;
async function request(path){
 try{const r=await fetch(path,{cache:'no-store'});const t=await r.text();if(!r.ok)alert(t);return r.ok;}
 catch(e){console.error(e);alert('Falha de comunicação com o ESP32');return false;}
}
function renderSetpoint(v){
 let n=Number(v);
 if(!Number.isFinite(n))n=0;
 n=Math.max(0,Math.min(60,Math.round(n)));
 targetFrequency=n;
 $('slider').value=n;
 $('setpoint').textContent=String(n);
 $('freqDown').disabled=n<=0;
 $('freqUp').disabled=n>=60;
}
function setFreq(v){
 let n=Number(v);
 if(!Number.isFinite(n))return;
 n=Math.max(0,Math.min(60,Math.round(n)));
 renderSetpoint(n);
 localEditUntil=Date.now()+1200;
 request('/frequency?value='+encodeURIComponent(n));
}
$('freqDown').onclick=()=>setFreq(targetFrequency-1);
$('freqUp').onclick=()=>setFreq(targetFrequency+1);
$('slider').addEventListener('pointerdown',()=>{sliderDragging=true;});
window.addEventListener('pointerup',()=>{sliderDragging=false;});
$('slider').addEventListener('input',e=>{
 renderSetpoint(e.target.value);
 localEditUntil=Date.now()+1200;
});
$('slider').addEventListener('change',()=>setFreq(targetFrequency));
$('run').onclick=()=>request('/run');
$('stop').onclick=()=>request('/stop');
$('forward').onclick=()=>{if(confirm('Confirme que o motor está fisicamente parado antes de alterar o sentido.'))request('/direction?value=1');};
$('reverse').onclick=()=>{if(confirm('Confirme que o motor está fisicamente parado antes de alterar o sentido.'))request('/direction?value=0');};
$('reset').onclick=()=>{if(confirm('A saída está parada? Reiniciar o ESP32 para tentar recuperar a falha.'))request('/reset');};
async function poll(){
 try{
  const r=await fetch('/status',{cache:'no-store'});const d=await r.json();
  $('actual').textContent=Number(d.actual).toFixed(1);
  const receivedSetpoint=Number(d.setpoint);
  if(Number.isFinite(receivedSetpoint) && Date.now()>localEditUntil && !sliderDragging){
   renderSetpoint(receivedSetpoint);
  }
  $('status').textContent=d.msg;
  $('status').className=d.fault?'status fault':'status';
  $('run').disabled=d.fault||!d.ready;
  $('reset').style.display=d.fault?'block':'none';
  $('forward').style.background=d.dir?'#0284c7':'#334155';
  $('reverse').style.background=d.dir?'#334155':'#0284c7';
  $('diagnostic').textContent='PWM pronto: '+d.ready+' | Ativo: '+d.active+' | Atualizações perdidas: '+d.overruns+' | Referência: '+Number(d.setpoint).toFixed(1)+' Hz';
 }catch(e){$('status').textContent='SEM COMUNICAÇÃO';}
}
setInterval(poll,500);poll();
</script></body></html>
)rawliteral";

// ============================================================
// 4. FUNCOES AUXILIARES DE ESTADO
// ============================================================

static bool forceAllOutputsIdle();

static void setTimingFault() {
    portENTER_CRITICAL(&stateMux);
    pwmTimingFault = true;
    runCommand = false;
    portEXIT_CRITICAL(&stateMux);
}

static void stopAfterTimingFault() {
    // Executada em contexto de tarefa; as chamadas MCPWM nao estao na ISR.
    setTimingFault();
    if (pwmHardwareReady) forceAllOutputsIdle();
    portENTER_CRITICAL(&stateMux);
    inverterActive = false;
    frequencyActual = 0.0f;
    portEXIT_CRITICAL(&stateMux);
}

static bool readFaultState() {
    bool f;
    portENTER_CRITICAL(&stateMux);
    f = mcpwmInitFailed || pwmTimingFault;
    portEXIT_CRITICAL(&stateMux);
    return f;
}

static bool checkEsp(esp_err_t err, const char *operation) {
    if (err == ESP_OK) return true;
    Serial.printf("ERRO %s: %s (0x%x)\n", operation, esp_err_to_name(err), (unsigned int)err);
    mcpwmInitFailed = true;
    return false;
}

static bool forceAllOutputsIdle() {
    if (!pwmHardwareReady) return false;
    bool ok = true;
#if PWM_IDLE_IS_HIGH
    ok &= (mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A) == ESP_OK);
    ok &= (mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B) == ESP_OK);
    ok &= (mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A) == ESP_OK);
    ok &= (mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B) == ESP_OK);
    ok &= (mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A) == ESP_OK);
    ok &= (mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B) == ESP_OK);
#else
    ok &= (mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A) == ESP_OK);
    ok &= (mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B) == ESP_OK);
    ok &= (mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A) == ESP_OK);
    ok &= (mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B) == ESP_OK);
    ok &= (mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A) == ESP_OK);
    ok &= (mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B) == ESP_OK);
#endif
    return ok;
}

static bool enablePwmWaveforms() {
    bool ok = true;
    ok &= (mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, PWM_DUTY_MODE) == ESP_OK);
    ok &= (mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, PWM_DUTY_MODE) == ESP_OK);
    ok &= (mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, PWM_DUTY_MODE) == ESP_OK);
    ok &= (mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, PWM_DUTY_MODE) == ESP_OK);
    ok &= (mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, PWM_DUTY_MODE) == ESP_OK);
    ok &= (mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, PWM_DUTY_MODE) == ESP_OK);
    return ok;
}

static float tableLookup(float angleDeg) {
    while (angleDeg >= 360.0f) angleDeg -= 360.0f;
    while (angleDeg < 0.0f) angleDeg += 360.0f;
    const float position = angleDeg * ((float)SINE_TABLE_SIZE / 360.0f);
    uint16_t i0 = (uint16_t)position;
    if (i0 >= SINE_TABLE_SIZE) i0 = 0;
    const uint16_t i1 = (uint16_t)((i0 + 1U) % SINE_TABLE_SIZE);
    const float frac = (i0 == 0 && position >= (float)SINE_TABLE_SIZE)
        ? 0.0f : position - (float)i0;
    return sine3Table[i0] + (sine3Table[i1] - sine3Table[i0]) * frac;
}

static bool parseFrequencyArgument(float *valueOut) {
    if (!server.hasArg("value")) return false;
    String arg = server.arg("value");
    char buffer[32];
    arg.toCharArray(buffer, sizeof(buffer));
    char *end = nullptr;
    const float parsed = strtof(buffer, &end);
    if (end == buffer) return false;
    while (*end && isspace((unsigned char)*end)) ++end;
    if (*end != '\0' || !isfinite(parsed)) return false;
    if (parsed < 0.0f || parsed > MOTOR_FREQ_MAX_HZ) return false;
    *valueOut = parsed;
    return true;
}

// ============================================================
// 5. ISR MCPWM: SOMENTE NOTIFICA A TAREFA
// ============================================================

void IRAM_ATTR mcpwmZeroEventISR(void *arg) {
    (void)arg;
    const uint32_t status = MCPWM0.int_st.val;
    if (status & PWM_ISR_TICK_BIT) {
        MCPWM0.int_clr.val = PWM_ISR_TICK_BIT;
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        TaskHandle_t task = pwmTaskHandle;
        if (task != nullptr) {
            vTaskNotifyGiveFromISR(task, &higherPriorityTaskWoken);
        }
        if (higherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// ============================================================
// 6. TAREFA DE ATUALIZACAO DO SPWM
// ============================================================

void pwmUpdateTask(void *parameter) {
    (void)parameter;
    bool wasActive = false;

    for (;;) {
        const uint32_t ticks = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ticks == 0) continue;

        bool activeSnapshot;
        bool faultSnapshot;
        bool directionSnapshot;
        float frequencySnapshot;

        portENTER_CRITICAL(&stateMux);
        activeSnapshot = inverterActive;
        faultSnapshot = mcpwmInitFailed || pwmTimingFault;
        directionSnapshot = currentDirectionForward;
        frequencySnapshot = frequencyActual;
        if (ticks > 1) pwmUpdateOverruns += (ticks - 1U);
        portEXIT_CRITICAL(&stateMux);

        if (ticks > 1U) {
            // Se a tarefa perdeu um ou mais periodos de portadora, interrompe
            // a geracao em vez de continuar com uma sequencia temporal incerta.
            stopAfterTimingFault();
            continue;
        }

        if (faultSnapshot || !activeSnapshot || frequencySnapshot <= 0.1f) {
            phaseU_deg = 0.0f;
            wasActive = false;
            continue;
        }

        if (!wasActive) {
            phaseU_deg = 0.0f;
            wasActive = true;
        }

        // Um evento TEZ por periodo de 100 us = 10 k atualizacoes por segundo.
        phaseU_deg += frequencySnapshot * 360.0f / (float)PWM_OUTPUT_HZ;
        if (phaseU_deg >= 360.0f) phaseU_deg -= 360.0f;

        float phaseV = phaseU_deg + (directionSnapshot ? 120.0f : 240.0f);
        float phaseW = phaseU_deg + (directionSnapshot ? 240.0f : 120.0f);
        if (phaseV >= 360.0f) phaseV -= 360.0f;
        if (phaseW >= 360.0f) phaseW -= 360.0f;

        float modulation = MODULATION_BOOST +
            (frequencySnapshot / MOTOR_FREQ_MAX_HZ) * (MODULATION_MAX - MODULATION_BOOST);
        if (modulation > MODULATION_MAX) modulation = MODULATION_MAX;
        if (modulation < 0.0f) modulation = 0.0f;

        float dutyU = 50.0f + 50.0f * modulation * tableLookup(phaseU_deg);
        float dutyV = 50.0f + 50.0f * modulation * tableLookup(phaseV);
        float dutyW = 50.0f + 50.0f * modulation * tableLookup(phaseW);

        // Limites matematicos; nao substituem validacao de pulso minimo do IPM.
        dutyU = constrain(dutyU, 0.0f, 100.0f);
        dutyV = constrain(dutyV, 0.0f, 100.0f);
        dutyW = constrain(dutyW, 0.0f, 100.0f);

        const int64_t updateStartUs = esp_timer_get_time();
        const esp_err_t errU = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyU);
        const esp_err_t errV = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dutyV);
        const esp_err_t errW = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dutyW);
        const int64_t updateDurationUs = esp_timer_get_time() - updateStartUs;

        // O worker acorda no TEZ; o proximo TEP ocorre aprox. 50 us depois.
        // Se ultrapassar 40 us, registra falha temporal para parar em software.
        if (errU != ESP_OK || errV != ESP_OK || errW != ESP_OK || updateDurationUs > 40) {
            stopAfterTimingFault();
            Serial.printf("Falha PWM: U=%s V=%s W=%s tempo=%lld us\n",
                          esp_err_to_name(errU), esp_err_to_name(errV),
                          esp_err_to_name(errW), (long long)updateDurationUs);
        }
    }
}

// ============================================================
// 7. HTTP HANDLERS
// ============================================================

void handleRoot() {
    server.send_P(200, "text/html; charset=utf-8", PAGE);
}

void handleRun() {
    if (!pwmHardwareReady || mcpwmInitFailed) {
        server.send(503, "text/plain", "MCPWM nao esta pronto");
        return;
    }
    if (readFaultState()) {
        server.send(409, "text/plain", "Falha interna memorizada. Use REINICIAR APOS FALHA depois de parar a saida.");
        return;
    }

    portENTER_CRITICAL(&stateMux);
    if (!inverterActive && frequencyActual <= 0.1f) {
        // Aplica o sentido escolhido somente com a saida PWM desabilitada.
        // Sem sensor, o operador ainda precisa confirmar que o rotor parou.
        currentDirectionForward = targetDirectionForward;
    } else if (currentDirectionForward != targetDirectionForward) {
        portEXIT_CRITICAL(&stateMux);
        server.send(409, "text/plain", "Aguarde a parada e confirme que o rotor esta parado antes de inverter.");
        return;
    }
    runCommand = true;
    portEXIT_CRITICAL(&stateMux);

    server.send(200, "text/plain", "RUN aceito; rampa de aceleracao iniciada");
}

void handleStop() {
    portENTER_CRITICAL(&stateMux);
    runCommand = false;
    portEXIT_CRITICAL(&stateMux);
    server.send(200, "text/plain", "STOP aceito; rampa de desaceleracao iniciada");
}

void handleReset() {
    bool active;
    float freq;
    bool requestedRun;
    portENTER_CRITICAL(&stateMux);
    active = inverterActive;
    freq = frequencyActual;
    requestedRun = runCommand;
    portEXIT_CRITICAL(&stateMux);

    if (active || requestedRun || freq > 0.1f) {
        server.send(409, "text/plain", "Solicite STOP e aguarde a rampa terminar antes do reset.");
        return;
    }

    server.send(200, "text/plain", "Reiniciando ESP32 para limpar a falha interna...");
    delay(300);
    ESP.restart();
}

void handleFrequency() {
    float value;
    if (!parseFrequencyArgument(&value)) {
        server.send(400, "text/plain", "Frequencia invalida. Use um valor de 0 a 60 Hz.");
        return;
    }
    frequencySetpoint = value;
    server.send(200, "text/plain", "Referencia atualizada");
}

void handleDirection() {
    if (!server.hasArg("value")) {
        server.send(400, "text/plain", "Parametro value ausente");
        return;
    }
    const String value = server.arg("value");
    if (value != "0" && value != "1") {
        server.send(400, "text/plain", "Sentido invalido");
        return;
    }

    bool active;
    float freq;
    bool requestedRun;
    portENTER_CRITICAL(&stateMux);
    active = inverterActive;
    freq = frequencyActual;
    requestedRun = runCommand;
    portEXIT_CRITICAL(&stateMux);

    if (active || requestedRun || freq > 0.1f) {
        server.send(409, "text/plain", "O sentido so pode ser alterado com PWM desabilitado e frequencia zero.");
        return;
    }

    targetDirectionForward = (value == "1");
    server.send(200, "text/plain", "Sentido configurado. Confirme que o rotor esta fisicamente parado antes do RUN.");
}

void handleStatus() {
    bool runSnapshot, activeSnapshot, faultSnapshot, directionSnapshot;
    float actualSnapshot;
    uint32_t overrunsSnapshot;

    portENTER_CRITICAL(&stateMux);
    runSnapshot = runCommand;
    activeSnapshot = inverterActive;
    faultSnapshot = mcpwmInitFailed || pwmTimingFault;
    directionSnapshot = targetDirectionForward;
    actualSnapshot = frequencyActual;
    overrunsSnapshot = pwmUpdateOverruns;
    portEXIT_CRITICAL(&stateMux);

    String msg = "PARADO";
    if (mcpwmInitFailed) msg = "FALHA NA INICIALIZACAO MCPWM";
    else if (pwmTimingFault) msg = "FALHA DE TEMPO NA ATUALIZACAO PWM";
    else if (runSnapshot) msg = "RUN - RAMPA/OPERACAO";
    else if (activeSnapshot) msg = "PARANDO - RAMPA DE DESACELERACAO";

    String json = "{";
    json += "\"ready\":"; json += (pwmHardwareReady && !mcpwmInitFailed) ? "true" : "false";
    json += ",\"run\":"; json += runSnapshot ? "true" : "false";
    json += ",\"active\":"; json += activeSnapshot ? "true" : "false";
    json += ",\"fault\":"; json += faultSnapshot ? "true" : "false";
    json += ",\"msg\":\"" + msg + "\"";
    json += ",\"dir\":"; json += directionSnapshot ? "true" : "false";
    json += ",\"actual\":"; json += String(actualSnapshot, 2);
    json += ",\"setpoint\":"; json += String(frequencySetpoint, 2);
    json += ",\"overruns\":"; json += String(overrunsSnapshot);
    json += "}";
    server.send(200, "application/json", json);
}

// ============================================================
// 8. CONTROLE V/F E MAQUINA DE ESTADOS (LOOP, 100 Hz)
// ============================================================

void updateSystem() {
    const uint32_t now = millis();
    if ((uint32_t)(now - lastControlMillis) < CONTROL_PERIOD_MS) return;
    float dt = (float)(now - lastControlMillis) / 1000.0f;
    lastControlMillis = now;
    if (dt > 0.1f) dt = 0.1f; // evita salto grande apos bloqueio momentaneo

    bool fault;
    bool requestedRun;
    bool active;
    float actual;
    portENTER_CRITICAL(&stateMux);
    fault = mcpwmInitFailed || pwmTimingFault;
    requestedRun = runCommand;
    active = inverterActive;
    actual = frequencyActual;
    portEXIT_CRITICAL(&stateMux);

    if (fault) {
        if (pwmHardwareReady && active) forceAllOutputsIdle();
        portENTER_CRITICAL(&stateMux);
        runCommand = false;
        inverterActive = false;
        frequencyActual = 0.0f;
        portEXIT_CRITICAL(&stateMux);
        return;
    }

    const float target = requestedRun ? frequencySetpoint : 0.0f;
    const float maxStep = RAMP_HZ_PER_SECOND * dt;
    float nextFreq = actual;

    if (nextFreq < target) {
        nextFreq += maxStep;
        if (nextFreq > target) nextFreq = target;
    } else if (nextFreq > target) {
        nextFreq -= maxStep;
        if (nextFreq < target) nextFreq = target;
    }

    bool nextActive = active;

    if (nextFreq > 0.1f && !nextActive) {
        if (!enablePwmWaveforms()) {
            setTimingFault();
            forceAllOutputsIdle();
            return;
        }
        nextActive = true;
    }

    if (nextFreq <= 0.1f && target <= 0.1f) {
        nextFreq = 0.0f;
        if (nextActive) {
            if (!forceAllOutputsIdle()) {
                setTimingFault();
            }
        }
        nextActive = false;
        if (requestedRun && frequencySetpoint <= 0.1f) {
            portENTER_CRITICAL(&stateMux);
            runCommand = false;
            portEXIT_CRITICAL(&stateMux);
        }
    }

    portENTER_CRITICAL(&stateMux);
    frequencyActual = nextFreq;
    inverterActive = nextActive;
    portEXIT_CRITICAL(&stateMux);
}

// ============================================================
// 9. INICIALIZACAO DO MCPWM
// ============================================================

bool initializeMcpwm() {
    // Mantem GPIOs como saidas em HIGH antes de conectar a matriz MCPWM.
    // A seguranca no boot NAO pode depender apenas desta configuracao.
    for (uint8_t i = 0; i < 6; ++i) {
        pinMode(PWM_PINS[i], OUTPUT);
        digitalWrite(PWM_PINS[i], PWM_IDLE_IS_HIGH ? HIGH : LOW);
    }

    mcpwm_config_t config = {};
    config.frequency = MCPWM_CONFIG_HZ;
    config.cmpr_a = 50.0f;
    config.cmpr_b = 50.0f;
    config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    config.duty_mode = PWM_DUTY_MODE;

    // Inicializa os tres timers antes de rotear os GPIOs de saida.
    if (!checkEsp(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &config), "mcpwm_init T0")) return false;
    if (!checkEsp(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &config), "mcpwm_init T1")) return false;
    if (!checkEsp(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &config), "mcpwm_init T2")) return false;

    // Dead-time em cada meio-ponte. O modo escolhido precisa ser conferido
    // com a polaridade real dos 6N137 e das entradas HIN/LIN do IPM.
    if (!checkEsp(mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, PWM_DEADTIME_MODE, DEADTIME_TICKS, DEADTIME_TICKS), "deadtime T0")) return false;
    if (!checkEsp(mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, PWM_DEADTIME_MODE, DEADTIME_TICKS, DEADTIME_TICKS), "deadtime T1")) return false;
    if (!checkEsp(mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, PWM_DEADTIME_MODE, DEADTIME_TICKS, DEADTIME_TICKS), "deadtime T2")) return false;

    // Sincroniza T1 e T2 pelo evento TEZ de T0 (contador em zero).
    mcpwm_sync_config_t syncConfig = {};
    syncConfig.sync_sig = MCPWM_SELECT_TIMER0_SYNC;
    syncConfig.timer_val = 0;
    syncConfig.count_direction = MCPWM_TIMER_DIRECTION_UP;
    if (!checkEsp(mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_1, &syncConfig), "sync T1")) return false;
    if (!checkEsp(mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_2, &syncConfig), "sync T2")) return false;
    if (!checkEsp(mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ), "sync output T0")) return false;

    // Define nivel idle nos geradores antes de conectar os pinos externos.
#if PWM_IDLE_IS_HIGH
    if (!checkEsp(mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A), "idle U_H")) return false;
    if (!checkEsp(mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B), "idle U_L")) return false;
    if (!checkEsp(mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A), "idle V_H")) return false;
    if (!checkEsp(mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B), "idle V_L")) return false;
    if (!checkEsp(mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A), "idle W_H")) return false;
    if (!checkEsp(mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B), "idle W_L")) return false;
#else
    if (!checkEsp(mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A), "idle U_H")) return false;
    if (!checkEsp(mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B), "idle U_L")) return false;
    if (!checkEsp(mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A), "idle V_H")) return false;
    if (!checkEsp(mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B), "idle V_L")) return false;
    if (!checkEsp(mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A), "idle W_H")) return false;
    if (!checkEsp(mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B), "idle W_L")) return false;
#endif

    // Conecta as seis saidas MCPWM aos GPIOs depois do estado idle definido.
    if (!checkEsp(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_UH), "GPIO U_H")) return false;
    if (!checkEsp(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_UL), "GPIO U_L")) return false;
    if (!checkEsp(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_VH), "GPIO V_H")) return false;
    if (!checkEsp(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PIN_VL), "GPIO V_L")) return false;
    if (!checkEsp(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PIN_WH), "GPIO W_H")) return false;
    if (!checkEsp(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, PIN_WL), "GPIO W_L")) return false;

    return true;
}

// ============================================================
// 10. SETUP
// ============================================================

void setup() {
    pinMode(PIN_RELE, OUTPUT);
    digitalWrite(PIN_RELE, LOW); 
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("Inicializando Inversor TCC V9 - ENSAIO DE SINAIS");

    // Inicializa LCD 16x2 I2C (SDA=GPIO21, SCL=GPIO22).
    Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    Serial.println("LCD 16x2 I2C inicializado (endereco configurado em LCD_I2C_ADDRESS).");

    //Rele de pré carga
    Serial.println("Inicio da pre-carga");
    Serial.println("Acionando rele de bypass");
    lcd.setCursor(0, 0);
    lcd.print("TCC - Carlos");

    lcd.setCursor(0, 1);
    lcd.print("5seg CC Link");

    // Aguarda 5 segundos de pre-carga
    delay(5000);

    // Aciona o rele de bypass
    digitalWrite(PIN_RELE, HIGH);

    // Aguarda estabilizacao
    delay(500);

    // Atualiza LCD
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("Barramento OK");

    lcd.setCursor(0, 1);
    lcd.print("Iniciando...");


    // Tabela normalizada: maximo de sin(x) + sin(3x)/6 = sqrt(3)/2.
    // Fator 2/sqrt(3) = 1.1547005 normaliza o pico para +/-1.
    for (uint16_t i = 0; i < SINE_TABLE_SIZE; ++i) {
        const float angle = 2.0f * PI * (float)i / (float)SINE_TABLE_SIZE;
        const float fundamental = sinf(angle);
        const float third = sinf(3.0f * angle) / 6.0f;
        sine3Table[i] = (fundamental + third) * 1.154700538f;
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, handleRoot);
    server.on("/run", HTTP_GET, handleRun);
    server.on("/stop", HTTP_GET, handleStop);
    server.on("/reset", HTTP_GET, handleReset);
    server.on("/frequency", HTTP_GET, handleFrequency);
    server.on("/direction", HTTP_GET, handleDirection);
    server.on("/status", HTTP_GET, handleStatus);
    server.begin();

    if (!initializeMcpwm()) {
        mcpwmInitFailed = true;
        pwmHardwareReady = false;
        Serial.println("MCPWM NAO INICIALIZADO. PWM PERMANECE BLOQUEADO.");
        return;
    }

    pwmHardwareReady = true;

    // A tarefa de duty e criada antes de habilitar a interrupcao TEZ.
    if (xTaskCreatePinnedToCore(pwmUpdateTask, "pwmUpdate", PWM_TASK_STACK,
                                nullptr, PWM_TASK_PRIORITY, &pwmTaskHandle,
                                PWM_TASK_CORE) != pdPASS) {
        mcpwmInitFailed = true;
        pwmHardwareReady = false;
        Serial.println("Falha ao criar tarefa PWM.");
        return;
    }

    // ISR do MCPWM no Timer0 TEZ. A ISR apenas notifica a tarefa FreeRTOS.
    if (!checkEsp(mcpwm_isr_register(MCPWM_UNIT_0, mcpwmZeroEventISR, nullptr,
                                     ESP_INTR_FLAG_IRAM, &mcpwmIntrHandle), "registrar ISR MCPWM")) {
        pwmHardwareReady = false;
        return;
    }

    // Bit 3: interrupcao Timer0 TEZ. Nao habilitamos outros eventos do MCPWM.
    MCPWM0.int_clr.val = PWM_ISR_TICK_BIT;
    MCPWM0.int_ena.val = PWM_ISR_TICK_BIT;

    lastControlMillis = millis();
    Serial.println("V9 pronta para teste de sinais. Barramento CC deve permanecer desconectado.");
}

// ============================================================
// 11. LOOP
// ============================================================

void loop() {
    server.handleClient();
    updateSystem();
    updateLcdDisplay();
    delay(1);
}
