#include <WiFi.h>
#include <WebServer.h>
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

// =====================================================
// ESP32 - INVERSOR TRIFÁSICO V/F - TCC
// FIRMWARE V9: BARE-METAL ISR, TIMER SYNC, GPIO STRAPPING FIX
// CORE ARDUINO IDE COMPATÍVEL: 2.x
// =====================================================

const char* AP_SSID = "INVERSOR_TCC";
const char* AP_PASSWORD = "12345678";
WebServer server(80);

// ---------------- HARDWARE E PWM ----------------
// Fase W removida do GPIO 15 (Strapping). Nova Pinagem:
const int pinosPWM[6] = {19, 18, 17, 16, 33, 32}; 
#define PINO_EMERGENCIA GPIO_NUM_25 

#define FREQUENCIA_CHAVEAMENTO_CONF 20000 
#define DEAD_TIME_TICKS 20 // 2.0us nominais - OBRIGATÓRIO MEDIR NO OSCILOSCÓPIO

#define TAMANHO_TABELA 360
float tabelaSeno[TAMANHO_TABELA];
volatile float anguloFaseU = 0.0;
uint32_t max_duty_ticks = 0; // Valor máximo do registrador do timer (100% Duty)

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- VARIÁVEIS ATÔMICAS E ESTADOS ----------------
const float FREQ_MAX = 60.0f;
const float RAMPA_HZ_S = 5.0f; 

volatile bool runCommand = false;
volatile bool faultLatched = false; 
volatile bool mcpwmInitFailed = false;

volatile float frequencyActual = 0.0f; 
volatile bool currentDirectionForward = true;
volatile bool inverterActive = false; 

float frequencySetpoint = 0.0f;
bool targetDirectionForward = true;
unsigned long lastControlTime = 0;

// ---------------- HTML DA PÁGINA ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Inversor TCC V9</title>
<style>
* { box-sizing: border-box; }
body { margin: 0; padding: 20px; background: #101827; color: #f1f5f9; font-family: Arial, sans-serif; }
.container { max-width: 600px; margin: auto; }
.card { background: #1e293b; padding: 20px; margin-bottom: 16px; border-radius: 14px; }
h1 { font-size: 24px; margin-top: 0; }
.frequency { font-size: 42px; font-weight: bold; color: #38bdf8; text-align: center; margin: 10px 0; }
input[type=range], input[type=number] { width: 100%; padding: 12px; margin-top: 10px; border-radius: 8px; border: none; font-size: 18px; background: #334155; color: white;}
button { width: 100%; padding: 16px; margin-top: 10px; border: none; border-radius: 10px; color: white; font-size: 18px; font-weight: bold; cursor: pointer;}
.run { background: #15803d; }
.stop { background: #dc2626; }
.reset { background: #b91c1c; display: none; }
.direction { background: #334155; }
.status { text-align: center; padding: 12px; border-radius: 8px; font-weight: bold; font-size: 20px;}
.row { display: flex; gap: 10px; }
.row button { flex: 1; }
</style>
</head>
<body>
<div class="container">
    <div class="card">
        <h1>Inversor de Frequência</h1>
    </div>
    
    <div class="card">
        <div class="status" id="status" style="background:#334155; color:#fbbf24;">PARADO</div>
        <div class="frequency"><span id="freq">0.0</span> Hz</div>

        <label style="color:#94a3b8;">Frequência Alvo (Hz)</label>
        <input type="range" id="slider" min="0" max="60" step="0.5" value="0" oninput="updateSlider()">
        <input type="number" id="freqInput" min="0" max="60" step="0.5" value="0">
        
        <div class="row" style="margin-top: 15px;">
            <button class="run" id="btnRun" onclick="sendRun()">RUN</button>
            <button class="stop" onclick="sendStop()">STOP</button>
        </div>
        <button class="reset" id="btnReset" onclick="sendReset()">RESETAR FALHA (REINICIAR ESP32)</button>
    </div>
    
    <div class="card">
        <h3 style="margin-top:0;">Sentido de Rotação</h3>
        <div class="row">
            <button class="direction" id="btnDir" onclick="setDirection(1)">Direto</button>
            <button class="direction" id="btnRev" onclick="setDirection(0)">Reverso</button>
        </div>
    </div>
</div>

<script>
let desiredFrequency = 0;
function updateSlider() {
    let val = document.getElementById("slider").value;
    document.getElementById("freqInput").value = val;
    desiredFrequency = parseFloat(val);
    fetch("/frequency?value=" + desiredFrequency).catch(console.error);
}
document.getElementById("freqInput").addEventListener("change", function() {
    let val = Math.max(0, Math.min(60, parseFloat(this.value) || 0));
    this.value = val; document.getElementById("slider").value = val;
    desiredFrequency = val;
    fetch("/frequency?value=" + val).catch(console.error);
});
function sendRun() { 
    fetch("/run").then(r => {
        if(!r.ok) r.text().then(alert);
    }).catch(console.error); 
}
function sendStop() { fetch("/stop"); }
function sendReset() { 
    if(confirm("Reiniciar o Inversor para limpar a falha?")) {
        fetch("/reset").then(r => {
            if(!r.ok) r.text().then(alert);
            else setTimeout(() => location.reload(), 3000);
        }); 
    }
}
function setDirection(val) { fetch("/direction?value=" + val); }

setInterval(() => {
    fetch("/status").then(r => r.json()).then(data => {
        let statEl = document.getElementById("status");
        if(data.fault) {
            statEl.innerText = "FALHA LATCHED: " + data.msg;
            statEl.style.color = "#ef4444";
            document.getElementById("btnReset").style.display = "block";
            document.getElementById("btnRun").disabled = true;
            document.getElementById("btnRun").style.opacity = 0.5;
        } else if(data.run) {
            statEl.innerText = "EM OPERAÇÃO (RUN)";
            statEl.style.color = "#4ade80";
        } else {
            statEl.innerText = "PARADO (STOP)";
            statEl.style.color = "#fbbf24";
        }
        document.getElementById("freq").innerText = data.actual.toFixed(1);
        document.getElementById("btnDir").style.background = data.dir ? "#0284c7" : "#334155";
        document.getElementById("btnRev").style.background = data.dir ? "#334155" : "#0284c7";
    });
}, 500);
</script>
</body>
</html>
)rawliteral";


// =====================================================
// ISR DO SPWM (10 kHz) - ACESSO BARE-METAL E SEGURO
// =====================================================
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);

    // Latch Físico dentro da ISR
    if (gpio_get_level(PINO_EMERGENCIA) == 1) {
        faultLatched = true; 
    }
    
    if (!inverterActive || faultLatched || mcpwmInitFailed) {
        portEXIT_CRITICAL_ISR(&timerMux);
        return;
    }

    float passoAngulo = (frequencyActual * TAMANHO_TABELA) / 10000.0f;
    anguloFaseU += passoAngulo;
    if (anguloFaseU >= TAMANHO_TABELA) anguloFaseU -= TAMANHO_TABELA;

    float anguloFaseV, anguloFaseW;
    if (currentDirectionForward) {
        anguloFaseV = anguloFaseU + 120.0;
        anguloFaseW = anguloFaseU + 240.0;
    } else {
        anguloFaseV = anguloFaseU + 240.0;
        anguloFaseW = anguloFaseU + 120.0;
    }

    if (anguloFaseV >= TAMANHO_TABELA) anguloFaseV -= TAMANHO_TABELA;
    if (anguloFaseW >= TAMANHO_TABELA) anguloFaseW -= TAMANHO_TABELA;

    float tensaoBoost = 0.10f; 
    float indiceModulacao = tensaoBoost + (frequencyActual / 60.0f) * (1.0f - tensaoBoost);
    if (indiceModulacao > 1.0f) indiceModulacao = 1.0f;

    // Cálculo do Duty Cycle (0 a 100%)
    float duty_U = 50.0f + (indiceModulacao * 50.0f * tabelaSeno[(int)anguloFaseU]);
    float duty_V = 50.0f + (indiceModulacao * 50.0f * tabelaSeno[(int)anguloFaseV]);
    float duty_W = 50.0f + (indiceModulacao * 50.0f * tabelaSeno[(int)anguloFaseW]);

    // ESCRITA DIRETA NOS REGISTRADORES (Bare-Metal - Evita os spinlocks da mcpwm_set_duty)
    MCPWM0.channel[0].cmpr_value[0].cmpr_val = (uint32_t)((max_duty_ticks * duty_U) / 100.0f);
    MCPWM0.channel[1].cmpr_value[0].cmpr_val = (uint32_t)((max_duty_ticks * duty_V) / 100.0f);
    MCPWM0.channel[2].cmpr_value[0].cmpr_val = (uint32_t)((max_duty_ticks * duty_W) / 100.0f);

    portEXIT_CRITICAL_ISR(&timerMux);
}

// ---------------- HTTP HANDLERS ----------------
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleRun() { 
    if (mcpwmInitFailed) {
        server.send(400, "text/plain", "FALHA INIT PWM/TIMER");
        return;
    }
    if (faultLatched) {
        server.send(400, "text/plain", "SISTEMA BLOQUEADO. NECESSITA RESET.");
        return;
    }
    
    portENTER_CRITICAL(&timerMux);
    if (frequencyActual <= 0.1f) {
        currentDirectionForward = targetDirectionForward;
    } else {
        if (currentDirectionForward != targetDirectionForward) {
            portEXIT_CRITICAL(&timerMux);
            server.send(400, "text/plain", "RAMPA ATIVA: AGUARDE O MOTOR PARAR TOTALMENTE PARA INVERTER O SENTIDO.");
            return;
        }
    }
    portEXIT_CRITICAL(&timerMux);

    runCommand = true; 
    server.send(200, "text/plain", "RUN INICIADO"); 
}

void handleStop() { 
    runCommand = false; 
    server.send(200, "text/plain", "RAMPA DE PARADA INICIADA"); 
}

void handleReset() {
    if (gpio_get_level(PINO_EMERGENCIA) == 1) {
        server.send(400, "text/plain", "FALHA AINDA ATIVA. SOLTE O BOTAO NF DE EMERGENCIA PRIMEIRO.");
        return;
    }
    server.send(200, "text/plain", "Reiniciando Inversor...");
    delay(500);
    ESP.restart(); 
}

void handleFrequency() {
    if (!server.hasArg("value")) return;
    float val = server.arg("value").toFloat();
    if (val >= 0.0f && val <= FREQ_MAX) frequencySetpoint = val;
    server.send(200, "text/plain", "OK");
}

void handleDirection() {
    if (!server.hasArg("value")) return;
    targetDirectionForward = (server.arg("value").toInt() == 1); 
    server.send(200, "text/plain", "OK");
}

void handleStatus() {
    String msg = "SISTEMA OK";
    if (mcpwmInitFailed) msg = "FALHA INIT MCPWM/TIMER";
    else if (faultLatched) msg = "E-STOP ACIONADO (PIN 25)";

    portENTER_CRITICAL(&timerMux);
    float currentFreq = frequencyActual;
    portEXIT_CRITICAL(&timerMux);

    bool isFault = faultLatched || mcpwmInitFailed;

    String json = "{";
    json += "\"run\":"; json += runCommand ? "true" : "false";
    json += ",\"fault\":"; json += isFault ? "true" : "false";
    json += ",\"msg\":\"" + msg + "\"";
    json += ",\"dir\":"; json += targetDirectionForward ? "true" : "false"; 
    json += ",\"actual\":"; json += String(currentFreq, 1);
    json += "}";
    server.send(200, "application/json", json);
}

// ---------------- MÁQUINA DE ESTADO E RAMPA LENTA ----------------
void updateSystem() {
    unsigned long now = millis();
    if (now - lastControlTime < 10) return; 
    float dt = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;

    if (gpio_get_level(PINO_EMERGENCIA) == 1) {
        faultLatched = true;
    }

    portENTER_CRITICAL(&timerMux);
    float localFreq = frequencyActual;
    bool localInverterActive = inverterActive;
    portEXIT_CRITICAL(&timerMux);

    if (faultLatched || mcpwmInitFailed) {
        runCommand = false;
        // Zera o display de frequência para refletir a parada instantânea do E-Stop de Hardware
        if (localFreq > 0.0f) {
            portENTER_CRITICAL(&timerMux);
            frequencyActual = 0.0f;
            inverterActive = false;
            portEXIT_CRITICAL(&timerMux);
        }
        return;
    }

    float target = runCommand ? frequencySetpoint : 0.0f;
    float step = RAMPA_HZ_S * dt;

    if (localFreq < target) {
        localFreq += step;
        if (localFreq > target) localFreq = target;
    } else if (localFreq > target) {
        localFreq -= step;
        if (localFreq < target) localFreq = target;
    }
    
    if (localFreq > 0.1f && !localInverterActive) {
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        localInverterActive = true;
    } 
    else if (localFreq <= 0.1f && localInverterActive) {
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
        localInverterActive = false;
    }

    portENTER_CRITICAL(&timerMux);
    frequencyActual = localFreq;
    inverterActive = localInverterActive;
    portEXIT_CRITICAL(&timerMux);
}

// =====================================================
// SETUP E PROTEÇÕES DE SILÍCIO
// =====================================================
void setup() {
    Serial.begin(115200);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_0, PINO_EMERGENCIA);
    gpio_set_pull_mode((gpio_num_t)PINO_EMERGENCIA, GPIO_PULLUP_ONLY);

    if (gpio_get_level(PINO_EMERGENCIA) == 1) {
        faultLatched = true;
    }

    for (int i = 0; i < TAMANHO_TABELA; i++) { 
        float rad = (i * PI) / 180.0f;
        float senoComInjecao = sin(rad) + (1.0f / 6.0f) * sin(3.0f * rad);
        tabelaSeno[i] = senoComInjecao * 1.1547005f; 
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/run", HTTP_GET, handleRun);
    server.on("/stop", HTTP_GET, handleStop);
    server.on("/reset", HTTP_GET, handleReset);
    server.on("/frequency", HTTP_GET, handleFrequency);
    server.on("/direction", HTTP_GET, handleDirection);
    server.on("/status", HTTP_GET, handleStatus);
    server.begin();

    esp_err_t err = ESP_OK;
    mcpwm_config_t pwm_config;
    pwm_config.frequency = FREQUENCIA_CHAVEAMENTO_CONF; 
    pwm_config.cmpr_a = 0;    
    pwm_config.cmpr_b = 0;    
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; 
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pinosPWM[0]);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, pinosPWM[1]);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, pinosPWM[2]);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, pinosPWM[3]);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, pinosPWM[4]);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, pinosPWM[5]);

    err |= mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    err |= mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    err |= mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
    
    // Captura o valor máximo do período configurado no hardware para a matemática Bare-Metal na ISR
    max_duty_ticks = MCPWM0.timer[0].period.period;

    // SINCRONISMO DE HARDWARE: Alinha a Fase dos Timers 1 e 2 rigorosamente ao Timer 0
    err |= mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);
    err |= mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_SYNC0, 0);
    err |= mcpwm_sync_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_SELECT_SYNC0, 0);

    err |= mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_LOW_COMPL, DEAD_TIME_TICKS, DEAD_TIME_TICKS);
    err |= mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_LOW_COMPL, DEAD_TIME_TICKS, DEAD_TIME_TICKS);
    err |= mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_LOW_COMPL, DEAD_TIME_TICKS, DEAD_TIME_TICKS);

    err |= mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F0);
    err |= mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F0, MCPWM_ACTION_FORCE_HIGH, MCPWM_ACTION_FORCE_HIGH);
    err |= mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_F0, MCPWM_ACTION_FORCE_HIGH, MCPWM_ACTION_FORCE_HIGH);
    err |= mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_SELECT_F0, MCPWM_ACTION_FORCE_HIGH, MCPWM_ACTION_FORCE_HIGH);

    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);

    timer = timerBegin(0, 80, true);
    if (!timer) err |= ESP_FAIL;
    
    if (err == ESP_OK) {
        timerAttachInterrupt(timer, &onTimer, true);
        timerAlarmWrite(timer, 100, true); 
        timerAlarmEnable(timer);
    } else {
        mcpwmInitFailed = true;
    }

    lastControlTime = millis();
}

void loop() {
    server.handleClient();
    updateSystem(); 
}