#include <WiFi.h>
#include <WebServer.h>
#include "driver/mcpwm.h"

// =====================================================
// ESP32 - Inversor Trifásico (Controle V/f) - TCC
// VERSÃO REVISADA: Lógica Invertida 6N137 + Boot Seguro
// =====================================================

// ---------------- WIFI ----------------
const char* AP_SSID = "INVERSOR_TCC";
const char* AP_PASSWORD = "12345678";
WebServer server(80);

// ---------------- HARDWARE E PWM ----------------
#define PWM_U_H 19
#define PWM_U_L 18
#define PWM_V_H 17
#define PWM_V_L 16
#define PWM_W_H 15
#define PWM_W_L 14

#define FREQUENCIA_CHAVEAMENTO 10000 // 10 kHz
#define TAMANHO_TABELA 360
float tabelaSeno[TAMANHO_TABELA];
volatile float anguloFaseU = 0.0;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- CONTROLE E ESTADOS ----------------
const float FREQ_MIN = 0.0f;
const float FREQ_MAX = 60.0f;
const float RAMPA_HZ_S = 5.0f; 
const unsigned long CONTROL_PERIOD_MS = 10; 

volatile bool runCommand = false;
volatile bool inverterIsRunning = false; // Controle de estado seguro
volatile bool targetDirectionForward = true;
volatile bool currentDirectionForward = true;
volatile float frequencySetpoint = 0.0f;
volatile float frequencyActual = 0.0f;

#define PINO_TEMP 36 
volatile bool modoAutomatico = false;
volatile float temperaturaAtual = 0.0f;
unsigned long lastControlTime = 0;

// ---------------- HTML / JAVASCRIPT DA PÁGINA ----------------
// (Mantido exatamente como o seu original, pois a interface web está perfeita)
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Inversor TCC</title>
<style>
* { box-sizing: border-box; }
body { margin: 0; padding: 20px; background: #101827; color: #f1f5f9; font-family: Arial, sans-serif; }
.container { max-width: 600px; margin: auto; }
.card { background: #1e293b; padding: 20px; margin-bottom: 16px; border-radius: 14px; }
h1 { font-size: 24px; margin-top: 0; }
.subtitle { color: #94a3b8; font-size: 14px; }
.frequency { font-size: 42px; font-weight: bold; color: #38bdf8; text-align: center; margin: 10px 0; }
.temp { font-size: 28px; font-weight: bold; color: #ef4444; text-align: center; margin-bottom: 20px; }
input[type=range] { width: 100%; }
input[type=number] { width: 100%; padding: 12px; margin-top: 10px; border-radius: 8px; border: none; font-size: 18px; background: #334155; color: white;}
input:disabled { opacity: 0.5; cursor: not-allowed; }
button { width: 100%; padding: 16px; margin-top: 10px; border: none; border-radius: 10px; color: white; font-size: 18px; font-weight: bold; cursor: pointer;}
.run { background: #15803d; }
.stop { background: #dc2626; }
.direction { background: #334155; }
.status { text-align: center; padding: 12px; border-radius: 8px; background: #334155; font-weight: bold; font-size: 20px;}
.row { display: flex; gap: 10px; }
.row button { flex: 1; }
</style>
</head>
<body>
<div class="container">
    <div class="card">
        <h1>Inversor de Frequência</h1>
        <div class="subtitle">Projeto TCC - ESP32 V/f</div>
    </div>
    
    <div class="card">
        <div class="status" id="status" style="color:#fbbf24;">PARADO</div>
        <div class="temp"><span id="temp">0.0</span> °C</div>
        <div class="frequency"><span id="freq">0.0</span> Hz</div>
        
        <div class="row" style="margin-bottom: 15px;">
            <button class="direction" id="btnMan" onclick="setMode(0)">Modo Manual</button>
            <button class="direction" id="btnAut" onclick="setMode(1)">Modo Auto</button>
        </div>

        <label style="color:#94a3b8;">Frequência Alvo (Hz)</label>
        <input type="range" id="slider" min="0" max="60" step="0.5" value="0" oninput="updateSlider()">
        <input type="number" id="freqInput" min="0" max="60" step="0.5" value="0">
        
        <div class="row" style="margin-top: 15px;">
            <button class="run" onclick="sendRun()">RUN</button>
            <button class="stop" onclick="sendStop()">STOP</button>
        </div>
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
let isFirstLoad = true;

function updateSlider() {
    let value = document.getElementById("slider").value;
    document.getElementById("freqInput").value = value;
    desiredFrequency = parseFloat(value);
    document.getElementById("freq").innerText = desiredFrequency.toFixed(1);
    sendFrequency();
}

document.getElementById("freqInput").addEventListener("change", function() {
    let value = parseFloat(this.value);
    if (isNaN(value)) value = 0;
    value = Math.max(0, Math.min(60, value));
    this.value = value;
    document.getElementById("slider").value = value;
    desiredFrequency = value;
    document.getElementById("freq").innerText = value.toFixed(1);
    sendFrequency();
});

function sendFrequency() { fetch("/frequency?value=" + desiredFrequency).catch(console.error); }
function sendRun() { fetch("/run").then(() => refreshStatus()).catch(console.error); }
function sendStop() { fetch("/stop").then(() => refreshStatus()).catch(console.error); }
function setDirection(value) { fetch("/direction?value=" + value).then(() => refreshStatus()).catch(console.error); }
function setMode(value) { fetch("/mode?auto=" + value).then(() => refreshStatus()).catch(console.error); }

function refreshStatus() {
    fetch("/status")
    .then(response => response.json())
    .then(data => {
        if(isFirstLoad) {
            document.getElementById("slider").value = data.target;
            document.getElementById("freqInput").value = data.target;
            document.getElementById("freq").innerText = data.target.toFixed(1);
            desiredFrequency = data.target;
            isFirstLoad = false;
        }

        let statEl = document.getElementById("status");
        if(data.run) {
            statEl.innerText = "EM OPERAÇÃO (RUN)";
            statEl.style.color = "#4ade80";
        } else {
            statEl.innerText = "PARADO (STOP)";
            statEl.style.color = "#fbbf24";
        }
        
        document.getElementById("temp").innerText = data.temp.toFixed(1);
        
        if (data.auto) {
            document.getElementById("freq").innerText = data.target.toFixed(1);
            document.getElementById("slider").value = data.target;
            document.getElementById("freqInput").value = data.target;
        }

        document.getElementById("slider").disabled = data.auto;
        document.getElementById("freqInput").disabled = data.auto;

        document.getElementById("btnMan").style.background = data.auto ? "#334155" : "#0284c7";
        document.getElementById("btnAut").style.background = data.auto ? "#0284c7" : "#334155";
        document.getElementById("btnDir").style.background = data.target_direction ? "#0284c7" : "#334155";
        document.getElementById("btnRev").style.background = data.target_direction ? "#334155" : "#0284c7";
    }).catch(console.error);
}

setInterval(refreshStatus, 500);
refreshStatus();
</script>
</body>
</html>
)rawliteral";

// =====================================================
// INTERRUPÇÃO DO TIMER (GERAÇÃO SPWM E V/F)
// =====================================================
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    
    // ESTADO DE PARADA SEGURA (Correção Aplicada)
    if (!runCommand || frequencyActual < 0.1f) {
        if (inverterIsRunning) {
            // Força fisicamente o Nível ALTO nos 6 pinos. 
            // ALTO no ESP32 = Optoacoplador LIGA = Entrada do IPM vai para GND (Desliga as Chaves)
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
            inverterIsRunning = false;
        }
        portEXIT_CRITICAL_ISR(&timerMux);
        return;
    }

    // ESTADO DE OPERAÇÃO
    if (!inverterIsRunning) {
        // Remove a trava de nível HIGH e devolve os pinos para o gerador de PWM de hardware
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        inverterIsRunning = true;
    }

    // Geração do SPWM
    float passoAngulo = (frequencyActual * TAMANHO_TABELA) / FREQUENCIA_CHAVEAMENTO;
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

    float tensaoBoost = 0.10; 
    float indiceModulacao = tensaoBoost + (frequencyActual / 60.0) * (1.0 - tensaoBoost);
    if (indiceModulacao > 1.0) indiceModulacao = 1.0;

    float duty_U = 50.0 + (indiceModulacao * 50.0 * tabelaSeno[(int)anguloFaseU]);
    float duty_V = 50.0 + (indiceModulacao * 50.0 * tabelaSeno[(int)anguloFaseV]);
    float duty_W = 50.0 + (indiceModulacao * 50.0 * tabelaSeno[(int)anguloFaseW]);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_U);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_V);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_W);

    portEXIT_CRITICAL_ISR(&timerMux);
}

// ---------------- HTTP HANDLERS ----------------
void handleRoot() { server.send_P(200, "text/html", PAGE); }
void handleRun() { runCommand = true; server.send(200, "text/plain", "RUN"); }
void handleStop() { runCommand = false; server.send(200, "text/plain", "STOP"); }
void handleFrequency() {
    if (!server.hasArg("value")) return;
    float value = server.arg("value").toFloat();
    if (value >= FREQ_MIN && value <= FREQ_MAX) {
        frequencySetpoint = value;
        server.send(200, "text/plain", "OK");
    }
}
void handleDirection() {
    if (!server.hasArg("value")) return;
    targetDirectionForward = (server.arg("value").toInt() == 1); 
    server.send(200, "text/plain", "OK");
}
void handleMode() {
    if (!server.hasArg("auto")) return;
    modoAutomatico = (server.arg("auto").toInt() == 1);
    server.send(200, "text/plain", "OK");
}
void handleStatus() {
    String json = "{";
    json += "\"run\":"; json += runCommand ? "true" : "false";
    json += ",\"direction\":"; json += currentDirectionForward ? "true" : "false"; 
    json += ",\"target_direction\":"; json += targetDirectionForward ? "true" : "false"; 
    json += ",\"auto\":"; json += modoAutomatico ? "true" : "false"; 
    json += ",\"temp\":"; json += String(temperaturaAtual, 1);
    json += ",\"target\":"; json += String(frequencySetpoint, 1);
    json += ",\"actual\":"; json += String(frequencyActual, 1);
    json += "}";
    server.send(200, "application/json", json);
}

// ---------------- LÓGICA DE CONTROLE ----------------
void updateControl() {
    unsigned long now = millis();
    if (now - lastControlTime < CONTROL_PERIOD_MS) return;
    unsigned long elapsed = now - lastControlTime;
    lastControlTime = now;
    
    int leituraBruta = analogRead(PINO_TEMP);
    float tempLida = (leituraBruta / 4095.0) * 100.0; 
    temperaturaAtual = (temperaturaAtual * 0.9) + (tempLida * 0.1); 

    if (modoAutomatico && runCommand) {
        if (temperaturaAtual <= 30.0) {
            frequencySetpoint = 0.0;
        } else if (temperaturaAtual >= 50.0) {
            frequencySetpoint = 60.0; 
        } else {
            frequencySetpoint = 15.0 + ((temperaturaAtual - 30.0) * (45.0 / 20.0));
        }
    }

    float dt = elapsed / 1000.0f;
    float step = RAMPA_HZ_S * dt;
    float target = frequencySetpoint;

    if (frequencyActual < target) {
        frequencyActual += step;
        if (frequencyActual > target) frequencyActual = target;
    } else if (frequencyActual > target) {
        frequencyActual -= step;
        if (frequencyActual < target) frequencyActual = target;
    }

    if (frequencyActual <= 0.1f && (targetDirectionForward != currentDirectionForward)) {
        currentDirectionForward = targetDirectionForward;
    }
} 

// =====================================================
// SETUP GERAL E PROTEÇÃO DE BOOT
// =====================================================
void setup() {
    Serial.begin(115200);

    // 1. BOOT SEGURO (Impede shoot-through durante a inicialização)
    uint8_t pinosPWM[] = {PWM_U_H, PWM_U_L, PWM_V_H, PWM_V_L, PWM_W_H, PWM_W_L};
    for (int i = 0; i < 6; i++) {
        pinMode(pinosPWM[i], OUTPUT);
        digitalWrite(pinosPWM[i], HIGH); // Mantém os optos ligados (Saída 0V no IPM)
    }
    delay(50); // Garante estabilização elétrica antes de ligar o hardware interno

    for (int i = 0; i < TAMANHO_TABELA; i++) { tabelaSeno[i] = sin((i * PI) / 180.0); }

    WiFi.mode(WIFI_AP);
    if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
        Serial.println("Wi-Fi: INVERSOR_TCC | IP: 192.168.4.1");
    }

    server.on("/", HTTP_GET, handleRoot);
    server.on("/run", HTTP_GET, handleRun);
    server.on("/stop", HTTP_GET, handleStop);
    server.on("/frequency", HTTP_GET, handleFrequency);
    server.on("/direction", HTTP_GET, handleDirection);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/mode", HTTP_GET, handleMode);
    server.begin();

    // 2. CONFIGURA O MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U_H);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_U_L);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V_H);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PWM_V_L);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W_H);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, PWM_W_L);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = FREQUENCIA_CHAVEAMENTO;
    pwm_config.cmpr_a = 0;    
    pwm_config.cmpr_b = 0;    
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; 
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // 3. DEAD-TIME CONFIGURADO CORRETAMENTE
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_LOW_COMPL, 320, 320);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_LOW_COMPL, 320, 320);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_LOW_COMPL, 320, 320);

    // Garante que o estado inicial do hardware MCPWM esteja em High
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);

    // 4. INICIA O TIMER DO SPWM
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 100, true); 
    timerAlarmEnable(timer);

    lastControlTime = millis();
}

void loop() {
    server.handleClient();
    updateControl();
}