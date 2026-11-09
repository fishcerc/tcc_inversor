#include <WiFi.h>
#include <WebServer.h>
#include "driver/mcpwm.h"

// =====================================================
// ESP32 - INVERSOR TRIFÁSICO V/F - TCC
// FIRMWARE V4: STOP INSTANTÂNEO NA ISR E FALHA FÍSICA
// =====================================================

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

// Novo: Pino de Botão de Emergência / Falha Física (NF)
#define PINO_EMERGENCIA 25 

#define FREQUENCIA_CHAVEAMENTO 10000 
#define DEAD_TIME_TICKS 20 // Nominal: 20 * 100ns = 2.0us (A SER VALIDADO NO OSCILOSCÓPIO)

#define TAMANHO_TABELA 360
float tabelaSeno[TAMANHO_TABELA];
volatile float anguloFaseU = 0.0;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- CONTROLE E SEGURANÇA ----------------
const float FREQ_MAX = 60.0f;
const float RAMPA_HZ_S = 5.0f; 

volatile bool runCommand = false;
volatile bool hardwareIsPWM = false; // Flag que indica se o PWM está engatado nos pinos
volatile bool systemFault = false;   // Trava de segurança
String faultMessage = "SISTEMA OK";

volatile bool targetDirectionForward = true;
volatile bool currentDirectionForward = true;
volatile float frequencySetpoint = 0.0f;
volatile float frequencyActual = 0.0f;

unsigned long lastControlTime = 0;

// ---------------- HTML DA PÁGINA (Intacto) ----------------
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
.frequency { font-size: 42px; font-weight: bold; color: #38bdf8; text-align: center; margin: 10px 0; }
input[type=range], input[type=number] { width: 100%; padding: 12px; margin-top: 10px; border-radius: 8px; border: none; font-size: 18px; background: #334155; color: white;}
button { width: 100%; padding: 16px; margin-top: 10px; border: none; border-radius: 10px; color: white; font-size: 18px; font-weight: bold; cursor: pointer;}
.run { background: #15803d; }
.stop { background: #dc2626; }
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
function updateSlider() {
    let val = document.getElementById("slider").value;
    document.getElementById("freqInput").value = val;
    desiredFrequency = parseFloat(val);
    document.getElementById("freq").innerText = desiredFrequency.toFixed(1);
    fetch("/frequency?value=" + desiredFrequency).catch(console.error);
}
document.getElementById("freqInput").addEventListener("change", function() {
    let val = Math.max(0, Math.min(60, parseFloat(this.value) || 0));
    this.value = val; document.getElementById("slider").value = val;
    desiredFrequency = val; document.getElementById("freq").innerText = val.toFixed(1);
    fetch("/frequency?value=" + val).catch(console.error);
});
function sendRun() { fetch("/run"); }
function sendStop() { fetch("/stop"); }
function setDirection(val) { fetch("/direction?value=" + val); }

setInterval(() => {
    fetch("/status").then(r => r.json()).then(data => {
        let statEl = document.getElementById("status");
        if(data.fault) {
            statEl.innerText = "FALHA: " + data.msg;
            statEl.style.color = "#ef4444";
        } else if(data.run) {
            statEl.innerText = "EM OPERAÇÃO (RUN)";
            statEl.style.color = "#4ade80";
        } else {
            statEl.innerText = "PARADO (STOP)";
            statEl.style.color = "#fbbf24";
        }
        document.getElementById("btnDir").style.background = data.dir ? "#0284c7" : "#334155";
        document.getElementById("btnRev").style.background = data.dir ? "#334155" : "#0284c7";
    });
}, 500);
</script>
</body>
</html>
)rawliteral";


// =====================================================
// ISR DO SPWM (10 kHz) - PRIORIDADE MÁXIMA DE SEGURANÇA
// =====================================================
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);

    // 1. LEITURA FÍSICA IMEDIATA (Falha Externa / Botão Emergência)
    // Se o pino 25 soltar do GND (ficar HIGH), aciona o E-STOP
    if (digitalRead(PINO_EMERGENCIA) == HIGH) {
        systemFault = true;
        runCommand = false;
        faultMessage = "PARADA EMERGENCIA (PIN 25)";
    }
    
    // 2. KILL SWITCH IMEDIATO (Sem esperar loop de 10ms)
    if (!runCommand || systemFault) {
        if (hardwareIsPWM) {
            // Força o pino para ALTO direto no registrador do MCPWM 
            // (ALTO no ESP32 = Optoacoplador LIGA = Entrada do IPM em 0V = IGBT OFF)
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
            mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
            hardwareIsPWM = false;
        }
        portEXIT_CRITICAL_ISR(&timerMux);
        return; // Sai sem atualizar o SPWM
    }

    // 3. GERAÇÃO DO SPWM (Só executa se tudo estiver OK)
    // Passo baseado na interrupção de 100us (10 kHz)
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
void handleRun() { 
    if(!systemFault) { runCommand = true; }
    server.send(200, "text/plain", "OK"); 
}
void handleStop() { runCommand = false; server.send(200, "text/plain", "OK"); }
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
    String json = "{";
    json += "\"run\":"; json += runCommand ? "true" : "false";
    json += ",\"fault\":"; json += systemFault ? "true" : "false";
    json += ",\"msg\":\"" + faultMessage + "\"";
    json += ",\"dir\":"; json += targetDirectionForward ? "true" : "false"; 
    json += "}";
    server.send(200, "application/json", json);
}

// ---------------- MÁQUINA DE ESTADO E RAMPA LENTA ----------------
void updateSystem() {
    unsigned long now = millis();
    if (now - lastControlTime < 10) return; // Roda a 100 Hz
    float dt = (now - lastControlTime) / 1000.0f;
    lastControlTime = now;

    // Religamento Seguro: Devolve o controle ao gerador de PWM apenas quando RUN estiver ativo e sem falhas
    if (runCommand && !systemFault && !hardwareIsPWM) {
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        hardwareIsPWM = true;
    }

    // Rampa de Frequência (Mantida Intacta)
    float target = runCommand ? frequencySetpoint : 0.0f;
    float step = RAMPA_HZ_S * dt;

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
// SETUP E PROTEÇÕES INICIAIS
// =====================================================
void setup() {
    Serial.begin(115200);

    // Configura o Pino de Falha/Emergência (Puxado para HIGH pelo resistor interno)
    // Para rodar na bancada, você DEVE ligar o GPIO 25 ao GND.
    pinMode(PINO_EMERGENCIA, INPUT_PULLUP);

    for (int i = 0; i < TAMANHO_TABELA; i++) { tabelaSeno[i] = sin((i * PI) / 180.0); }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/run", HTTP_GET, handleRun);
    server.on("/stop", HTTP_GET, handleStop);
    server.on("/frequency", HTTP_GET, handleFrequency);
    server.on("/direction", HTTP_GET, handleDirection);
    server.on("/status", HTTP_GET, handleStatus);
    server.begin();

    // INICIALIZAÇÃO DO MCPWM
    esp_err_t err = ESP_OK;
    mcpwm_config_t pwm_config;
    pwm_config.frequency = FREQUENCIA_CHAVEAMENTO;
    pwm_config.cmpr_a = 0;    
    pwm_config.cmpr_b = 0;    
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; 
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U_H);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_U_L);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V_H);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PWM_V_L);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W_H);
    err |= mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, PWM_W_L);

    err |= mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    err |= mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    err |= mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // CONFIGURAÇÃO DO DEAD-TIME 
    err |= mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_LOW_COMPL, DEAD_TIME_TICKS, DEAD_TIME_TICKS);
    err |= mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_LOW_COMPL, DEAD_TIME_TICKS, DEAD_TIME_TICKS);
    err |= mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_LOW_COMPL, DEAD_TIME_TICKS, DEAD_TIME_TICKS);

    if (err != ESP_OK) {
        systemFault = true;
        faultMessage = "FALHA INIT MCPWM";
    }

    // BOOT SEGURO: Força imediatamente os pinos para nível HIGH (Sinal 0V no motor)
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
    hardwareIsPWM = false;

    // INICIA A ISR DO SPWM (10 kHz)
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 100, true); 
    timerAlarmEnable(timer);

    lastControlTime = millis();
}

void loop() {
    server.handleClient();
    updateSystem(); 
}