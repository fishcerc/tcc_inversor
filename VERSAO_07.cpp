#include <WiFi.h>
#include <WebServer.h>
#include "driver/mcpwm.h"
#include "driver/gpio.h"

// =====================================================
// ESP32 - INVERSOR TRIFÁSICO V/F - TCC
// FIRMWARE V6: HARDWARE FAULT (E-STOP) E ISR BARE-METAL
// =====================================================

const char* AP_SSID = "INVERSOR_TCC";
const char* AP_PASSWORD = "12345678";
WebServer server(80);

// ---------------- HARDWARE E PWM ----------------
const int pinosPWM[6] = {19, 18, 17, 16, 15, 14}; // U_H, U_L, V_H, V_L, W_H, W_L
#define PINO_EMERGENCIA 25 

// Para UP_DOWN_COUNTER gerar 10 kHz reais nos pinos, configura-se 20000
#define FREQUENCIA_CHAVEAMENTO_CONF 20000 
#define DEAD_TIME_TICKS 20 // Nominal: 2.0us (100ns/tick). OBRIGATÓRIO MEDIR.

#define TAMANHO_TABELA 360
float tabelaSeno[TAMANHO_TABELA];
volatile float anguloFaseU = 0.0;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- VARIÁVEIS ATÔMICAS E ESTADOS ----------------
const float FREQ_MAX = 60.0f;
const float RAMPA_HZ_S = 5.0f; 

bool runCommand = false;
bool mcpwmInitFailed = false;

// Variáveis de proteção compartilhadas (Protegidas por Spinlock)
volatile float frequencyActual = 0.0f; 
volatile bool currentDirectionForward = true;
volatile bool inverterActive = false; // Controla se a ISR deve calcular o SPWM

// Variáveis apenas de Loop
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
<title>Inversor TCC V6</title>
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
    fetch("/frequency?value=" + desiredFrequency).catch(console.error);
}
document.getElementById("freqInput").addEventListener("change", function() {
    let val = Math.max(0, Math.min(60, parseFloat(this.value) || 0));
    this.value = val; document.getElementById("slider").value = val;
    desiredFrequency = val;
    fetch("/frequency?value=" + val).catch(console.error);
});
function sendRun() { fetch("/run").then(r => r.text()).then(alert).catch(console.error); }
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
// ISR DO SPWM (10 kHz) - MATEMÁTICA PURA (Sem Lógica de Falha)
// =====================================================
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);

    // Se o inversor estiver inativo (0 Hz ou parado), poupa o processamento
    if (!inverterActive) {
        portEXIT_CRITICAL_ISR(&timerMux);
        return;
    }

    // A ISR confia cegamente no hardware. Se o E-STOP físico atuar, 
    // o módulo MCPWM ignora estes comandos e mantém os pinos em nível HIGH por si só.
    
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
    if (mcpwmInitFailed) {
        server.send(400, "text/plain", "FALHA INIT PWM");
        return;
    }
    // gpio_get_level = 1 significa E-STOP Acionado (Fio Partido ou Botão Pressionado)
    if (gpio_get_level((gpio_num_t)PINO_EMERGENCIA) == 1) {
        server.send(400, "text/plain", "FALHA E-STOP FISICO ATIVO");
        return;
    }

    // Limpa a falha One-Shot do hardware para permitir nova partida
    mcpwm_fault_clear(MCPWM_UNIT_0, MCPWM_SELECT_F0);

    runCommand = true; 
    server.send(200, "text/plain", "RUN INICIADO"); 
}

void handleStop() { 
    runCommand = false; 
    server.send(200, "text/plain", "RAMPA DE PARADA INICIADA"); 
}

void handleFrequency() {
    if (!server.hasArg("value")) {
        server.send(400, "text/plain", "Falta valor");
        return;
    }
    float val = server.arg("value").toFloat();
    if (val >= 0.0f && val <= FREQ_MAX) {
        frequencySetpoint = val;
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Freq Invalida");
    }
}

void handleDirection() {
    if (!server.hasArg("value")) return;
    targetDirectionForward = (server.arg("value").toInt() == 1); 
    server.send(200, "text/plain", "OK");
}

void handleStatus() {
    bool eStopActive = (gpio_get_level((gpio_num_t)PINO_EMERGENCIA) == 1);
    
    String msg = "SISTEMA OK";
    if (mcpwmInitFailed) msg = "FALHA MCPWM HW";
    else if (eStopActive) msg = "PARADA EMERGENCIA (PIN 25)";

    portENTER_CRITICAL(&timerMux);
    float currentFreq = frequencyActual;
    portEXIT_CRITICAL(&timerMux);

    String json = "{";
    json += "\"run\":"; json += runCommand ? "true" : "false";
    json += ",\"fault\":"; json += (eStopActive || mcpwmInitFailed) ? "true" : "false";
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

    // Monitoramento do Hardware de Emergência pelo Software
    bool eStopActive = (gpio_get_level((gpio_num_t)PINO_EMERGENCIA) == 1);
    if (eStopActive) {
        runCommand = false;
        // O MCPWM já travou as saídas via hardware neste momento
    }

    portENTER_CRITICAL(&timerMux);
    float localFreq = frequencyActual;
    portEXIT_CRITICAL(&timerMux);

    // Rampa de Frequência (Parada Suave vs Aceleração)
    float target = runCommand ? frequencySetpoint : 0.0f;
    float step = RAMPA_HZ_S * dt;

    if (localFreq < target) {
        localFreq += step;
        if (localFreq > target) localFreq = target;
    } else if (localFreq > target) {
        localFreq -= step;
        if (localFreq < target) localFreq = target;
    }

    // Lógica de Ativação/Desativação Física dos Pulsos a 0Hz
    bool localInverterActive = inverterActive;
    
    if (localFreq > 0.1f && !localInverterActive && !eStopActive && !mcpwmInitFailed) {
        // Liga o motor: Devolve o controle ao modo PWM (Senoide)
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        localInverterActive = true;
    } 
    else if (localFreq <= 0.1f && localInverterActive) {
        // Motor parado (Rampa zerou): Força os pinos para HIGH via software (Optos ON, IPM OFF)
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
        mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
        localInverterActive = false;
        
        // Permite inversão de sentido apenas quando parado
        currentDirectionForward = targetDirectionForward;
    }

    portENTER_CRITICAL(&timerMux);
    frequencyActual = localFreq;
    inverterActive = localInverterActive;
    portEXIT_CRITICAL(&timerMux);
}

// =====================================================
// SETUP E PROTEÇÕES DE SILÍCIO (HARDWARE FAULT)
// =====================================================
void setup() {
    Serial.begin(115200);

    // 1. CONFIGURAÇÃO DE HARDWARE DO E-STOP
    // Ligar um botão Normalmente Fechado (NF) entre GPIO 25 e GND.
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_FAULT_0, PINO_EMERGENCIA);
    gpio_set_pull_mode((gpio_num_t)PINO_EMERGENCIA, GPIO_PULLUP_ONLY);

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

    // 2. INICIALIZAÇÃO DO MCPWM (Com checagem rigorosa de erros)
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

    err |= mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_LOW_COMPL, DEAD_TIME_TICKS, DEAD_TIME_TICKS);
    err |= mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_LOW_COMPL, DEAD_TIME_TICKS, DEAD_TIME_TICKS);
    err |= mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_LOW_COMPL, DEAD_TIME_TICKS, DEAD_TIME_TICKS);

    // 3. A CARTADA FINAL: PROTEÇÃO DE FALHA DIRETO NO SILÍCIO
    // Configura o sinal de falha para disparar em nível ALTO (Fio partido ou botão acionado)
    err |= mcpwm_fault_init(MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F0);
    
    // Configura a Ação: Ao falhar, o hardware FORÇA NÍVEL ALTO nos pinos, ligando os optos e DESLIGANDO o IPM.
    // O modo "One-Shot" trava o inversor até ser explicitamente liberado via software.
    err |= mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F0, MCPWM_ACTION_FORCE_HIGH, MCPWM_ACTION_FORCE_HIGH);
    err |= mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_SELECT_F0, MCPWM_ACTION_FORCE_HIGH, MCPWM_ACTION_FORCE_HIGH);
    err |= mcpwm_fault_set_oneshot_mode(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_SELECT_F0, MCPWM_ACTION_FORCE_HIGH, MCPWM_ACTION_FORCE_HIGH);

    if (err != ESP_OK) mcpwmInitFailed = true;

    // 4. FORÇA ESTADO SEGURO LOGO APÓS O INIT (Para garantir o Boot)
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
    inverterActive = false;

    // 5. INICIA A ISR DO TIMER
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 100, true); // Interrupção a cada 100 us
    timerAlarmEnable(timer);

    lastControlTime = millis();
}

void loop() {
    server.handleClient();
    updateSystem(); 
}