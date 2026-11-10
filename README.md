# Inversor Trifásico TCC — V9

**Projeto de Automação Industrial | ESP32 | SPWM | IKCM15L60GD**

## 1. Sobre o projeto

O projeto Inversor TCC tem como objetivo desenvolver um inversor de frequência trifásico utilizando o ESP32 como unidade de controle, um módulo IPM para o estágio de potência e uma interface de operação para ajuste da frequência e controle do motor.

O firmware V9 foi desenvolvido para geração e controle de sinais PWM trifásicos, utilizando modulação SPWM, controle de frequência, rampa de aceleração e desaceleração e uma interface web acessível por Wi-Fi.

O projeto também contempla uma interface LCD 16×2, controle de sentido de rotação e uma estrutura de software baseada em tarefas FreeRTOS.

**Estado atual:** firmware de ensaio de sinais. A validação dos sinais, dos circuitos de acionamento e do estágio de potência é necessária antes de qualquer operação com motor ou barramento CC de alta tensão.

---

## 2. Objetivos

- Desenvolver um sistema de controle de inversor trifásico baseado em ESP32.
- Gerar sinais PWM para três fases.
- Implementar modulação SPWM com injeção de terceira harmônica.
- Implementar controle de frequência de 0 a 60 Hz.
- Desenvolver uma rampa de aceleração e desaceleração.
- Controlar RUN e STOP por interface web.
- Implementar seleção de sentido de rotação.
- Exibir frequência e estado do sistema em LCD.
- Monitorar falhas de atualização do PWM.
- Desenvolver e validar o circuito de pré-carga e bypass do barramento CC.

---

## 3. Arquitetura geral

O sistema é dividido em quatro partes principais:

### 3.1 Unidade de controle

O ESP32 realiza:

- Processamento das referências de frequência.
- Geração das referências trifásicas.
- Atualização dos sinais PWM.
- Controle de RUN e STOP.
- Controle de sentido de rotação.
- Comunicação Wi-Fi.
- Interface web.
- Atualização do LCD.
- Monitoramento de falhas de software.

### 3.2 Interface de acionamento

O circuito de acionamento recebe os sinais de controle do ESP32 e os encaminha ao módulo IPM.

O projeto utiliza optoacopladores 6N137 na interface de sinais discutida para o acionamento.

A polaridade efetiva dos sinais e o estado de repouso devem ser confirmados por medição antes da conexão ao IPM.

### 3.3 Estágio de potência

O estágio de potência utiliza o módulo IPM IKCM15L60GD, destinado ao acionamento trifásico.

O módulo recebe o barramento CC e realiza a comutação das três fases do motor.

### 3.4 Interface de operação

A interface de operação é composta por:

- Wi-Fi em modo Access Point.
- Página web de controle.
- LCD 16×2 com interface I2C.
- Comandos de frequência, RUN, STOP e sentido de rotação.

---

## 4. Hardware

### 4.1 Microcontrolador

**ESP32 clássico — ESP32-WROOM / DevKit**

O ESP32 é responsável pelo controle do inversor e pela geração dos sinais PWM.

O firmware utiliza o periférico MCPWM do ESP32 e a API legacy do ESP-IDF 4.x, compatível com a configuração indicada no código V9.

### 4.2 Módulo IPM

**IKCM15L60GD**

O módulo IPM é utilizado como estágio de potência do inversor trifásico.

O projeto considera:

- Alimentação de controle do módulo.
- Entradas de comando dos IGBTs.
- Capacitores bootstrap.
- Barramento CC.
- Saídas trifásicas.
- Emissores inferiores.
- Entrada de temperatura.
- Entrada de proteção ITRIP.

A pinagem e as conexões do módulo devem ser conferidas diretamente no datasheet antes da energização.

### 4.3 Componentes do circuito de potência e controle

Componentes discutidos durante o desenvolvimento:

- ESP32 DevKit.
- Módulo IPM IKCM15L60GD.
- Optoacopladores 6N137.
- Capacitores bootstrap de 4,7 µF / 50 V.
- Capacitores cerâmicos de 100 nF para bootstrap.
- Capacitores de barramento CC de 330 µF.
- Resistor de pré-carga de 68 Ω / 5 W.
- Ponte retificadora.
- Relé de bypass.
- Fonte de alimentação de 15 V para o IPM.
- Fonte de 5 V para ESP32 e IHM.
- LCD 16×2 I2C.
- Fusível.
- Capacitores de desacoplamento.
- Circuito de acionamento do relé.

Os valores acima representam os componentes discutidos durante a
