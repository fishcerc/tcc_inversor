# Inversor Trifásico TCC — V13

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

Os valores acima representam os componentes discutidos durante a montagem. A adequação elétrica, térmica e de tensão de cada componente deve ser confirmada antes da operação.

---

## 5. Pinagem do ESP32

A configuração de GPIO utilizada no firmware V9 é:

| Função | GPIO |
|---|---:|
| PWM fase U — superior | 19 |
| PWM fase U — inferior | 18 |
| PWM fase V — superior | 17 |
| PWM fase V — inferior | 16 |
| PWM fase W — superior | 32 |
| PWM fase W — inferior | 14 |
| Relé | 23 |
| LCD SDA | 21 |
| LCD SCL | 22 |

### Observações

- Os GPIOs 19, 18, 17, 16, 32 e 14 são utilizados para os seis sinais PWM.
- O GPIO 23 é utilizado para o comando do relé.
- O LCD utiliza comunicação I2C.
- O endereço I2C configurado no firmware é `0x27`.
- Caso o módulo LCD utilize outro endereço, será necessário ajustar a configuração.
- A configuração dos GPIOs deve ser confirmada na placa física utilizada.

---

## 6. Geração PWM

O firmware utiliza o periférico MCPWM do ESP32 para geração dos sinais de controle.

### 6.1 Configuração

| Parâmetro | Configuração |
|---|---|
| Frequência PWM alvo | 10 kHz |
| Configuração MCPWM | 20 kHz |
| Modo de contagem | UP_DOWN |
| Dead time nominal | 2 µs |
| Número de fases | 3 |
| Número de saídas | 6 |

O modo UP_DOWN é utilizado para geração dos sinais de comutação.

A configuração dos sinais complementares e do dead time deve ser verificada por osciloscópio.

### 6.2 Saídas

O firmware utiliza três pares de sinais:

- U_H e U_L.
- V_H e V_L.
- W_H e W_L.

Cada par representa os comandos de acionamento superior e inferior de uma fase.

A existência de sinais complementares no ESP32 não garante, por si só, que o acionamento do IPM esteja correto. A polaridade final depende também da interface de optoacoplamento e da configuração elétrica do circuito.

---

## 7. Modulação SPWM

O firmware V9 utiliza SPWM com injeção de terceira harmônica.

A geração das referências é realizada a partir de uma tabela senoidal de 1024 posições.

O programa realiza:

1. Inicialização da tabela senoidal.
2. Cálculo do ângulo da fase U.
3. Cálculo das fases V e W.
4. Aplicação do sentido de rotação selecionado.
5. Cálculo do índice da tabela.
6. Interpolação entre posições da tabela.
7. Cálculo do duty cycle.
8. Atualização dos sinais MCPWM.

A modulação é utilizada para produzir referências trifásicas defasadas em 120°.

### 7.1 Defasagem das fases

No sentido direto:

- Fase U: referência principal.
- Fase V: U + 120°.
- Fase W: U + 240°.

No sentido reverso:

- Fase U: referência principal.
- Fase V: U + 240°.
- Fase W: U + 120°.

A inversão é realizada pela alteração da sequência das referências das fases.

---

## 8. Controle V/F

O firmware utiliza uma referência de frequência de 0 a 60 Hz.

A frequência máxima configurada é:

**60 Hz**

A modulação é ajustada proporcionalmente à frequência de referência.

O firmware utiliza:

- Frequência mínima de operação efetiva da tarefa: 0,1 Hz.
- Frequência máxima: 60 Hz.
- Boost de modulação: 10%.
- Modulação máxima: 100%.

A relação V/F implementada no firmware é uma aproximação de controle da modulação.

A tensão efetiva de saída depende do barramento CC, da modulação, do estágio de potência e das características do motor.

---

## 9. Rampa de aceleração e desaceleração

O firmware utiliza uma rampa de frequência.

A taxa configurada é:

**5 Hz por segundo**

A referência de frequência é ajustada progressivamente até atingir o valor solicitado.

O sistema também utiliza a rampa durante o comando STOP.

A desaceleração ocorre por redução progressiva da frequência de referência.

A rampa não substitui uma estratégia de frenagem elétrica ou mecânica. O comportamento do motor durante a desaceleração deve ser verificado durante os ensaios.

---

## 10. Controle RUN e STOP

O sistema possui comandos de RUN e STOP na interface web.

### RUN

O comando RUN solicita a ativação do inversor.

O firmware verifica se:

- O MCPWM está pronto.
- A inicialização não apresentou falha.
- Não existe falha interna memorizada.

O sistema utiliza a referência de frequência configurada no painel.

### STOP

O comando STOP solicita a parada do inversor.

A parada é realizada por redução da frequência de referência.

O comportamento do motor durante a parada depende da carga e da estratégia de desaceleração.

### Limitação

O firmware V9 não possui entrada física de emergência.

O comando RUN/STOP é realizado pelo painel web.

A interface web não deve ser considerada um sistema de segurança funcional.

---

## 11. Controle de sentido de rotação

A interface web permite selecionar:

- Direto.
- Reverso.

A troca de sentido é aceita somente com a saída PWM desabilitada.

O painel solicita confirmação do operador antes de realizar a alteração.

Como o projeto não possui sensor de velocidade, o firmware não confirma fisicamente que o rotor parou.

A confirmação de parada deve ser realizada pelo operador antes de alterar o sentido de rotação.

---

## 12. Interface web

O ESP32 opera em modo Access Point.

### Configuração Wi-Fi

| Parâmetro | Valor |
|---|---|
| SSID | INVERSOR_TCC |
| Senha | 12345678 |
| Servidor HTTP | Porta 80 |

A interface web é acessada por um dispositivo conectado à rede Wi-Fi criada pelo ESP32.

### Recursos do painel

O painel disponibiliza:

- Frequência atual.
- Frequência de referência.
- Ajuste de frequência.
- Botão RUN.
- Botão STOP.
- Sentido direto.
- Sentido reverso.
- Diagnóstico.
- Indicador de falha.
- Botão de reinicialização após falha.

### Ajuste de frequência

O ajuste de frequência é realizado por:

- Slider.
- Botão de incremento.
- Botão de decremento.

A faixa configurada é de 0 a 60 Hz.

O incremento e decremento são de 1 Hz.

### Atualização de status

O painel consulta periodicamente o endpoint de status.

A atualização ocorre a cada 500 ms.

O painel exibe:

- Frequência atual.
- Referência de frequência.
- Estado do PWM.
- Estado do inversor.
- Contador de atualizações perdidas.
- Estado de falha.

---

## 13. LCD 16×2 I2C

O projeto utiliza um LCD 16×2 com comunicação I2C.

### Pinagem

| LCD | ESP32 |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |

O endereço configurado no firmware é `0x27`.

### Informações exibidas

A primeira linha apresenta:

- Frequência de referência.
- Estado RUN ou STOP.

A segunda linha apresenta:

- Sentido direto.
- Sentido reverso.

O firmware atualiza o LCD somente quando o conteúdo muda, reduzindo operações desnecessárias no barramento I2C.

---

## 14. FreeRTOS e atualização do PWM

O firmware utiliza uma tarefa FreeRTOS dedicada à atualização dos sinais PWM.

A interrupção MCPWM não realiza o cálculo completo das referências.

A ISR realiza a notificação da tarefa de atualização.

### Funcionamento

1. O MCPWM gera o evento de interrupção.
2. A ISR identifica o evento.
3. A tarefa PWM é notificada.
4. A tarefa lê o estado compartilhado.
5. Calcula as referências das fases.
6. Calcula os duty cycles.
7. Atualiza os sinais MCPWM.

A atualização do PWM é realizada fora da ISR.

### Monitoramento temporal

O firmware monitora o tempo de atualização.

O limite configurado é de 40 µs.

Se ocorrer erro de atualização ou atraso acima do limite, o firmware registra uma falha temporal.

O programa também contabiliza notificações acumuladas e atualizações perdidas.

---

## 15. Diagnóstico e falhas

O firmware possui mecanismos de diagnóstico para falhas internas.

### Falhas monitoradas

- Falha de inicialização do MCPWM.
- Erro de configuração do PWM.
- Erro de atualização dos sinais.
- Atraso de atualização.
- Acúmulo de notificações.
- Falha temporal.

### Comportamento em falha

Quando ocorre uma falha temporal, o firmware:

1. Registra a falha.
2. Desabilita o comando RUN.
3. Solicita o estado de repouso das saídas.
4. Desativa o estado lógico do inversor.
5. Zera a frequência atual.

A recuperação é realizada pelo comando de reinicialização após falha.

### Limitação

Esses mecanismos são de software.

Eles não substituem:

- Proteção contra sobrecorrente.
- Proteção contra curto-circuito.
- Proteção contra sobretensão.
- Proteção contra subtensão.
- Proteção térmica validada.
- Circuito físico de emergência.

---

## 16. Circuito de pré-carga do barramento CC

O projeto contempla um circuito de pré-carga para limitar a corrente inicial de carregamento dos capacitores do barramento CC.

Durante o desenvolvimento foram discutidos:

- Ponte retificadora.
- Capacitores do barramento.
- Resistor de pré-carga.
- Relé de bypass.
- Alimentação do circuito de controle.

O resistor discutido para pré-carga é:

**68 Ω / 5 W**

O banco de capacitores discutido utiliza quatro capacitores de 330 µF, organizados em dois pares.

A configuração elétrica final do banco deve ser confirmada no esquema do circuito.

### Funcionamento conceitual

1. O barramento CC é conectado ao circuito de pré-carga.
2. A corrente inicial é limitada pelo resistor.
3. Os capacitores do barramento são carregados.
4. O circuito de bypass realiza a conexão após a etapa de pré-carga.

O tempo real de carregamento depende da tensão inicial, capacitância total, resistência efetiva e comportamento do circuito.

O valor de tempo não deve ser definido somente por um atraso fixo de software.

A confirmação da carga deve ser realizada por medição apropriada do barramento.

---

## 17. Relé de bypass

O firmware utiliza o GPIO 23 para o comando do relé.

O relé está associado à etapa de bypass do circuito de pré-carga discutido durante o desenvolvimento.

O código atual configura o GPIO como saída e utiliza:

- LOW no estado inicial.
- HIGH para o acionamento.

Essa lógica pressupõe um circuito de acionamento ativo em nível alto.

A polaridade real deve ser confirmada no driver do relé.

### Limitação do firmware

O firmware não mede diretamente a tensão do barramento CC.

Portanto, o acionamento do relé baseado em atraso não confirma que os capacitores estejam carregados.

O tempo de pré-carga e a condição de fechamento devem ser validados com o circuito real.

---

## 18. Compilação e ambiente

O firmware foi desenvolvido para:

- ESP32 clássico.
- Arduino IDE.
- Arduino-ESP32 Core 2.x.
- MCPWM legacy do ESP-IDF 4.x.

### Bibliotecas utilizadas

| Biblioteca | Função |
|---|---|
| WiFi | Comunicação Wi-Fi |
| WebServer | Servidor HTTP |
| Wire | Comunicação I2C |
| LiquidCrystal_I2C | Controle do LCD |
| math.h | Operações matemáticas |
| driver/mcpwm.h | Controle MCPWM |
| driver/gpio.h | Controle GPIO |
| esp_intr_alloc.h | Gerenciamento de interrupções |
| esp_timer.h | Temporização |
| FreeRTOS | Tarefas e sincronização |

A compatibilidade das bibliotecas deve ser verificada na versão instalada do Arduino-ESP32.

---

## 19. Etapas de desenvolvimento e testes

### Etapa 1 — Verificação do ESP32

- Confirmar a placa ESP32 utilizada.
- Confirmar os GPIOs.
- Verificar alimentação.
- Confirmar comunicação serial.
- Compilar o firmware.

### Etapa 2 — Verificação do LCD

- Confirmar alimentação.
- Confirmar SDA e SCL.
- Confirmar endereço I2C.
- Verificar atualização das informações.

### Etapa 3 — Verificação do Wi-Fi

- Confirmar criação do Access Point.
- Conectar um dispositivo à rede.
- Acessar a interface web.
- Verificar atualização do status.

### Etapa 4 — Verificação dos sinais PWM

- Verificar os seis GPIOs.
- Medir a frequência PWM.
- Verificar a forma de onda.
- Verificar a defasagem entre as fases.
- Verificar os sinais complementares.
- Verificar o dead time.

### Etapa 5 — Verificação da modulação

- Ajustar a frequência de referência.
- Verificar a alteração dos duty cycles.
- Confirmar a sequência das fases.
- Verificar a alteração do sentido de rotação.

### Etapa 6 — Verificação de RUN e STOP

- Confirmar ativação do comando RUN.
- Confirmar redução de frequência no STOP.
- Verificar comportamento após falha.
- Confirmar bloqueio de RUN em condição de falha.

### Etapa 7 — Verificação do circuito de pré-carga

- Verificar a ponte retificadora.
- Verificar o banco de capacitores.
- Verificar o resistor de pré-carga.
- Medir a tensão do barramento.
- Verificar o comportamento do relé.
- Confirmar a sequência de carregamento.

### Etapa 8 — Validação do estágio de potência

A validação do IPM e do motor deve ocorrer somente após a verificação dos sinais, das proteções e do circuito de alimentação.

A operação com barramento de alta tensão exige procedimento de ensaio apropriado e instrumentação compatível.

---

## 20. Limitações atuais

O firmware V9 apresenta as seguintes limitações:

- Não possui entrada física de emergência.
- Não possui realimentação de velocidade.
- Não possui medição direta da tensão do barramento CC.
- Não confirma a carga dos capacitores antes do bypass.
- Não possui validação completa do acionamento do IPM.
- Não possui proteção de potência certificada.
- Não foi validado para operação com barramento de 311 Vcc.
- Não deve ser considerado firmware de operação industrial.

A interface web e o diagnóstico de software não substituem os circuitos físicos de proteção.

---

## 21. Próximos desenvolvimentos

Possíveis etapas futuras do projeto:

- Validar a geração dos seis sinais PWM.
- Validar a polaridade dos sinais após os optoacopladores.
- Confirmar a configuração do dead time.
- Validar a interface com o IKCM15L60GD.
- Implementar medição de tensão do barramento CC.
- Implementar confirmação da etapa de pré-carga.
- Implementar controle seguro do relé de bypass.
- Avaliar proteção física contra falhas.
- Avaliar realimentação de corrente.
- Avaliar realimentação de velocidade.
- Validar a operação com motor trifásico.

Esses itens são objetivos de desenvolvimento e não representam funcionalidades já implementadas no firmware V9.

---

## 22. Segurança

**Este projeto é experimental.**

O firmware V9 foi desenvolvido como base de ensaio de sinais.

O barramento CC de aproximadamente 311 V representa risco grave de choque elétrico e pode permanecer carregado mesmo após o desligamento da alimentação.

Durante os testes iniciais:

- Mantenha o barramento de potência fisicamente desconectado.
- Utilize fonte de baixa tensão e limitada para ensaios de sinais.
- Verifique os sinais com instrumentação apropriada.
- Confirme o estado das saídas durante boot e reset.
- Não considere o STOP web como emergência física.
- Não conecte o motor antes da validação do circuito de acionamento.
- Não energize o IPM sem conferir o datasheet e as conexões.
- Não realize medições no barramento CC sem procedimento e instrumentos adequados.

O circuito externo deve manter os gates desabilitados durante boot, reset, falha e ausência de alimentação do ESP32.

---

## 23. Resumo

O Inversor TCC V9 é uma plataforma experimental de controle de inversor trifásico baseada em ESP32.

O firmware reúne geração de sinais PWM, modulação SPWM, controle de frequência, rampa V/F, interface web, LCD e diagnóstico de falhas temporais.

O desenvolvimento do projeto envolve tanto o firmware quanto o circuito de acionamento, o módulo IPM e o circuito de pré-carga do barramento CC.

A próxima etapa principal é a validação dos sinais e das condições de acionamento antes da integração com o estágio de potência.

**Projeto em desenvolvimento — não validado para operação industrial ou acionamento com barramento de alta tensão.**
