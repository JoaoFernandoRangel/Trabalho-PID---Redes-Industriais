# Sistema de Controle PID com ESP32

## Visão Geral
Este projeto implementa um sistema de controle PID (Proporcional-Integral-Derivativo) utilizando um ESP32 para controlar a posição de um atuador (como um servo motor ou motor DC) com feedback de um potenciômetro. O sistema inclui comunicação WiFi e MQTT para monitoramento e controle remoto através do ThingsBoard.

## Recursos Principais
- Controle PID ajustável com parâmetros Kp, Ki e Kd configuráveis remotamente
- Interface MQTT para comunicação com o ThingsBoard
- Telemetria em tempo real (setpoint, feedback, saída do PID, etc.)
- Controle remoto do setpoint e parâmetros PID
- Limites de segurança configuráveis (ledMax e ledMin)
- Persistência de dados (opcional, usando LittleFS)
- Multi-tarefa usando FreeRTOS (duas tasks (Potencialmente 3) rodando em núcleos diferentes)

## Hardware Necessário
- ESP32
- Potenciômetro (conectado ao pino 34)
- Motor DC ou servo motor (controlado por ponte H conectada aos pinos 32 e 33)
- Ambos o servo e o potênciometro fazem parte de um kit didático utilizado com frequência no IFES
- LED de status (pino interno LED_BUILTIN)
- Conexão WiFi

## Configuração
1. Edite o `conf.h` com suas credenciais:
   ```cpp
   #define SSID "seu_wifi"
   #define PWD "sua_senha"
   #define tbToken "seu_token_do_thingsboard"
   ```

## Funcionalidades MQTT
O sistema publica telemetria no tópico `v1/devices/me/telemetry` e escuta comandos RPC no tópico `v1/devices/me/rpc/request/+`.

### Comandos RPC disponíveis:
- `setSetPoint` - Define o valor desejado (setpoint)
- `setKp`, `setKi`, `setKd` - Ajusta os parâmetros PID
- `setLedMax`, `setLedMin` - Define os limites de segurança

## Estrutura do Código
- **PIDControl**: Task no Core 0 que lê o sensor e calcula o PID
- **mqttTask**: Task no Core 1 que gerencia conexões WiFi/MQTT e envia telemetria
- **salvaSetpoint**: Task no Core 1 (opcional) que salva periodicamente a posição atual

## Bibliotecas Necessárias
- Arduino.h -> Bibliotecas padrão do Arduino para PlatformIO
- PID_v1.h -> Biblioteca de cálculo de PID
- PubSubClient.h -> Client MQTT
- WiFi.h -> Biblioteca de conexão à wifi
- NTPClient.h -> Client de servidor NTP
- cJSON.h -> Biblioteca para processamento de JSON
- LittleFS.h (opcional) -> Sistema de arquivos do ESP32

## Melhorias Planejadas (TODOs)
- Adicionar widget para estado do LED no dashboard
- Pausar controle quando perder conexão
- Retornar à última posição após reset da placa
- Implementar botão de pânico (local e remoto)

## Observações
- O sistema mapeia a leitura do potenciômetro (0-4095) para um ângulo (0-180°)
- Os limites de segurança (ledMax e ledMin) acionam um LED quando ultrapassados
- A persistência de dados (LittleFS) não está implementada por padrão
- O desgaste do potênciometro na planta física causa ruídos nas leituras que distorcem a visão no dashboard. A implementação de um filtro na leitura do feedback pode ser considerada.

Para qualquer dúvida ou problema, consulte os comentários no código ou abra uma issue no repositório.