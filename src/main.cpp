#include <Arduino.h>
#include "PID_v1.h"
#include "PubSubClient.h"
#include "WiFi.h"
#include "WiFiClient.h"
#include "NTPClient.h"
#include "WiFiUdp.h"
#include "cJSON.h"
#include "conf.h"
#include "LittleFS.h"

// TODO Adicionar widget no dashboard para estado do led

// TODO Freiar a task de controle quando houver perda de conexão

// TODO Freiar PID quando perder conexão

// TODO Fazer voltar para onde estava quando reiniciar

// TODO Botão de pânico - No dashboard e local

// Variáveis globais e configurações do PID
double kp = 30, kd = 0, ki = 0;
double setpoint = 90,feedback, output;
PID pid(&feedback, &output, &setpoint, kp, ki, kd, DIRECT);

#define potPin 34
#define rightPin 32
#define leftPin 33

// Array de pontos e variáveis de controle
double pontos[5] = {30, 60, 90, 150, 120};
int counter = 0;
unsigned long timer[2] = {0};
uint16_t ledMax = 145; // Máximo, Mínimo
uint16_t ledMin = 45;  // Máximo, Mínimo

WiFiClient espClient;
PubSubClient client(espClient);
WiFiUDP _ntpUDP;
NTPClient _timeClient(_ntpUDP, "pool.ntp.org", -10800);
const char *mqtt_server = "demo.thingsboard.io";

void connectToWifi();
void manageMQTT();
void reconnectMQTT();
void manageWiFi();
void callback(char *topic, byte *payload, unsigned int length);
String buildPayload(double kp, double ki, double kd, double ouput, double setpoint, double feedback, bool ledState, String timeStamp,
                    unsigned long epoch);
void handleDownlink(String payload);
bool sendTelemetry(String payload);
void savePosToFile(double position);
double getLastPosFromFile();

// Protótipos das funções
void iniciaPinos();
void iniciaPID(int time, int limits);
void PIDControl(void *pvParameters);    // Task no Core 0
void mqttTask(void *pvParameters);      // Task no Core 1
void salvaSetpoint(void *pvParameters); // Task no Core 1

bool ledState;

void setup() {
    Serial.begin(115200);
    iniciaPID(100, 4095);
    iniciaPinos();
    // if (LittleFS.begin()) {
    //     setpoint = getLastPosFromFile();
    // }

    // Cria as tasks:
    // Task no Core 0 - Leitura do PID
    xTaskCreatePinnedToCore(PIDControl, "LeituraPID", 10000, NULL, 1, NULL, 0);

    // Tasks no Core 1
    // Task 1 - Controle do Motor
    xTaskCreatePinnedToCore(mqttTask, "mqttTask", 10000, NULL, 1, NULL, 1);

    // Task 2 - Mudança de Setpoint
    // xTaskCreatePinnedToCore(salvaSetpoint, "salvaSetpoint", 10000, NULL, 1, NULL, 1);
}

// Task no Core 0 - Leitura do sensor e cálculo do PID
void PIDControl(void *pvParameters) {
    while (1) {
        feedback = map(analogRead(potPin), 130, 4095, 0, 180);
        if (output > 0) {
            analogWrite(rightPin, output);
            analogWrite(leftPin, 0);
        } else {
            analogWrite(rightPin, 0);
            analogWrite(leftPin, abs(output));
        }
        if (feedback > ledMax || feedback < ledMin) {
            ledState = true;
        } else {
            ledState = false;
        }
        digitalWrite(LED_BUILTIN, ledState);

        pid.Compute();
        vTaskDelay(10 / portTICK_PERIOD_MS); // Pequeno delay para dar tempo a outras tasks
    }
}

// Task no Core 1 - Controle do Motor
void mqttTask(void *pvParameters) {
    connectToWifi();
    _timeClient.begin();
    _timeClient.update();
    manageMQTT();
    unsigned long lastMsg = millis();
    while (1) {
        if (WiFi.status() == WL_DISCONNECTED) {
            connectToWifi();
        }
        if (!client.connected()) {
            manageMQTT();
        } else {
            client.loop();
            if (millis() - lastMsg > 500) {
                if (sendTelemetry(
                        buildPayload(kp, ki, kd, output, setpoint, feedback, ledState, _timeClient.getFormattedTime(), millis()))) {
                    Serial.println("Mensagem publicada com sucesso");
                } else {
                    Serial.println("Falha ao publicar a mensagem");
                }
                lastMsg = millis();
            }
            _timeClient.update();
        }
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

bool sendTelemetry(String payload) {
    return client.publish("v1/devices/me/telemetry", payload.c_str());
}

// Task no Core 1 - Mudança periódica do setpoint
void salvaSetpoint(void *pvParameters) {
    unsigned long time = millis();
    while (1) {
        if (millis() - time > 1000) {
            savePosToFile(setpoint);
            time = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay maior pois a verificação é a cada 2s
    }
}

void iniciaPinos() {
    pinMode(potPin, INPUT);
    pinMode(rightPin, OUTPUT);
    pinMode(leftPin, OUTPUT);
    digitalWrite(rightPin, LOW);
    digitalWrite(leftPin, LOW);
    pinMode(LED_BUILTIN, OUTPUT);
}

void iniciaPID(int time, int limits) {
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-limits, limits);
    pid.SetSampleTime(time);
}

void connectToWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PWD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Conectando ao WiFi...");
    }
    Serial.println("Conectado ao WiFi");
}
void connectToWifi(unsigned long &t) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PWD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Conectando ao WiFi...");
    }
    Serial.println("Conectado ao WiFi");
}
void manageMQTT() {
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    reconnectMQTT();
}

void reconnectMQTT() {
    int contadorMQTT = 0;
    while (!client.connected() && contadorMQTT < 15) {
        Serial.print("Tentando conectar ao MQTT...");
        if (client.connect("ESP32Client", tbToken, NULL)) {
            Serial.println("Conectado");
            client.subscribe("v1/devices/me/rpc/request/+");
            contadorMQTT = 0;
        } else {
            Serial.print("falhou, rc=");
            Serial.print(client.state());
            Serial.println(" tente novamente em 5 segundos");
            delay(5000);
            contadorMQTT++;
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length) {
    // Converta o payload para uma string
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print("Mensagem recebida [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(message);
    handleDownlink(message);
    // TODO adicionar led state
    client.publish("v1/devices/me/telemetry",
                   buildPayload(kp, ki, kd, output, setpoint, feedback, ledState, _timeClient.getFormattedTime(), millis()).c_str());
}

void manageWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWifi();
    }
}

void handleDownlink(String payload) {
    cJSON *root = cJSON_Parse(payload.c_str());
    if (root == NULL) {
        Serial.println("Erro ao parsear JSON");
        return;
    }

    cJSON *method = cJSON_GetObjectItem(root, "method");
    cJSON *params = cJSON_GetObjectItem(root, "params");

    if (method == NULL || params == NULL) {
        Serial.println("JSON inválido - faltam campos obrigatórios");
        cJSON_Delete(root);
        return;
    }

    String methodStr = method->valuestring;

    if (methodStr == "setSetPoint") {
        setpoint = params->valuedouble;
        Serial.printf("Setpoint atualizado para: %.2f\n", setpoint);
    } else if (methodStr == "setKp") {
        kp = params->valuedouble;
        pid.SetTunings(kp, ki, kd);
        Serial.printf("Kp atualizado para: %.2f\n", kp);
    } else if (methodStr == "setKi") {
        ki = params->valuedouble;
        pid.SetTunings(kp, ki, kd);
        Serial.printf("Ki atualizado para: %.2f\n", ki);
    } else if (methodStr == "setKd") {
        kd = params->valuedouble;
        pid.SetTunings(kp, ki, kd);
        Serial.printf("Kd atualizado para: %.2f\n", kd);
    } else if (methodStr == "setLedMax") {
        ledMax = params->valueint;;
        Serial.printf("LedMax atualizado para: %.2f\n", params->valuedouble);
    } else if (methodStr == "setLedMin") {
        ledMin = params->valueint;
        Serial.printf("LedMin atualizado para: %.2f\n", params->valuedouble);
    } else {
        Serial.printf("Método desconhecido: %s\n", methodStr.c_str());
    }
    cJSON_Delete(root);
}

String buildPayload(double kp, double ki, double kd, double ouput, double setpoint, double feedback, bool ledState, String timeStamp,
                    unsigned long epoch) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "kp", kp);
    cJSON_AddNumberToObject(root, "ki", ki);
    cJSON_AddNumberToObject(root, "kd", kd);
    cJSON_AddNumberToObject(root, "ouput", ouput);
    cJSON_AddStringToObject(root, "timestamp", timeStamp.c_str());
    cJSON_AddNumberToObject(root, "setpoint", setpoint);
    cJSON_AddNumberToObject(root, "feedback", feedback);
    cJSON_AddBoolToObject(root, "ledState", ledState);
    cJSON_AddNumberToObject(root, "epoch", epoch);
    String ret = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return ret;
}

void loop() {
    vTaskSuspend(NULL);
}

void savePosToFile(double position) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "lastPos", position);
    String str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    File f = LittleFS.open("/lastPos.json", "w+");
    if (f) {
        f.write((const uint8_t *)str.c_str(), str.length());
        f.close(); // Boa prática: sempre feche o arquivo
    }
}

double getLastPosFromFile() {
    File f = LittleFS.open("/lastPos.json", "e+");
    String s = f.readString();
    if (f) {
        cJSON *root = cJSON_Parse(s.c_str());
        cJSON *doub = cJSON_GetObjectItem(root, "lastPos");
        cJSON_Delete(root);
        f.close();
        return cJSON_GetNumberValue(doub);
    }
    f.close();
    return 0;
}