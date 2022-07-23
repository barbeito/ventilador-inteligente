/**
 * Programa: Controle de velocidade de um ventilador
 * Autores: Janderson Barbeito, Nikolas Fantoni e Wallefy Matheus
 * Instituição: Universidade Federal de Minas Gerais
 * Disciplina: Projeto de Sistemas Embutidos - 2022-1
 * 
*/
#define BLYNK_TEMPLATE_ID "TMPLpH1fF5D0"
#define BLYNK_DEVICE_NAME "smart ventilador"
#define BLYNK_AUTH_TOKEN "2nItflMm1Ps2PauAPCFhINJriLeQrDM_"

#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <IRremote.hpp>

// ====== definições =====================================
#define BLYNK_PRINT Serial

#define IR_PIN 5
#define RELE_1 26
#define RELE_2 25
#define RELE_3 33
#define LED_AZUL 18 // modo automatico
#define LED_VERD 19 // modo manual
#define DHT_SENSOR 27
#define DHTTYPE DHT11
#define SSID "2G_Janderson"
#define PASSWORD "cwkf@165243"
#define AUTOMATICO 1
#define MANUAL 0
#define TEMPO_DEBOUCE 10

typedef enum {
    DESLIGADO = 1,
    VELOCIDADE_1,
    VELOCIDADE_2,
    VELOCIDADE_3,
} ESTADO_VENTILADOR;

// ====== variáveis =====================================
char auth[] = BLYNK_AUTH_TOKEN;
BlynkTimer timer;
unsigned long delayBotao1;

volatile int modoOperacao = MANUAL;
ESTADO_VENTILADOR estadoVentilador = DESLIGADO;

// ====== funções =====================================
void conectaWifi();
void enviaTemperatura();
void modoAutomatico();
float lerTemperatura();
void definiModoOperacao();
void definiDesligar();
void definiVelocidade1();
void definiVelocidade2();
void definiVelocidade3();

// função de interrupção para controle do ventilador via controle remoto
void IRAM_ATTR controleRemotoIsr();

DHT dht_sensor(DHT_SENSOR, DHTTYPE);

SemaphoreHandle_t modoMutex;
SemaphoreHandle_t estadoMutex;

//======  Botões de ações  =======================================================================
//======  Definir modo     =======================================================================
BLYNK_WRITE(V6) {
  double pinValue = param.asDouble();
  definiModoOperacao(pinValue);
}

//======  Desligar         =======================================================================
BLYNK_WRITE(V1) {
  int pinValue = param.asInt();
  if(pinValue == 1) {
    definiDesligar();
  }
}

// //======  Velocidade 1     =======================================================================
BLYNK_WRITE(V2) {
  int pinValue = param.asInt();
  if(pinValue == 1) {
    definiVelocidade1();
  }
}

// //======  Velocidade 2     =======================================================================
BLYNK_WRITE(V3) {
  int pinValue = param.asInt();
  if(pinValue == 1) {
    definiVelocidade2();
  }
}

// //======  Velocidade 3     =======================================================================
BLYNK_WRITE(V4) {
  int pinValue = param.asInt();
  if(pinValue == 1) {
    definiVelocidade3();
  }
}

void setup()
{
  Serial.begin(115200);

  Serial.println("Tentando criar mutex...");
  modoMutex = xSemaphoreCreateMutex();
  while(modoMutex == NULL) {
    Serial.print(".");
    modoMutex = xSemaphoreCreateMutex();
  }
  estadoMutex = xSemaphoreCreateMutex();
  while(estadoMutex == NULL) {
    Serial.print(".");
    estadoMutex = xSemaphoreCreateMutex();
  }

  pinMode(DHT_SENSOR, INPUT);
  pinMode(RELE_1, OUTPUT);
  pinMode(RELE_2, OUTPUT);
  pinMode(RELE_3, OUTPUT);

  digitalWrite(RELE_1, LOW);
  // relés 2 e 3 são ativados em LOW e desativados em HIGH
  digitalWrite(RELE_2, HIGH);
  digitalWrite(RELE_3, HIGH);

  // leds indicadores do modo de operação:
  pinMode(LED_AZUL, OUTPUT);
  pinMode(LED_VERD, OUTPUT);
  digitalWrite(LED_AZUL, LOW);
  digitalWrite(LED_VERD, LOW);

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  attachInterrupt(digitalPinToInterrupt(IR_PIN), controleRemotoIsr, RISING);
  delayBotao1 = millis();

  conectaWifi();

  dht_sensor.begin();
  Blynk.begin(auth, SSID, PASSWORD);
  timer.setInterval(2000L, enviaTemperatura);
}

void loop()
{
  Blynk.run();
  timer.run();
  if(modoOperacao == AUTOMATICO) {
    digitalWrite(LED_VERD, LOW);
    digitalWrite(LED_AZUL, HIGH);
    modoAutomatico();
  }
  else {
    digitalWrite(LED_VERD, HIGH);
    digitalWrite(LED_AZUL, LOW);
  }
  float temp = lerTemperatura();
  Serial.println("Temperatura: " + String(temp));
}

float lerTemperatura() {
    float temp = dht_sensor.readTemperature();
    
    if(isnan(temp)) {
        return -1.0;
    }

    return temp;
}

void enviaTemperatura() {
    float valor = lerTemperatura();
    
    if(valor == -1.0) {
        return;
    }

    Blynk.virtualWrite(V5, valor);
}

void conectaWifi() {
    WiFi.begin(SSID, PASSWORD);

    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();

    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
}

void modoAutomatico() {
    // verifica valores da temperartur e para cada valor define a velocidade
    float temp = lerTemperatura();
    if(temp < 0) {
      return;
    }

    if(modoOperacao == AUTOMATICO && temp <= 25.0) {
      xSemaphoreTake(estadoMutex, portMAX_DELAY);
      definiVelocidadeMotor(VELOCIDADE_3);
      xSemaphoreGive(estadoMutex);
    }
    else if (modoOperacao == AUTOMATICO && temp > 25.0 && temp <= 28.0) {
      xSemaphoreTake(estadoMutex, portMAX_DELAY);
      definiVelocidadeMotor(VELOCIDADE_2);
      xSemaphoreGive(estadoMutex);
    }
    else if (modoOperacao == AUTOMATICO && temp > 28.0) {
      xSemaphoreTake(estadoMutex, portMAX_DELAY);
      definiVelocidadeMotor(VELOCIDADE_1);
      xSemaphoreGive(estadoMutex);
    }
}

// região crítica só chamar após dar lock no mutex
void definiVelocidadeMotor(ESTADO_VENTILADOR estado) {
    if(estado == DESLIGADO) {
        digitalWrite(RELE_1, LOW);
        digitalWrite(RELE_2, HIGH);
        digitalWrite(RELE_3, HIGH);
    } else if (estado == VELOCIDADE_1) {
        digitalWrite(RELE_2, HIGH);
        digitalWrite(RELE_3, HIGH);
        digitalWrite(RELE_1, HIGH);
    } else if (estado == VELOCIDADE_2) {
        digitalWrite(RELE_1, LOW);
        digitalWrite(RELE_3, HIGH);
        digitalWrite(RELE_2, LOW);
    } else if (estado == VELOCIDADE_3) {
        digitalWrite(RELE_1, LOW);
        digitalWrite(RELE_2, HIGH);
        digitalWrite(RELE_3, LOW);
    }
}

void definiModoOperacao(double pinValue) {
  xSemaphoreTake(modoMutex, portMAX_DELAY);
  if(!isnan(pinValue) && pinValue == 255.0) {
      modoOperacao = AUTOMATICO; // automático
  } else {
      modoOperacao = MANUAL;
  }
  xSemaphoreGive(modoMutex);
}

void definiDesligar() {
  xSemaphoreTake(estadoMutex, portMAX_DELAY);
  if(modoOperacao == MANUAL) {
    definiVelocidadeMotor(DESLIGADO);
  }
  xSemaphoreGive(estadoMutex);
}

void definiVelocidade1() {
  xSemaphoreTake(estadoMutex, portMAX_DELAY);
  if(modoOperacao == MANUAL) {
    definiVelocidadeMotor(VELOCIDADE_1);
  }
  xSemaphoreGive(estadoMutex);
}

void definiVelocidade2() {
  xSemaphoreTake(estadoMutex, portMAX_DELAY);
  if(modoOperacao == MANUAL) {
    definiVelocidadeMotor(VELOCIDADE_2);
  }
  xSemaphoreGive(estadoMutex);
}

void definiVelocidade3() {
  xSemaphoreTake(estadoMutex, portMAX_DELAY);
  if(modoOperacao == MANUAL) {
    definiVelocidadeMotor(VELOCIDADE_3);
  }
  xSemaphoreGive(estadoMutex);
}

void IRAM_ATTR controleRemotoIsr() {
  if(IrReceiver.decode()) {
    uint32_t codigoBotao = IrReceiver.decodedIRData.decodedRawData;
    if(codigoBotao == 0xB946FF00) {
      xSemaphoreTake(modoMutex, portMAX_DELAY);
      modoOperacao = !modoOperacao;
      xSemaphoreGive(modoMutex);
    }
    if(modoOperacao == MANUAL && codigoBotao == 0xE916FF00) {
      definiDesligar();
    }
    else if(modoOperacao == MANUAL && codigoBotao == 0xF30CFF00) {
      definiVelocidade1();
    }
    else if(modoOperacao == MANUAL && codigoBotao == 0xE718FF00) {
      definiVelocidade2();
    }
    else if(modoOperacao == MANUAL && codigoBotao == 0xA15EFF00) {
      definiVelocidade3();
    }
    IrReceiver.resume();
  }
}
