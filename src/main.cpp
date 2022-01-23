#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>

#define maxperiod_siz 80 // max number of samples in a period
#define measures 10      // number of periods stored
#define samp_siz 4       // number of samples for average
#define rise_threshold 3 // number of rising measures to determine a peak

//  WIFI VARIABLES
#define WIFISSID " Phone"
#define PASSWORD "12345678901"
#define TOKEN "BBFF-EpKOwNyeMCpbvlLzLzsq8YAyikyym2"
#define VARIABLE_LABEL_TEMPERATURE "temperature"
#define VARIABLE_LABEL_Spo2 "Spo2"
#define VARIABLE_LABEL_Bpm "bpm"
#define DEVICE_ID "esp32"
#define SERVER "things.ubidots.com" // Servidor do Ubidots (broker)
#define PORT 1883
// Tópico aonde serão feitos os publish, "esp32-dht" é o DEVICE_LABEL
#define TOPIC "/v1.6/devices/esp32-spo2"
/************** end **********/

// SPO2 VARIABLES
#define AnalogPin 34
#define redLed 13
#define IRLed 14
#define arraySize 100 // total number of samples
#define sampleSize 10 // size of samples to calculate period of signal
#define peakError 3   // 10% peak value error for bpm calculation
#define period 10     // amount of milliseconds to read the sensor

float start, values, average;
float arrayRed[arraySize], arrayIR[arraySize];
float peak_max_ref = 0, peak_ref_top = 0, peak_ref_bottom = 0;
float arrayPeakValue[arraySize], arrayPeakPosition[arraySize];
float beatsPosition[2], beatConversionTime = 0;
int n, rising = 0, cycle = 0, finger_cycle = 0;
int finger_print = 0, peak_detection_cycle = 1;
int peakCounter = 0, beats = 0, beatsPositionDiff = 0;
boolean finger = LOW, finger_detect = LOW;

float AC_red = 0;
float DC_red = 0;
float AC_ir = 0;
float DC_ir = 0;

float max_value_red = 0;
float max_value_ir = 0;
int i = 0;
int j = 0;
float min_value_red = 0;
float min_value_ir = 0;
int k = 0;
int l = 0;

float a = 0;
float b = 0;
float R = 0;
float Spo2 = 0;

float bpm = 0;
/******************end********************/

// NTC VARIABLES
float R1 = 10000;
float R3 = 1400;
float R4 = 3000;
float R5 = 12000;
float R6 = 18000;
float Vy = 3.409;

float Vcc = 5;
float R_NTC = 0;
float red_ADC = 0;
float IR_ADC = 0;
float Vo = 0;
float Ro = 10000;
float To = 25 + 273.15;
float T_K = 0;
float T_C = 0;
float B = 4300;
float temperature = 0;

/***********end**********/

WiFiClient ubidots;
// Objeto PubSubClient usado para publish–subscribe
PubSubClient client(ubidots);

// lcd character animation
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args) write(args);
#else
#define printByte(args) print(args, BYTE);
#endif
boolean lcdstate = true;
boolean dataSendStates = false;
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
byte termometer[] = {
    0x00,
    0x0F,
    0x0A,
    0x0B,
    0x0A,
    0x0E,
    0x0E,
    0x1F};

byte celsios[] = {
    0x00,
    0x0E,
    0x0A,
    0x0E,
    0x00,
    0x00,
    0x00,
    0x00};

byte wifi[] = {
    B00000,
    B00000,
    B00000,
    B00111,
    B01000,
    B10011,
    B10100,
    B10101};

// a liquid crystal displays BPM
LiquidCrystal_I2C lcd(0x27, 16, 2);
const unsigned long eventInterval = 1000;
const unsigned long eventIntervalwifi = 1000;


unsigned long previousTime = 0;
unsigned long previousTime1 = 0;

// FUNCTIONS:

bool mqttInit()
{
  // Inicia WiFi com o SSID e a senha
  WiFi.begin(WIFISSID, PASSWORD);
  // Loop até que o WiFi esteja conectado

  // Seta servidor com o broker e a porta
  client.setServer(SERVER, PORT);
}

void reconnect()
{
  // Loop até que o MQTT esteja conectado
  while (!client.connected())
  {
    // sinaliza desconexão do mqtt no display

    lcd.println(" MQTT connection...");
    delay(100);
    lcd.clear();
    // Tenta conectar
    if (client.connect(DEVICE_ID, TOKEN, ""))
      Serial.println("connected");
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Aguarda 2 segundos antes de retomar
      delay(2000);
    }
  }
}

bool sendValues(float temperature, float spo2)
{
  char json[250];
  // Atribui para a cadeia de caracteres "json" os valores referentes a temperatura e os envia para avariável do ubidots correspondente
  sprintf(json, "{\"%s\":{\"value\":%02.02f, \"context\":{\"temperature\":%02.02f,\"spo2\":%02.02f}}}", VARIABLE_LABEL_TEMPERATURE, temperature, temperature, spo2);
  if (!client.publish(TOPIC, json))
    return false;
  // Atribui para a cadeia de caracteres "json" os valores referentes a umidade e os envia para avariável do ubidots correspondente
  sprintf(json, "{\"%s\":{\"value\":%02.02f, \"context\":{\"temperature\":%02.02f,\"spo2\":%02.02f}}}", VARIABLE_LABEL_Spo2, spo2, temperature, spo2);
  if (!client.publish(TOPIC, json))
  {

    return false;
  }

  // Se tudo der certo retorna true

  return true;
}

float temperature_measure()
{
  R_NTC = (float)R1 * (Vcc * (1 + R6 / R5) / (Vo + R6 * Vy / R5) - 1);
  // Serial.print("R_NTC: ");
  // Serial.println(R_NTC);

  // Modelo Beta
  T_K = (float)1 / (1 / B * log(R_NTC / Ro) + 1 / To);
  T_C = T_K - 273.15;
  // Serial.print("T_NTC: ");
  // Serial.println(T_C);
  return T_C;
}

void lcdDisplay(int bpm, int spo2, int temp)
{
  unsigned long currentTime = millis();

  lcd.createChar(0, heart);
  lcd.createChar(1, celsios);
  lcd.createChar(3, termometer);
  lcd.createChar(4, wifi);
  /* This is the event */
  if (currentTime - previousTime >= eventInterval)
  {
    /* Event code */
    if (lcdstate == true)
    {
      lcd.setCursor(1, 0);
      lcd.printByte(0);
    }
    else
    {
      lcd.setCursor(1, 0);
      lcd.print(" ");
    }
    lcdstate = !lcdstate;
    /* Update the timing for the next time around */
    previousTime = currentTime;
  }
  lcd.print(" ");
  lcd.print(bpm);
  lcd.print("/bpm");
  lcd.setCursor(10, 0);
  lcd.print(spo2);
  lcd.print("%Spo");
  lcd.setCursor(0, 1);
  lcd.print(" ");
  lcd.printByte(3);
  lcd.print(" ");
  lcd.print(temp);
  lcd.printByte(1);
  lcd.print("C");
  if (dataSendStates)
  {
    lcd.setCursor(15, 1);
    lcd.printByte(4);
  }
  else
  {
    lcd.setCursor(15, 1);
    lcd.print(" ");
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  lcd.init(); // initialize the lcd
  lcd.backlight();
  // wifi init
  mqttInit();

  pinMode(AnalogPin, INPUT);
  pinMode(redLed, OUTPUT);
  pinMode(IRLed, OUTPUT);

  // turns off leds
  digitalWrite(redLed, LOW);
  digitalWrite(IRLed, LOW);
}

void loop()
{
  unsigned long currentTime = millis();
  if (currentTime - previousTime1 >= eventIntervalwifi)
  {
    if (WiFi.status() == WL_CONNECTED && client.connect(DEVICE_ID, TOKEN, ""))
    {
      dataSendStates = true;
      sendValues(60, 20.3);
    }
    else
    {
      dataSendStates = false;
    }

    previousTime1 = currentTime;
  }

  lcdDisplay(10, 20, 30);

  /* This is the event*/
}
