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
const unsigned long eventIntervalwifi_config = 1000;

unsigned long previousTime = 0;
unsigned long previousTime1 = 0;

// FUNCTIONS:

bool mqttInit()
{
  // Inicia WiFi com o SSID e a senha
  WiFi.begin(WIFISSID, PASSWORD);
  // Loop até que o WiFi esteja conectado
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(2000);
    lcd.home();
    lcd.println("connecting to ");
    lcd.setCursor(5, 1);
    lcd.print("WiFi..");
    delay(2000);
    lcd.clear();
  }
  lcd.clear();
  delay(2000);
  // Exibe no monitor serial
  lcd.home();
  lcd.println("Connected to");
  lcd.setCursor(5, 1);
  lcd.println("network");
  delay(2000);
  lcd.clear();
  // Seta servidor com o broker e a porta
  client.setServer(SERVER, PORT);
  // Conecta no ubidots com o Device id e o token, o password é informado como vazio
  while (!client.connect(DEVICE_ID, TOKEN, ""))
  {
    Serial.println("MQTT - Connect error");
    return false;
  }
  Serial.println("MQTT - Connect ok");
  return true;
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
    dataSendStates = false;
    return false;
  }

  // Se tudo der certo retorna true
  dataSendStates = true;
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
  if(dataSendStates){
  lcd.setCursor(15, 1);
  lcd.printByte(4);
  }

}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  lcd.init(); // initialize the lcd
  lcd.backlight();

  // Wifi seup
 // lcd.println("Setting up mqtt...");
  //delay(1000);
  //lcd.clear();
  // Inicializa mqtt (conecta o esp com o wifi, configura e conecta com o servidor da ubidots)
  if (!mqttInit())
  {
    delay(3000);

    lcd.println("Failed!");
    ESP.restart();
  }

  //lcd.println("OK");

 // lcd.clear();

  pinMode(AnalogPin, INPUT);
  pinMode(redLed, OUTPUT);
  pinMode(IRLed, OUTPUT);

  // turns off leds
  digitalWrite(redLed, LOW);
  digitalWrite(IRLed, LOW);
}

void loop()
{
  /*
    digitalWrite(redLed, HIGH);
    delay(20);
    red_ADC = analogRead(AnalogPin);
    digitalWrite(redLed, LOW);

    digitalWrite(IRLed, HIGH);
    delay(20);
    IR_ADC = analogRead(AnalogPin);
    digitalWrite(IRLed, LOW);

    Serial.print(red_ADC);
    Serial.print(",");
    Serial.println(IR_ADC);
  */

  // READING RED LED

  digitalWrite(redLed, HIGH);
  digitalWrite(IRLed, LOW);
  start = millis();
  n = 0;
  values = 0;
  do
  {
    values += analogRead(AnalogPin);
    n++;
  } while (millis() < start + period); // it runs this cycle for "period" milliseconds

  average = values / n; // average value filter for the graph and calculations
  arrayRed[cycle] = average;

  // READING IR LED

  digitalWrite(redLed, LOW);
  digitalWrite(IRLed, HIGH);
  start = millis();
  n = 0;
  values = 0;
  do
  {
    values += analogRead(AnalogPin);
    n++;
  } while (millis() < start + period); // it runs this cycle for "period" milliseconds

  average = values / n; // average value filter for the graph and calculations
  arrayIR[cycle] = average;

  if (cycle == 99)
  {
    finger = HIGH;
    /*
        if (finger_cycle == 1) {
          finger_detect = HIGH;
          finger_cycle = 0;
          Serial.print("RESET DETECT,");
        }

        if (finger_detect == LOW) {
          finger_cycle++;
          Serial.print("CYCLE INCREMENT,");
        }

        //FINGER IN
        if (finger_detect == HIGH) {
          Serial.println("entra finger high");
          for (i = 0; i <= arraySize; i++) {
            if (arrayRed[i] > 1020) {
              finger = HIGH;
              finger_detect = LOW;
              Serial.println("FINGER IN");
            }
          }
        }

        //FINGER OUT
        if (finger_detect == LOW) {
          Serial.println("entra finger low");
          for (i = 0; i <= arraySize; i++) {
            if (arrayRed[i] == 0) {
              finger = LOW;
              finger_detect = LOW;
              Serial.println("FINGER OUT");
            }
          }
          Serial.println("sai finger low");
        }
    */
    // BPM (beats per minute) && SPO2 calculation
    if (finger == HIGH)
    {
      // do {
      // AC_red:
      for (i = 0; i < arraySize; i++)
      {
        if (arrayRed[i] > arrayRed[i - 1])
          max_value_red = arrayRed[i];
      }

      // DC_red
      for (k = 0; k < arraySize; k++)
      {
        if (arrayRed[k] < arrayRed[k - 1])
          min_value_red = arrayRed[k];
      }
      DC_red = (max_value_red + min_value_red) / 2;
      AC_red = max_value_red - min_value_red;

      // AC_IR
      for (j = 0; j < arraySize; j++)
      {
        if (arrayIR[j] > arrayIR[j - 1])
          max_value_ir = arrayIR[j];
      }

      for (l = 0; l < arraySize; l++)
      {
        if (arrayIR[l] < arrayIR[l - 1])
          min_value_ir = arrayIR[l];
      }

      DC_ir = (max_value_ir + min_value_ir) / 2;
      AC_ir = max_value_ir - min_value_ir;

      // bpm calculation
      if (peak_detection_cycle == 1)
      {

        peak_max_ref = max_value_red;                              // highest sample peak detected
        peak_ref_top = peak_max_ref + peak_max_ref * peakError;    // calculates the top of peak range detection
        peak_ref_bottom = peak_max_ref - peak_max_ref * peakError; // calculates the bottom of peak range detection

        for (i = 0; i < arraySize; i++) // cleans the array
          arrayPeakValue[i] = 0;
        peakCounter = 0; // resets peak counter
        beats = 0;       // resets beats counter
      }
      if (peak_detection_cycle == 0)
      {

        for (i = 1; i <= arraySize; i++)
        {
          if (arrayRed[i] > arrayRed[i - 1])
          {
            rising = 1; // the signal is rising
          }
          if (arrayRed[i] < arrayRed[i - 1] && rising == 1) // if the signal starts dropping and it was rising before, we found a peak
          {
            arrayPeakValue[peakCounter] = arrayRed[i - 1]; // saves peak value
            arrayPeakPosition[peakCounter] = i - 1;        // saves peak position of the array
            peakCounter++;
            rising = 0;
          }
        }
        for (i = 0; i < arraySize; i++)
        {
          if ((arrayPeakValue[i] > peak_ref_bottom) && (arrayPeakValue[i] < peak_ref_top) && (beats < 2))
          { // if the value of the peak is withing the estimated range and it didn't register 2 beats
            beatsPosition[beats] = arrayPeakPosition[i];
            beats++;
          }
        }
        beatsPositionDiff = beatsPosition[1] - beatsPosition[0];
        beatConversionTime = beatsPositionDiff * 2 * period;
        // Serial.print("beatsPositionDiff: ");
        // Serial.println(beatsPositionDiff);
        // Serial.print("beatConversionTime: ");
        // Serial.println(beatConversionTime);
        bpm = (60 / beatConversionTime) * 1000;
      }

      // R = (AC_red / DC_red) / (AC_ir / DC_ir);
      // Spo2 = a - b * R;
      R = (AC_red * min_value_ir) / (AC_ir * min_value_red);
      // Spo2 = (10.0002 * pow(R, 3)) - (52.887 * pow(R, 3)) + (26.871 * R) + 98.283;
      Spo2 = -16.666666 * pow(R, 2) + 8.333333 * R + 100;
      //} while (millis() < start + bpm_time);
    }
    /*
        //Serial.print("min_value_red: ");
        Serial.print(min_value_red);
        Serial.print(",");
        //Serial.print("min_value_ir: ");
        Serial.print(min_value_ir);
        Serial.print(",");
        Serial.print(max_value_red);
        Serial.print(",");
        Serial.println(max_value_ir);
        //Serial.print("DC_red: ");
        //Serial.print(AC_red);
        //Serial.print(",");
        //Serial.print(AC_ir);
        //Serial.print(",");
        //Serial.print(DC_red);
        //Serial.print(",");
        //Serial.print("DC_ir: ");
        //Serial.println(DC_ir);
    */
    if (Spo2 >= 70 && Spo2 <= 100)
    {
      Serial.print("SPO2: ");
      Serial.println(Spo2);
    }

    // Serial.print("BPM: ");
    // Serial.print(",");
    // Serial.print(bpm);
    // Serial.print(",");
  }

  // PRINTING
  // Serial.print(arrayRed[cycle]);
  // Serial.print(",");
  // Serial.println(arrayIR[cycle]);
  /* Serial.print(",");
   Serial.println(R);

   Serial.print(",");
   Serial.println(cycle);*/
  // CYCLE COUNT
  cycle++;
  if (cycle == arraySize)
  {
    cycle = 0;
    if (peak_detection_cycle == 1)
      peak_detection_cycle = 0;
    else
      peak_detection_cycle = 1;
  }

  // Vo=outADC*5/1023;
  // Serial.print("Vo: ");
  // Serial.println(Vo);

  // delay(20);
  // digitalWrite(redLed, LOW);

  temperature = temperature_measure();

  // lcd
  lcdDisplay(89, Spo2, 12);

  // Serial.println("Acabou cálculo");
  // Serial.println();
  // delay(1500);

  // Se o esp foi desconectado do ubidots, tentamos reconectar
  if (!client.connected())
    reconnect();
  // Lê a temperatura e umidade e exibimos no display passando uma flag (que sinaliza sucesso na leitura)

  // Esperamos 2.5s antes de exibir o status do envio para dar efeito de pisca no display
  unsigned long currentTime = millis();

  /* This is the event  */
  if (currentTime - previousTime >= eventIntervalwifi)
  {
    if (sendValues(Spo2, 20.3))
    {
      Serial.println("Successfully sent data");
    }
    else
    {
      Serial.println("Failed to send sensor data");
    }
    previousTime = currentTime;
  }
}
