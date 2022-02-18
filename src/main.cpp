#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>

#include "data.h"
#include "Settings.h"
#include <UbidotsEsp32Mqtt.h>

#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s

WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

//*******Pantalla****/////
#include "TFT_eSPI.h"
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();

//*******Sensor de temperatura*****//
#include <DHT.h>
#include <string.h>

#define DHTPIN 27     // definicion del pin al que se conecta el sensor
#define DHTTYPE DHT11 // definir el tipo de dht

#define LED1Pin 2 // 32
#define LED2Pin 33

DHT dht(DHTPIN, DHTTYPE); // constructor

/****************************************
 * Define Constants
 ****************************************/
const char *UBIDOTS_TOKEN = "BBFF-j1TruHugbGjhYBFo9QxRdBRSET2lTo"; // Put here your Ubidots TOKEN
const char *DEVICE_LABEL = "esp32";                                // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "Temperatura";                        // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL2 = "Humedad";                           // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL3 = "sw1";                               //  Replace with your variable label to subscribe to
const char *VARIABLE_LABEL4 = "sw2";                               //  Replace with your variable label to subscribe to
const char *VARIABLE_LABEL5 = "ConfSW1";                           //  Replace with your variable label to subscribe to
const char *VARIABLE_LABEL6 = "ConfSW2";                           //  Replace with your variable label to subscribe to
const char *CUSTOM_TOPIC = "/v2.0/devices/demo/+";                 // This will subscribe to all the messages received by the Device labeled as "demo"

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

unsigned long timer;
// uint8_t analogPin = 33; // Pin used to read data from GPIO34 ADC_CH6.

int estado_sw1 = 0;
int estado_sw2 = 0, confsw1 = 0, confsw2 = 0;

char str1[] = "/v2.0/devices/esp32/sw1/lv";
// strcpy(str1,);

char *buff = "this is a test string";

Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
 * Auxiliar Functions
 ****************************************/

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  buff = topic;
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    // Serial.print((char)payload[i]);
    if ((char)payload[0] == '1')
    {
      // digitalWrite(LED, HIGH);
      if (strcmp(str1, topic) == 0)
      {
        estado_sw1 = 1;
        confsw1 = 1;
      }
      else
      {
        estado_sw2 = 1;
        confsw2 = 1;
      }
    }
    else
    {
      // digitalWrite(LED, LOW);
      // estado_sw1=0;
      if (strcmp(str1, topic) == 0)
      {
        estado_sw1 = 0;
        confsw1 = 0;
      }
      else
      {
        estado_sw2 = 0;
        confsw2 = 0;
      }
    }
  }
  Serial.println();
}

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("Camilo_AP", "facil123");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();
  // Rutinas de Ubidots
  // Sensor init
  dht.begin();
  pinMode(LED1Pin, OUTPUT);
  pinMode(LED2Pin, OUTPUT);

  // Pantalla init
  tft.init();

  // Código de pantalla de carga
  // Con esta función se rota la pantalla
  tft.setRotation(1);
  // Con esta función cambio el color del fondo de la pantalla
  tft.fillScreen(TFT_BLACK);
  // Código para imprimir un texto en la pantalla, el primer argumento es la posición x, el segundo es la posición y, el tercero es el tamaño de la fuente
  tft.setTextColor(TFT_BLUE);
  tft.drawString("TRABAJO 2", 50, 50, 4); // X, Y, FONT
  // tft.drawString("Hola Mundo",10,120,2);//EL 2 es la fuente
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Connecting to server...", 48, 70, 2); // X, Y, FONT

  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL3); // Insert the dataSource and Variable's Labels
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL4); // Insert the dataSource and Variable's Labels
  // ubidots.subscribe(CUSTOM_TOPIC); // Insert the topic to subscribe to

  timer = millis();
}

void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo

  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  if (is_STA_mode())
  {
    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    delay(1000);                     // reatrdo de 2seg
    float h = dht.readHumidity();    // lee la humedad en fnc read y la almacena en h
    float t = dht.readTemperature(); // lee la temperatura en fnc read y la almacena en t
    if (isnan(h) || isnan(t))
    {
      Serial.println(F("Failed to read from DHT sensor!")); // isnan nos devuelve un 1 en caso de ue exista un fallo o un error en la lectura de la vble

      tft.fillScreen(TFT_BLACK);
      tft.drawString("Failed to read from DHT sensor", 20, 50, 2); // X, Y, FONT

      return;
    }

    // tft.drawString(String(i), 30, 50, 7);
    // i++;
    // delay(100);

    if (!ubidots.connected())
    {
      ubidots.reconnect();
      ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL3); // Insert the device and variable's Labels, respectively
      ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL4); // Insert the device and variable's Labels, respectively
    }
    if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
      // float value = analogRead(analogPin);
      // Serial.print(value);
      // Serial.print();
      // ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL); // Insert the dataSource and Variable's Labels

      ubidots.add(VARIABLE_LABEL, t); // Insert your variable Labels and the value to be sent
      ubidots.add(VARIABLE_LABEL2, h);
      ubidots.publish(DEVICE_LABEL);
      ubidots.add(VARIABLE_LABEL5, confsw1); // Insert your variable Labels and the value to be sent
      ubidots.add(VARIABLE_LABEL6, confsw2);
      timer = millis();
    }
    ubidots.loop();

    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_CYAN);
    tft.drawString("Humedad (%):", 5, 0, 2); // X, Y, FONT
    tft.drawString(String(h), 90, 15, 7);

    tft.setTextColor(TFT_RED);
    tft.drawString("Temperatura(°C):", 5, 65, 2); // X, Y, FONT
    tft.drawString(String(t), 90, 80, 7);

    // Switch 1
    if (estado_sw1 == 1)
    {
      digitalWrite(LED1Pin, HIGH);
      tft.fillCircle(20, 110, 10, TFT_YELLOW); //(X,Y,radio,color)
    }
    else
    {
      digitalWrite(LED1Pin, LOW);
      tft.fillCircle(20, 110, 10, TFT_DARKGREY); //(X,Y,radio,color)
    }

    // Switch 2
    if (estado_sw2 == 1)
    {
      digitalWrite(LED2Pin, HIGH);
      tft.fillCircle(50, 110, 10, TFT_GREEN); //(X,Y,radio,color)
    }
    else
    {
      digitalWrite(LED2Pin, LOW);
      tft.fillCircle(50, 110, 10, TFT_DARKGREY); //(X,Y,radio,color)
    }
    // tft.fillCircle(50,110,10,TFT_DARKGREY);//(X,Y,radio,color)
    // fillCircle(50,50, 10, TFT_BLACK);
  }
  else // rutina para AP + WebServer
    server.handleClient();

  delay(10);
  detect_long_press();
}

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}