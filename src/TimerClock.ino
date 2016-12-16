/**
 * Uses a p channel mosfet for high level switching
 * requires a NPN transistor with 10k Ohm resistor in front of NPN to reduce the 3v out of the MC to 1.6V
 * and 10k Ohm as well after NPN to VCC (source 10V)
 * PWM signal from MC ground with 10k Ohm resistor
 * alternative code for relays is commented out
 * relays are accessed through port expander
 *
 */

#include <Arduino.h>

/*
   Test the PWM function of digital PINs
 */
#include <Streaming.h>
#include <DHT.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <Timezone.h>
#include <ArduinoOTA.h>
#include <Constants.h>
#include <FS.h>
#include <Wire.h>
#include <pcf8574a.h>
//#include <LiquidCrystal_I2C.h>
#include <RH_ASK.h>
#include <BME280.h>
#include <Adafruit_Sensor.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
//LiquidCrystal_I2C lcd(0x27, 20, 4);
//pcf8574a mcp(0x38); //instance

//temparure sensor
BME280 bme;                   // Default : forced mode, standby time = 1000 ms
                              // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
/**
 * @brief mDNS and OTA Constants
 * @{
 */
#define HOSTNAME "ESP8266-OTA-" ///< Hostename. The setup function adds the Chip ID at the end.
/// @}
/**
 * @brief Default WiFi connection information.
 * @{
 */
const char* ap_default_ssid = "esp8266"; ///< Default SSID.
const char* ap_default_psk = "esp8266esp8266"; ///< Default PSK.
/// @}

/// Uncomment the next line for verbose output over UART.
//#define SERIAL_VERBOSE
/**
 * @brief Read WiFi connection information from file system.
 * @param ssid String pointer for storing SSID.
 * @param pass String pointer for storing PSK.
 * @return True or False.
 *
 * The config file have to containt the WiFi SSID in the first line
 * and the WiFi PSK in the second line.
 * Line seperator can be \r\n (CR LF) \r or \n.
 */

ESP8266WebServer server ( 80 );

unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

//time zone setting
//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);

#define DHTPIN D9
//#define RFPIN D8
//RH_ASK rf_driver(2000, D8);  //will initialise the driver at 2000 bps, recieve on GPIO2, transmit on GPIO4, PTT on GPIO5
#define DHTTYPE DHT22 //DHT11, DHT21, DHT22
DHT dht(DHTPIN, DHTTYPE);

#define SCLPIN D3
#define SDAPIN D4

#define LEDPIN D6
#define GATEPIN D7
//#define POTIPIN A0  // no poti anymore
#define VOLTAGESENSORPIN A0

#define I2CBTNAUTO 5
#define I2CBTNUP  6
#define I2CBTNDOWN  7
#define BTNUP D2
#define BTNDOWN D5
#define BTNAUTO D10
#define RELAY0 0
#define RELAY1 1
#define RELAY2 2
#define RELAY3 3

#define HIGHLEVEL  2
#define LOWLEVEL  1
#define HIGHHOUR  8
#define HIGHMINUTE 0
#define LOWHOUR  8
#define LOWMINUTE 0

#define TIMER_ALARM 2
#define TIMER_MANUAL 1
#define TIMER_DEFAULT 0
#define TIMER_OFFLINE 3
#define TIMER_PUSHUP 4

#define ALARM_STARTHOUR 0
#define ALARM_STARTMIN 1
#define ALARM_AY 2
#define ALARM_LEVEL 3
#define ALARM_ENABLED 4
#define ALARM_ID 5

//#define LEDAUTOPIN D8
//#define LEDMANPIN D9

int stateUp = HIGH;      // the current state of the output pin
int readingUp;           // the current reading from the input pin
int previousAuto;
int readingAuto;
int previousUp = LOW;    // the previous reading from the input pin
int stateDown = HIGH;      // the current state of the output pin
int readingDown;           // the current reading from the input pin
int previousDown = LOW;    // the previous reading from the input pin

int curFanLevel = 0;
int fanLevel[] = {0, 20, 30, 40, 50, 60, 70, 80, 90,100};
int maxFanLevel = 10;
int timerMode = TIMER_DEFAULT;        // 0 default, 1 manual, 2 alarm mode
String timerModes[] ={"default", "manual", "alarm mode", "offline", "push up"};
int alarm1FanLevel = 3;
int alarm2FanLevel = 2;
int alarm3FanLevel = 3;

int alarms[10][6] = {
        { 7, 30, 0, 2, 1, 0 },                 //start hour, start min, daily, level, enabled, id
        { 8, 30, 0, 1, 1, 0 },
        { 11, 30, 0, 1, 1, 0 },
        { 12, 0, 0, 1, 1, 0 },
        { 17, 30, 0, 1, 1, 0 },
        { 18, 30, 0, 1, 1, 0 }
};
// int alarms[10][6] = {
//         { 7, 30, 0, 0, 1, 0 },                 //start hour, start min, daily, level, enabled, id
//         { 19, 00, 0, 3, 1, 0 },
// };

bool enableDht = true;
bool enableBme = false;
bool enableVoltagesensor = false;

int ledState = LOW;             // ledState used to set the LED
unsigned long ledPreviousMillis = 0;        // will store last time LED was updated
long ledInterval = 1000;

time_t getTime() {
        return CE.toLocal(getNtpTime());      //adjust utc to local time
}

bool loadConfig(String *ssid, String *pass)
{
        // open file for reading.
        File configFile = SPIFFS.open("/cl_conf.txt", "r");
        if (!configFile)
        {
                Serial.println("Failed to open cl_conf.txt.");

                return false;
        }

        // Read content from config file.
        String content = configFile.readString();
        configFile.close();

        content.trim();

        // Check if ther is a second line available.
        int8_t pos = content.indexOf("\r\n");
        uint8_t le = 2;
        // check for linux and mac line ending.
        if (pos == -1)
        {
                le = 1;
                pos = content.indexOf("\n");
                if (pos == -1)
                {
                        pos = content.indexOf("\r");
                }
        }

        // If there is no second line: Some information is missing.
        if (pos == -1)
        {
                Serial.println("Infvalid content.");
                Serial.println(content);

                return false;
        }

        // Store SSID and PSK into string vars.
        *ssid = content.substring(0, pos);
        *pass = content.substring(pos + le);

        ssid->trim();
        pass->trim();

#ifdef SERIAL_VERBOSE
        Serial.println("----- file content -----");
        Serial.println(content);
        Serial.println("----- file content -----");
        Serial.println("ssid: " + *ssid);
        Serial.println("psk:  " + *pass);
#endif

        return true;
} // loadConfig
/**
 * @brief Save WiFi SSID and PSK to configuration file.
 * @param ssid SSID as string pointer.
 * @param pass PSK as string pointer,
 * @return True or False.
 */
bool saveConfig(String *ssid, String *pass)
{
        // Open config file for writing.
        File configFile = SPIFFS.open("/cl_conf.txt", "w");
        if (!configFile)
        {
                Serial.println("Failed to open cl_conf.txt for writing");

                return false;
        }

        // Save SSID and PSK.
        configFile.println(*ssid);
        configFile.println(*pass);

        configFile.close();

        return true;
} // saveConfig


void setupAlarms() {
        // delete all alarms
        for (int i = 0; i<10; i++) {
                Alarm.free(alarms[i][ALARM_ID]);
        };
        for (int i = 0; i<10; i++) {
                if (alarms[i][ALARM_ENABLED]) {
                        uint8_t id;
                        id = Alarm.alarmRepeat(alarms[i][ALARM_STARTHOUR], alarms[i][ALARM_STARTMIN], 0, alarmTrigger);
                        Serial << "alarms[i][ALARM_ID] = " << id << endl;
                        alarms[i][ALARM_ID] = id;

                }
        }
//Alarm.alarmRepeat(dowSaturday,8,30,30,alarm3); // 8:30:30 every Saturday
}

void setup() {

        pinMode(LEDPIN, OUTPUT);
        pinMode(GATEPIN, OUTPUT);
        pinMode(DHTPIN, INPUT);
        analogWriteFreq(1000); // pwm frequen1000 is default 10000 works best

        pinMode(BTNUP, INPUT);
        pinMode(BTNDOWN, INPUT);
        pinMode(BTNAUTO, INPUT);

        //pinMode(LEDAUTOPIN, OUTPUT);
        //pinMode(LEDMANPIN, OUTPUT);

        pinMode(VOLTAGESENSORPIN, INPUT);

        // set the pins for I2C
        Wire.begin();
        // mcp.begin();
        // mcp.gpioPinMode(RELAY0,OUTPUT);
        // mcp.gpioPinMode(RELAY1,OUTPUT);
        // mcp.gpioPinMode(RELAY2,OUTPUT);
        // mcp.gpioPinMode(RELAY3,OUTPUT);
        //
        // mcp.gpioPinMode(I2CBTNAUTO,INPUT);
        // mcp.gpioPinMode(I2CBTNDOWN,INPUT);
        // mcp.gpioPinMode(I2CBTNUP,INPUT);

// initialize the LCD
        //lcd.init();
        //lcd.home();
        //lcd.print("Hello, World!");

        String station_ssid = WIFI_SSID;
        String station_psk = WIFI_PASSWORD;

        Serial.begin(115200);

        delay(100);

        Serial.println("\r\n");
        Serial.print("Chip ID: 0x");
        Serial.println(ESP.getChipId(), HEX);

        // Set Hostname.
        String hostname(HOSTNAME);
        hostname += String(ESP.getChipId(), HEX);
        WiFi.hostname(hostname);

        // Print hostname.
        Serial.println("Hostname: " + hostname);
        //Serial.println(WiFi.hostname());


        // Initialize file system.
        if (!SPIFFS.begin())
        {
                Serial.println("Failed to mount file system");
                return;
        }

        // Load wifi connection information.
        if (!loadConfig(&station_ssid, &station_psk))
        {
                station_ssid = "";
                station_psk = "";

                Serial.println("No WiFi connection information available.");
        }

        // Check WiFi connection
        // ... check mode
        if (WiFi.getMode() != WIFI_STA)
        {
                WiFi.mode(WIFI_STA);
                delay(10);
        }

        // ... Compare file config with sdk config.
        if (WiFi.SSID() != station_ssid || WiFi.psk() != station_psk)
        {
                Serial.println("WiFi config changed.");

                // ... Try to connect to WiFi station.
                WiFi.begin(station_ssid.c_str(), station_psk.c_str());

                // ... Pritn new SSID
                Serial.print("new SSID: ");
                Serial.println(WiFi.SSID());

                // ... Uncomment this for debugging output.
                //WiFi.printDiag(Serial);
        }
        else
        {
                // ... Begin with sdk config.
                WiFi.begin();
        }

        Serial.println("Wait for WiFi connection.");

        // ... Give ESP 10 seconds to connect to station.
        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000)
        {
                Serial.write('.');
                //Serial.print(WiFi.status());
                delay(500);
        }
        Serial.println();

        // Check connection
        if(WiFi.status() == WL_CONNECTED)
        {
                // ... print IP Address
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
        }
        else
        {
                Serial.println("Can not connect to WiFi station. Go into AP mode.");

                // Go into software AP mode.
                WiFi.mode(WIFI_AP);

                delay(10);

                WiFi.softAP(ap_default_ssid, ap_default_psk);

                Serial.print("IP address: ");
                Serial.println(WiFi.softAPIP());
        }

        // Start OTA server.
        ArduinoOTA.setHostname((const char *)hostname.c_str());
        ArduinoOTA.setPassword((const char *)"steffen");
        ArduinoOTA.begin();

        // Start the server
        server.on ( "/", handleRoot );
        server.on ( "/inline", [] () {
                            server.send ( 200, "text/plain", "this works as well" );
                    } );
        server.on ("/Fan=Up", handleFanUp);
        server.on ("/Fan=Down", handleFanDown);
        server.on ("/Fan=Auto", handleFanAuto);

        server.onNotFound ( handleNotFound );
        server.begin();
        Serial.println("Server started");

        // Print the IP address
        Serial.print("Use this URL to connect: ");
        Serial.print("http://");
        Serial.print(WiFi.localIP());
        Serial.println("/");

        Serial.println("Starting UDP");
        udp.begin(localPort);
        Serial.print("Local port: ");
        Serial.println(udp.localPort());

        if (enableDht) {
                dht.begin();
        }

        if (enableBme) {
                bme.begin();
        }

        //setSyncProvider((time_t)getNtpTime());
        setSyncProvider(getTime);
        //setTime((time_t)getNtpTime());
        //setTime(15, 39, 0, 9, 3, 2016);
        Serial.println(now());
        Serial << ("Zeit: ") << hour() << ":" << minute() << ":" << second() << endl;

        setupAlarms();

        // if (!rf_driver.init())
        //         Serial.println("init failed");
}
void LED_SimpleBlink(int LEDPin, long interval = 60, long duration = 5) {
        for (int i = 0; i < duration; i++) {
                analogWrite(LEDPin, 1023);
                delay(interval);
                analogWrite(LEDPin, 0);
                delay(interval);
        }
}
void loop() {

        // {
        //         uint8_t buf[12];
        //         uint8_t buflen = sizeof(buf);
        //         if (rf_driver.recv(buf, &buflen)) // Non-blocking
        //         {
        //                 int i;
        //                 // Message with a good checksum received, dump it.
        //                 Serial.print("Message: ");
        //                 Serial.println((char*)buf);
        //         } { Serial.println("no data received"); }
        // }

        // read the auto button
        //readingAuto = mcp.gpioDigitalRead(I2CBTNAUTO);
        readingAuto = digitalRead(BTNAUTO);
        //Serial << readingUp << endl;
        if (readingAuto == HIGH && previousAuto == LOW) {
                if ((timerMode == TIMER_ALARM) || (timerMode == TIMER_DEFAULT))  {
                        timerMode = TIMER_MANUAL;
                        Serial.println("Timer Manual");
                        //offline LED blink does not work somehow
                        //LED_SimpleBlink(LEDPIN, 600, 3);
                } else if (timerMode == TIMER_MANUAL)  {
                        timerMode = TIMER_OFFLINE;
                        Serial.println("Timer Offline");
                        //offline LED blink does not work somehow
                        //LED_SimpleBlink(LEDPIN, 3000, 3);
                } else {
                        timerMode = TIMER_ALARM;
                        Serial.println("Timer Alarm"); //goes to alarm mode on next alarm
                        //LED_SimpleBlink(LEDPIN, 60, 5);
                }
        } else if (readingAuto == LOW && previousAuto == HIGH) {
                stateDown = LOW;
        }
        previousAuto = readingAuto;

        //readingUp = mcp.gpioDigitalRead(I2CBTNUP);
        readingUp = digitalRead(BTNUP);
        //Serial << readingUp << endl;
        if (readingUp == HIGH && previousUp == LOW) {
                stateUp = HIGH;
                if (curFanLevel < maxFanLevel) {
                        curFanLevel++;
                }
                //timerMode = TIMER_MANUAL;
                //digitalWrite(LEDMANPIN, HIGH);
                //digitalWrite(LEDAUTOPIN, LOW);

                Serial.println("Button Up click");
        } else if (readingUp == LOW && previousUp == HIGH) {
                stateUp = LOW;
        }
        previousUp = readingUp;

        //readingDown = mcp.gpioDigitalRead(I2CBTNDOWN);
        readingDown = digitalRead(BTNDOWN);
        if (readingDown == HIGH && previousDown == LOW) {
                stateDown = HIGH;
                if (curFanLevel > 0) {
                        curFanLevel--;
                }
                //timerMode = TIMER_MANUAL;
                //digitalWrite(LEDMANPIN, HIGH);
                //digitalWrite(LEDAUTOPIN, LOW);
                Serial.println("Button Down click");
        } else if (readingDown == LOW && previousDown == HIGH) {
                stateDown = LOW;
        }
        previousDown = readingDown;

        //timer mode
        if (timerMode == TIMER_DEFAULT) {
                double nowTime = hour() * 100 + minute();
                if  ((nowTime >= (HIGHHOUR * 100 + HIGHMINUTE)) || (nowTime < (LOWHOUR * 100 + LOWMINUTE))) {
                        curFanLevel = HIGHLEVEL;
                        timerMode = TIMER_ALARM;
                } else {
                        curFanLevel = LOWLEVEL;
                        timerMode = TIMER_ALARM;
                }
        }

        // if (curFanLevel == 1) {
        //         mcp.gpioDigitalWrite(RELAY0,LOW);
        // } else {
        //         mcp.gpioDigitalWrite(RELAY0,HIGH);
        // }
        // if (curFanLevel == 2) {
        //         mcp.gpioDigitalWrite(RELAY1,LOW);
        // } else {
        //         mcp.gpioDigitalWrite(RELAY1,HIGH);
        // }
        // if (curFanLevel == 3) {
        //         mcp.gpioDigitalWrite(RELAY2,LOW);
        // } else {
        //         mcp.gpioDigitalWrite(RELAY2,HIGH);
        // }
        // if (curFanLevel == 4) {
        //         mcp.gpioDigitalWrite(RELAY3,LOW);
        // } else {
        //         mcp.gpioDigitalWrite(RELAY3,HIGH);
        // }

        //delay(200);
        //temp sensor

        //value = analogRead(POTIPIN);
        float value = 0;
        if (timerMode != TIMER_OFFLINE) {
          float value = (fanLevel[curFanLevel] * (1023.0 / 100.0)); //esp8266 digital out goes until 1024!!!
        }
        //analogWrite(LEDPIN, value);
        analogWrite(GATEPIN, value);
        //Serial << "timerMode: " << timerMode << " curFanLevel: " << curFanLevel << " Gate PWM Value: " << value << endl;

        //led blink
        if (timerMode == TIMER_MANUAL) {
                ledInterval = 500;
        } else if (timerMode == TIMER_OFFLINE) {
                ledInterval = -1;
        } else {
                ledInterval = 3000;
        }

        unsigned long ledCurrentMillis = millis();
        const unsigned long ledDuration = 10;
        if (ledInterval < 0) {
                ledState = LOW;
        } else {
                if (ledState == HIGH) {
                        if (ledCurrentMillis - ledPreviousMillis >= ledDuration) {
                                ledPreviousMillis = ledCurrentMillis;
                                ledState = LOW;
                        }
                } else if (ledState == LOW) {
                        if (ledCurrentMillis - ledPreviousMillis >= ledInterval) {
                                ledPreviousMillis = ledCurrentMillis;
                                ledState = HIGH;
                        }
                }
        }
        digitalWrite(LEDPIN, ledState);

        //webserver
        server.handleClient();
        ArduinoOTA.handle();
        // Set LEDPIN according to the request
        //digitalWrite(LEDPIN, value);
        Alarm.delay(1);

}

void handleFanUp() {
        if (curFanLevel < maxFanLevel) {
                curFanLevel++;
                //timerMode = TIMER_MANUAL;
                Serial << "fan up" << curFanLevel << endl;
        }
        handleRoot();
}
void handleFanDown() {
        if (curFanLevel > 0) {
                curFanLevel--;
                //timerMode = TIMER_MANUAL;
                Serial << "fan down" << curFanLevel << endl;
        }
        handleRoot();
}
void handleFanAuto() {
        if ((timerMode == TIMER_ALARM) || (timerMode == TIMER_DEFAULT))  {
                timerMode = TIMER_MANUAL;
                Serial.println("Timer Manual");
                //offline LED blink does not work somehow
                //LED_SimpleBlink(LEDPIN, 600, 3);
        } else if (timerMode == TIMER_MANUAL)  {
                timerMode = TIMER_OFFLINE;
                Serial.println("Timer Offline");
                //offline LED blink does not work somehow
                //LED_SimpleBlink(LEDPIN, 3000, 3);

        } else {
                timerMode = TIMER_ALARM;
                Serial.println("Timer Default"); //goes to alarm mode on next alarm
                //LED_SimpleBlink(LEDPIN, 60, 5);
        }
        //timerMode = TIMER_DEFAULT;
        //digitalWrite(LEDMANPIN, LOW);
        //digitalWrite(LEDAUTOPIN, HIGH);
        handleRoot();
}

void handleRoot() {

        char temp[600];
        int sec = millis() / 1000;
        int min = sec / 60;
        int hr = min / 60;
        //<meta http-equiv='refresh' content='5'/> //veranlasst den refresh nach 5s

        String answer = \
                "<html>\
  <head>\
    <title>Timer Control Vallux Ventilation</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1><a href=\"/\">Timer Control Vallux Ventilation</a></h1>";

        char buffer[24];
        //Serial << ("Zeit: ") << hour() << ":" << minute() << ":" << second() << endl;
        answer +="<p>";
        sprintf(buffer, "Time: %02d:%02d:%02d %02d.%02d.%04d", hour(), minute(), second(), day(), month(), year());
        answer += buffer;
        answer +="</p>";
        if (enableBme || enableDht) {
                float temperature(NAN), humidity(NAN), presssure(NAN);
                if (enableBme) {
                        bool metric = true;
                        uint8_t pressureUnit(1); // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
                        //bme.ReadData(presssure, temperature, humidity, metric, pressureUnit);          // Parameters: (float& pressure, float& temp, float& humidity, bool hPa = true, bool celsius = false)
                        // Alternatives to ReadData():
                        temperature = bme.ReadTemperature(true);
                        presssure = bme.ReadPressure(1);
                        humidity = bme.ReadHumidity();
                } else {

                        ////dht22 sensor
                        humidity = dht.readHumidity(); //Luftfeuchte auslesen
                        temperature = dht.readTemperature(); //Temperatur auslesen
                        presssure = 0.0;
                        // Prüfen ob eine gültige Zahl zurückgegeben wird. Wenn NaN (not a number) zurückgegeben wird, dann Fehler ausgeben.
                        if ((isnan(temperature) || isnan(humidity)) )
                        {
                                Serial.println("DHT22 konnte nicht ausgelesen werden");
                        }
                        else
                        {
                                //Serial << ("Luftfeuchte: ") << h << " Temperatur: " << t << " C" <<endl;
                        }
                }
                char str_temp[3];
                char str_temp2[3];
                char str_temp3[3];
                dtostrf(temperature, 2, 1, str_temp); // da %f nicht im arduino implementiert is
                dtostrf(humidity, 2, 0, str_temp2); // da %f nicht im arduino implementiert is
                dtostrf(presssure, 2, 0, str_temp3); // da %f nicht im arduino implementiert is
                char buffer[24];
                sprintf(buffer, "Temperature: %sC Humidity: %s%% Pressure: %shPa", str_temp, str_temp2, str_temp3);
                answer += "<p>";
                answer += buffer;
                answer += "</p>";
        }
        answer += "<p>Fanlevel: ";
        answer += curFanLevel;
        answer += " Timermode: ";
        answer += timerMode;
        answer += " " + timerModes[timerMode];
        answer += "</p>";
        if (enableVoltagesensor) {
                int samples = 20;
                double value = 0;
                for (int i = 0; i<samples; i++) {
                        double rawValue = analogRead(VOLTAGESENSORPIN);
                        value += rawValue;
                        delay(1);
                }
                float avgValue = value / samples;
                float voltage = avgValue * (12.0/1023.0);
                Serial << "Value: " << avgValue << " Voltage: " << voltage << endl;
                answer += "<p>Metered voltage: ";
                answer += voltage;
                answer += "V Voltage Value: ";
                answer += avgValue;
                answer += "</p>";
        }
        answer += "<p>Click <a href=\"/Fan=Up\">here</a> turn up the Fan</p>";
        answer += "<p>Click <a href=\"/Fan=Down\">here</a> turn down the Fan</p>";
        answer += "<p>Click <a href=\"/Fan=Auto\">here</a> to toggle auto/manual/offline Fan control</p>";
        answer += "<table>";
        answer += "<tr><th>ID</th><th>Start Time</th><th>Level</th><th>Enabled</th></tr>";
        for (int i = 0; i<10; i++) {
                if (alarms[i][ALARM_ENABLED] == 1) {
                        answer+=String("<tr>");
                        answer += String("<td>") + alarms[i][ALARM_ID] + String("</td>");
                        answer+=  String("<td>") + alarms[i][ALARM_STARTHOUR] + String(":");
                        answer+=  alarms[i][ALARM_STARTMIN] + String("</td>");
                        answer+=  String("<td>") + alarms[i][ALARM_LEVEL] + String("</td>");
                        answer+=  String("<td>") + alarms[i][ALARM_ENABLED] + String("</p></td>");
                        answer +=String("</tr>");
                }
        }
        answer += "</table>";
        answer += "<A HREF=\"javascript:history.go(0)\">Click to refresh the page</A>";
        answer += "</body></html>";
        server.sendHeader("Cache-Control", "no-cache");
        server.send ( 200, "text/html", answer );
        //digitalWrite ( led, 0 );
}

void handleNotFound() {
        //digitalWrite ( led, 1 );
        String message = "File Not Found\n\n";
        message += "URI: ";
        message += server.uri();
        message += "\nMethod: ";
        message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
        message += "\nArguments: ";
        message += server.args();
        message += "\n";

        for ( uint8_t i = 0; i < server.args(); i++ ) {
                message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
        }

        server.send ( 404, "text/plain", message );
        //digitalWrite ( led, 0 );
}
void alarmTrigger() {
        Serial << "alarm" << endl;
        for (int i = 0; i < 10; i++) {
                if (Alarm.getTriggeredAlarmId() == alarms[i][ALARM_ID]) {
                        if ((timerMode != TIMER_OFFLINE) || (timerMode != TIMER_MANUAL)) {
                                curFanLevel = alarms[i][ALARM_LEVEL];
                                timerMode = TIMER_ALARM;
                                return;
                        }
                }
        }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
        Serial.println("sending NTP packet...");
        // set all bytes in the buffer to 0
        memset(packetBuffer, 0, NTP_PACKET_SIZE);
        // Initialize values needed to form NTP request
        // (see URL above for details on the packets)
        packetBuffer[0] = 0b11100011; // LI, Version, Mode
        packetBuffer[1] = 0; // Stratum, or type of clock
        packetBuffer[2] = 6; // Polling Interval
        packetBuffer[3] = 0xEC; // Peer Clock Precision
        // 8 bytes of zero for Root Delay & Root Dispersion
        packetBuffer[12]  = 49;
        packetBuffer[13]  = 0x4E;
        packetBuffer[14]  = 49;
        packetBuffer[15]  = 52;

        // all NTP fields have been given values, now
        // you can send a packet requesting a timestamp:
        udp.beginPacket(address, 123); //NTP requests are to port 123
        udp.write(packetBuffer, NTP_PACKET_SIZE);
        udp.endPacket();
}

long int getNtpTime() {
        for (int i = 0; 3; i++) {
                //get a random server from the pool
                WiFi.hostByName(ntpServerName, timeServerIP);

                sendNTPpacket(timeServerIP); // send an NTP packet to a time server
                // wait to see if a reply is available
                delay(1000);

                int cb = udp.parsePacket();
                if (!cb) {
                        Serial.println("no packet yet");
                }
                else {
                        Serial.print("packet received, length=");
                        Serial.println(cb);
                        // We've received a packet, read the data from it
                        udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

                        //the timestamp starts at byte 40 of the received packet and is four bytes,
                        // or two words, long. First, esxtract the two words:

                        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
                        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
                        // combine the four bytes (two words) into a long integer
                        // this is NTP time (seconds since Jan 1 1900):
                        unsigned long secsSince1900 = highWord << 16 | lowWord;
                        Serial.print("Seconds since Jan 1 1900 = " );
                        Serial.println(secsSince1900);

                        // now convert NTP time into everyday time:
                        Serial.print("Unix time = ");
                        // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
                        const unsigned long seventyYears = 2208988800UL;
                        // subtract seventy years:
                        unsigned long epoch = secsSince1900 - seventyYears;
                        // print Unix time:
                        Serial.println(epoch);
                        return epoch;
                }
        }
}
// crashes this program but does work otherwhere
void LED_Blink(int LEDPin, long interval = 1000, long duration = 5) {
        int ledState = LOW;
        unsigned long previousMillis = 0;
        unsigned long startMillis = millis();
        unsigned long currentMillis = 0;
        while   ((millis() - startMillis) <= duration) {
                currentMillis = millis();
                if (currentMillis - previousMillis >= interval) {
                        // save the last time you blinked the LED
                        previousMillis = currentMillis;
                        // if the LED is off turn it on and vice-versa:
                        if (ledState == LOW) {
                                ledState = HIGH;
                        } else {
                                ledState = LOW;
                        }
                        // set the LED with the ledState of the variable:
                        digitalWrite(LEDPIN, ledState);
                }
        }
        //delay(10);
        yield();
}
