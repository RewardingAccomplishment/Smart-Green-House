#include <WiFi.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h>

#include "time.h"
#include "sntp.h"

const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)


#define DHT_INDOORS_PIN       18     // Digital pin connected to the DHT sensor
#define DHT_OUTDOORS_PIN       5     // Digital pin connected to the DHT sensor
#define DHTTYPE            DHT22     // DHT 22 (AM2302)
DHT_Unified dhtIndoors(DHT_INDOORS_PIN, DHTTYPE);
DHT_Unified dhtOutdoors(DHT_OUTDOORS_PIN, DHTTYPE);
hw_timer_t * dhtTimer = NULL;

#define DHT_TIM_NUM     1
#define DHT_TIM_DIV     80
#define DHT_TIM_COUNTUP true
#define DHT_TIMEOUT_S   5*60*1000000 // set tp 5 mins

#define MAX_RECORD  12*24   // 12 probes /h

static String timestamp[MAX_RECORD];    //= {"Moday", "Tuesday", "Friday", "Thursday"};
static String tempIndoors[MAX_RECORD];  //= {"1.0", "2.1", "3.2", "4.5"};
static String humiIndoors[MAX_RECORD];  //= {"2.0", "7.1", "2.2", "3.2"};;
static String tempOutdoors[MAX_RECORD]; //= {"3.1", "8.1", "1.2", "2.2"};
static String humiOutdoors[MAX_RECORD]; //= {"4.2", "9.1", "8.2", "6.5"};
static int record_idx = 0;

// Replace with your network credentials
const char* ssid = /*YOUR SSID*/;
const char* password = /* YOUR PWD*/;

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String relayM3State = "off";
String relayM4State = "off";
String relayM1State = "CLOSE";
String relayM1StateRedundant ="CLOSE";
String relayM2State = "off";

// Assign output variables to GPIO pins
const int relayM3 = 27; //M3
const int relayM4 = 26; //M4
const int relayM1 = 12; //M1
const int relayM2 = 14; //M2

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

unsigned long meas_currentTime = millis();

RTC_Millis RTC;

#define DOOR_IDLE      (0U)
#define DOOR_OPEN      (1U)
#define DOOR_CLOSE     (1U << 1U)
#define DOOR_BUSY      (1U << 2U)
#define DOOR_ERROR     (1U << 3U)

uint8_t doorState = DOOR_IDLE;

hw_timer_t * doorTimer = NULL;
#define DOOR_CLOSE_DURATION_US 60*1000000

#define TIM_NUM     0
#define TIM_DIV     80
#define TIM_COUNTUP true

void motorRotateLeft()
{
  digitalWrite(relayM1, LOW);
  digitalWrite(relayM2, HIGH);
}

void motorRotateRight()
{
  digitalWrite(relayM1, HIGH);
  digitalWrite(relayM2, LOW);
}

void motorTimeToFinish()
{
  if (!(doorState & DOOR_BUSY))
  {
    timerAlarmEnable(doorTimer);
  }
}

void motorStop()
{
  timerAlarmDisable(doorTimer);
  digitalWrite(relayM1, HIGH);
  digitalWrite(relayM2, HIGH);
  doorState &= (~DOOR_BUSY);
}

bool doorOpen()
{
  bool ret = true;

  if ((doorState != DOOR_BUSY) || (doorState == DOOR_IDLE))
  {
    motorRotateLeft();
    motorTimeToFinish();
    doorState = (DOOR_OPEN | DOOR_BUSY);
    ret = false;
  }

  return ret;
}

bool doorClose()
{
  bool ret = true;

  if ((doorState != DOOR_BUSY) || (doorState == DOOR_IDLE))
  {
    motorRotateRight();
    motorTimeToFinish();
    doorState = (DOOR_CLOSE | DOOR_BUSY);
    ret = false;
  }

  return ret;
}

void IRAM_ATTR doorTimerTimeout(){
  motorStop();
}

void doorTimerInit()
{
  doorTimer = timerBegin(TIM_NUM, TIM_DIV, TIM_COUNTUP);
  timerAttachInterrupt(doorTimer, &doorTimerTimeout, true);
  timerAlarmWrite(doorTimer, DOOR_CLOSE_DURATION_US, true);
}


bool dhtGetMeas(DHT_Unified *dht, String *current_humidity, String *current_temp)
{
  sensors_event_t event;
  dht->temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
    return true;
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    *current_temp=String(event.temperature);
    Serial.println(F("°C"));
  }
  dht->humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
    return true;
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    *current_humidity = String(event.relative_humidity);
    Serial.println(F("%"));
  }

  return false;
}

bool doMeas=false;
void IRAM_ATTR dhtTimerTimeout()
{
  doMeas=true;
}

void dhtTimerInit()
{
  dhtTimer = timerBegin(DHT_TIM_NUM, DHT_TIM_DIV, DHT_TIM_COUNTUP);
  timerAttachInterrupt(dhtTimer, &dhtTimerTimeout, true);
  timerAlarmWrite(dhtTimer, DHT_TIMEOUT_S, true);
  timerAlarmEnable(dhtTimer);

}

void dhtSetup(DHT_Unified *dht){
  dht->begin();
   Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht->temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.print  (F("------------------------------------"));
  // Print humidity sensor details.
  dht->humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));

}

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  // RTC.begin(DateTime(__DATE__, __TIME__));

}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}

void initTimeWithSntp()
{
  // set notification call-back function
  sntp_set_time_sync_notification_cb( timeavailable );

  /**
   * NTP server address could be aquired via DHCP,
   *
   * NOTE: This call should be made BEFORE esp32 aquires IP address via DHCP,
   * otherwise SNTP option 42 would be rejected by default.
   * NOTE: configTime() function call if made AFTER DHCP-client run
   * will OVERRIDE aquired NTP server address
   */
  sntp_servermode_dhcp(1);    // (optional)

  /**
   * This will set configured ntp servers and constant TimeZone/daylightOffset
   * should be OK if your time zone does not need to adjust daylightOffset twice a year,
   * in such a case time adjustment won't be handled automagicaly.
   */
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  /**
   * A more convenient approach to handle TimeZones with daylightOffset
   * would be to specify a environmnet variable with TimeZone definition including daylight adjustmnet rules.
   * A list of rules for your zone could be obtained from https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h
   */
  //configTzTime(time_zone, ntpServer1, ntpServer2);
}

void setup() {
  // following line sets the RTC to the date & time this sketch was compiled
  // RTC.begin(DateTime(__DATE__, __TIME__));
  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(relayM3, OUTPUT);
  pinMode(relayM4, OUTPUT);
  pinMode(relayM1, OUTPUT);
  pinMode(relayM2, OUTPUT);
  // Set outputs to LOW
  digitalWrite(relayM3, LOW);
  digitalWrite(relayM4, LOW);
  digitalWrite(relayM1, LOW);
  digitalWrite(relayM2, LOW);

  initTimeWithSntp();

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

  dhtSetup(&dhtIndoors);
  dhtSetup(&dhtOutdoors);

  doorTimerInit();
  dhtTimerInit();
}

void tempHumiMeas()
{
  meas_currentTime = millis();
  String current_temp;
  String current_humidity;
  current_temp.clear();
  current_humidity.clear();

  if(dhtGetMeas(&dhtIndoors, &current_humidity, &current_temp))
  {
    Serial.println(F("Error getting Indoors meas!"));
  }
  tempIndoors[record_idx] = current_temp;
  humiIndoors[record_idx] = current_humidity;

  current_temp.clear();
  current_humidity.clear();
  if(dhtGetMeas(&dhtOutdoors, &current_humidity, &current_temp))
  {
    Serial.println(F("Error getting Outdoors meas!"));
  }
  tempOutdoors[record_idx] = current_temp;
  humiOutdoors[record_idx] = current_humidity;

  Serial.println("");

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
  } else {
    char timeStringBuff[50]; //50 chars should be enough
    strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
    String asString(timeStringBuff);
    timestamp[record_idx] = timeStringBuff;
  }

  if (record_idx < MAX_RECORD){
    record_idx++;
  }else
  {
    record_idx = 0;
  }
}

void displayWebBody(WiFiClient * const client)
{
    client->println("<body>");
    displayWebHeadings(client);
    displayWebButton(client);
    displayWebPlots(client);
    displayWebFooter(client);
    client->println("<\body>");

}

void displayWebPage(WiFiClient * const client)
{
    displayWebHead(client);
    displayWebBody(client);
    client->println("<\html>");
    // The HTTP response ends with another blank line
    client->println();
}


void displayHttpOK(WiFiClient * const client)
{
  // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
  // and a content-type so the client knows what's coming, then a blank line:
  client->println("HTTP/1.1 200 OK");
  client->println("Content-type:text/html");
  client->println("Connection: close");
  client->println();
}

void displayWebHead(WiFiClient * const client)
{
  client->println("<head>  \
  <meta charset=\"utf-8\">  \
  <title>Temperature and Humidity Plot</title>  \
  <script src=\"https://cdn.plot.ly/plotly-latest.min.js\"></script>  \
  <style>  \
    html {  \
      font-family: Helvetica, Arial, sans-serif;  \
      display: block;  \
      margin: 0px auto;  \
      text-align: center;  \
    }  \
    .button {  \
      background-color: #4CAF50;  \
      border: none;  \
      color: white;  \
      padding: 16px 40px;  \
      text-decoration: none;  \
      font-size: 30px;  \
      margin: 2px;  \
      cursor: pointer;  \
    }  \
    .button2 {  \
      background-color: #555555;  \
    }  \
  </style>  \
</head>");
}

void displayWebHeadings(WiFiClient * const client)
{
  // Web Page Heading
  client->println("<h1>My Great Greenhouse Monitoring System</h1> \
  <h2>Dedicated to my wife</h2>");

  static int time_to_ovf_s = (MAX_RECORD - record_idx)*(DHT_TIMEOUT_S/1000000);
  client->println("<h3>Done "+ String(record_idx) \
            +" meas and remain time "+ String(time_to_ovf_s/3600) + ":" \
            + String((time_to_ovf_s%3600)/60) + ":" + String(time_to_ovf_s%60) \
            + " to overflow. </h3>");

  if(record_idx>0)
  {
    client->println("<h1>Indoors Temperature: " + tempIndoors[record_idx - 1] +"&deg;C </h1>");
    client->println("<h1>Indoors Humidity: " + humiIndoors[record_idx - 1] + "% </h1>");
    client->println("<h1>Outdoors Temperature: " + tempOutdoors[record_idx - 1] +"&deg;C </h1>");
    client->println("<h1>Outdoors Humidity: " + humiOutdoors[record_idx - 1] + "% </h1>");
  }

}

void displayWebButton(WiFiClient * const client)
{
#if 0
  // Display current state, and ON/OFF buttons for GPIO 26
  client->println("<p>GPIO 26 - State " + relayM3State + "</p>");
  // If the relayM3State is off, it displays the ON button
  if (relayM3State=="off") {
    client->println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
  } else {
    client->println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
  }

  // Display current state, and ON/OFF buttons for GPIO 27
  client->println("<p>GPIO 27 - State " + relayM4State + "</p>");
  // If the relayM4State is off, it displays the ON button
  if (relayM4State=="off") {
    client->println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
  } else {
    client->println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
  }
#endif
  // Display current state, and ON/OFF buttons for GPIO 27
  client->println("<p>The doors are " + relayM1State + "ED</p>");
  // If the relayM1State is off, it displays the ON button
  if (relayM1State=="CLOSE") {
    client->println("<p><a href=\"/28/on\"><button class=\"button\">OPEN</button></a></p>");
  } else {
    client->println("<p><a href=\"/28/off\"><button class=\"button button2\">OPEN</button></a></p>");
  }

  if (relayM1StateRedundant=="CLOSE") {
    client->println("<p><a href=\"/29/on\"><button class=\"button button2\">CLOSE</button></a></p>");
  } else {
    client->println("<p><a href=\"/29/off\"><button class=\"button\">CLOSE</button></a></p>");
  }
}

void displayWebPlots(WiFiClient * const client)
{
  client->println("<h2>Temperature and Humidity Plot</h2> \
  <div id=\"plot\"></div> \
  <div id=\"plot2\"></div>");

  for(int i=0; i < record_idx; i++)
  {
    client->println("<tr><td>"+ timestamp[i] +"</td> <td>"+ tempIndoors[i] +"</td> <td>"+ tempOutdoors[i] +
                    "</td> <td>"+ humiIndoors[i] +"</td> <td>"+ humiOutdoors[i] +"</td></tr>");
  }

  client->println("<script> \
    const dateTimes = [");
    for(int i=0; i < record_idx; i++)
    {
      client->println("\""+timestamp[i] +"\",");
    }
  client->println("];");

  client->println("const indoorsTemps = [");
    for(int i=0; i < record_idx; i++)
    {
      client->println(tempIndoors[i] +",");
    }
  client->println("];");

  client->println("const outdoorsTemps = [");
    for(int i=0; i < record_idx; i++)
    {
      client->println(tempOutdoors[i] +",");
    }
  client->println("];");

  // Sample data for the humidity plot
  client->println("const indoorsHumidity = [");
    for(int i=0; i < record_idx; i++)
    {
      client->println(humiIndoors[i] +",");
    }
  client->println("];");

  client->println("const outdoorsHumidity = [");
    for(int i=0; i < record_idx; i++)
    {
      client->println(humiOutdoors[i] +",");
    }
  client->println("];");

  // Create the traces for the temperature plot
  client->println("const indoorsTempTrace = { \
      x: dateTimes, \
      y: indoorsTemps, \
      mode: 'lines+markers', \
      name: 'Indoors Temperature', \
      line: { color: 'blue' }, \
    }; \
    const outdoorsTempTrace = { \
      x: dateTimes, \
      y: outdoorsTemps, \
      mode: 'lines+markers', \
      name: 'Outdoors Temperature', \
      line: { color: 'red' }, \
    };     \
    const dataTemp = [indoorsTempTrace, outdoorsTempTrace];");

    // Define the layout for the temperature plot
    // Create the temperature plot
    client->println("const layoutTemp = { \
      title: 'Temperature', \
      xaxis: { \
        title: 'Date/Time', \
        tickangle: -45, \
      }, \
      yaxis: { \
        title: 'Value', \
      }, \
    }; \
    Plotly.newPlot('plot', dataTemp, layoutTemp);");

    // Create the traces for the humidity plot
    client->println("const indoorsHumidityTrace = { \
      x: dateTimes, \
      y: indoorsHumidity, \
      mode: 'lines+markers', \
      name: 'Indoors Humidity', \
      line: { color: 'green' }, \
    }; \
    const outdoorsHumidityTrace = { \
      x: dateTimes, \
      y: outdoorsHumidity, \
      mode: 'lines+markers', \
      name: 'Outdoors Humidity', \
      line: { color: 'purple' }, \
    }; \
    const dataHumi = [indoorsHumidityTrace, outdoorsHumidityTrace];");

    // Define the layout for the humidity plot
    // Create the humidity plot
    client->println("const layoutHumi = { \
      title: 'Humidity', \
      xaxis: { \
        title: 'Date/Time', \
        tickangle: -45, \
      }, \
      yaxis: { \
        title: 'Value', \
      }, \
    }; \
    Plotly.newPlot('plot2', dataHumi, layoutHumi); \
  </script>");
}

void displayWebTable(WiFiClient * const client)
{
  client->println("<table style=\"width:100%\"> <tr><th>Date/Time</th><th>Temperature Indoors</th> \
                  <th>Temperature Outdoors</th><th>Humidity Indoors</th><th>Humidity Outdoors</th></tr>");
  for(int i=0; i < record_idx; i++)
  {
    client->println("<tr><td>"+ timestamp[i] +"</td> <td>"+ tempIndoors[i] +"</td> <td>"+ tempOutdoors[i] +
                    "</td> <td>"+ humiIndoors[i] +"</td> <td>"+ humiOutdoors[i] +"</td></tr>");
  }

  client->println("</table>");

}

void displayWebFooter(WiFiClient * const client)
{
  client->println("<footer><p>&copy; 2023 My Greenhouse Monitoring System. Author: Marcin Sosnowski</p></footer>");
}

void logicWeb()
{
  bool isDoorFree = !(doorState & DOOR_BUSY);

  // turns the GPIOs on and off
  if (header.indexOf("GET /26/on") >= 0) {
    Serial.println("GPIO 26 on");
    relayM3State = "on";
    digitalWrite(relayM3, LOW);
  } else if (header.indexOf("GET /26/off") >= 0) {
    Serial.println("GPIO 26 off");
    relayM3State = "off";
    digitalWrite(relayM3, HIGH);
  } else if (header.indexOf("GET /27/on") >= 0) {
    Serial.println("GPIO 27 on");
    relayM4State = "on";
    digitalWrite(relayM4, LOW);
  } else if (header.indexOf("GET /27/off") >= 0) {
    Serial.println("GPIO 27 off");
    relayM4State = "off";
    digitalWrite(relayM4, HIGH);
  } else if (isDoorFree && (header.indexOf("GET /28/on") >= 0)) {
    Serial.println("GPIO 28 on");
    relayM1State = "OPEN";
    relayM1StateRedundant = "OPEN";
    doorClose();
  } else if (isDoorFree && (header.indexOf("GET /28/off") >= 0)) {
    Serial.println("GPIO 28 off");
    relayM1State = "OPEN";
    relayM1StateRedundant = "OPEN";
    doorClose();
  } else if (isDoorFree && (header.indexOf("GET /29/on") >= 0)) {
    Serial.println("GPIO 29 on");
    relayM1State = "CLOSE";
    relayM1StateRedundant = "CLOSE";
    doorOpen();
  } else if (isDoorFree && (header.indexOf("GET /29/off") >= 0)) {
    Serial.println("GPIO 29 off");
    relayM1State = "CLOSE";
    relayM1StateRedundant = "CLOSE";
    doorOpen();
  }
}

void loop(){

  if(doMeas)
  {
    tempHumiMeas();
    doMeas=false;
    printLocalTime();
  }

  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            logicWeb();

            displayHttpOK(&client);
            displayWebPage(&client);
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");

  }
}
