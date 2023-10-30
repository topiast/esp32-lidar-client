#include <Arduino.h>
#include <vector>
#include <HardwareSerial.h>

#include <ArduinoJson.h>
#include <WiFi.h>
#include <ArduinoHttpClient.h>

#include "point.h"
#include "lidar.h"

// used to get from .env file
#define XSTR(x) #x
#define STR(x) XSTR(x)

// ***************Wireless Connection to Server ****************
// Define wifi
const char* ssid     = STR(SSID); 
const char* password = STR(PASSWORD);

// Define Server
// leviathan ip:
const String host = STR(HOST); // IP address of the server we will connect to
const int port = STR(PORT);

// --------- setup Wifi ------
WiFiClient wifi;
HttpClient client = HttpClient(wifi, host, port);
int status = WL_IDLE_STATUS;
// ---------------------------

// ***************Lidar ****************
const int lidarBaudRate = 115200;  // Baud rate for serial communication with the lidar module
const int motorControlPin = 12;    // Pin connected to the M_CTR line of the lidar module
Lidar lidar;

//Button to start the scan
const int buttonPin = 18;
int buttonState = LOW;
int lastButtonState = LOW;


void post_data(Point2d *pointsToSend, size_t postSize) {
  // const size_t pointSize = 2;  // Size of each point array
  // //using arduinojson.org/assistant to calculate the capacity
  // const size_t capacity = 6444;// JSON_ARRAY_SIZE(bufferSize) + bufferSize * JSON_ARRAY_SIZE(pointSize) + 7;

  // StaticJsonDocument<capacity> doc;

  // // Create a JSON array and add Point2d objects to it
  // JsonArray pointsArray = doc.createNestedArray("data");
  String jsonString;
  jsonString += "{\"data\":[";

  for (int i = 0; i < postSize; i++)
  {
    if(i != 0){
      jsonString += ","; 
    }
    jsonString += pointsToSend[i].toJsonString();
    Serial.println("Adding point");
    Serial.println(pointsToSend[i].x);
    Serial.println(pointsToSend[i].y);
    // JsonArray point = pointsArray.createNestedArray(); // Create a nested array for each point
    // point.add(pointsToSend[i].x);  // Add x value to the point array
    // point.add(pointsToSend[i].y);  // Add y value to the point array
  }
  jsonString += "]}";

  // Serialize the JSON document to a string
  // String jsonString;
  // serializeJson(doc, jsonString);

  // if(doc.overflowed()){
  //   Serial.println("JSON buffer overflow");
  // }

  String path = "/api/lidar";
  String contentType = "application/json";

  Serial.println("Sending data");

  client.post(path, contentType, jsonString);
  // client.get(path);
  const int statusCode = client.responseStatusCode();
  String response = client.responseBody();


  Serial.print("Status code: ");
  Serial.println(statusCode);
}


Point2d calculate_point(float distance, float angle){
  float x = distance * cos(angle);
  float y = distance * sin(angle);
  Point2d point = {x, y};
  return point;
}

void setup() {
  Serial.begin(115200);  // Initialize the primary serial port for debugging

  // Button setup
  pinMode(buttonPin, INPUT_PULLUP);
  buttonState = digitalRead(buttonPin);


  Serial.println("************************* Setup Wifi ******************************");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Try to connect to server");

  // test the connection to the server
  client.get("/api/lidar/reset");

  // client.post(path, contentType, jsonString);
  const int statusCode = client.responseStatusCode();
  String response = client.responseBody();

  Serial.print("Status code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);

  // const size_t size = JSON_ARRAY_SIZE(bufferSize) + bufferSize * JSON_ARRAY_SIZE(2) + 7;
  // Serial.print("Size: ");
  // Serial.println(size);

  Serial.println("************************* Setup Lidar ******************************");

  lidar.begin(lidarBaudRate, 16, 17);  // Initialize the serial port for the lidar module
  
  pinMode(motorControlPin, OUTPUT);  // Set the motor control pin as OUTPUT
  digitalWrite(motorControlPin, HIGH);  // Enable lidar motor rotation
  delay(1000);  // Wait for the lidar module to boot up
  // Serial.write('b');  // Send the 'b' command to start the lidar scanning
}

void loop() {
  lastButtonState = buttonState;
  buttonState = digitalRead(buttonPin);

  if(lastButtonState == HIGH && buttonState == LOW){
    Serial.println("Button pressed");
    // reset the lidar
    client.get("/api/lidar/reset");
    
    //status code
    const int statusCode = client.responseStatusCode();
    String response = client.responseBody();

    Serial.println("Starting scan");
    // start the lidar
    bool keep_scanning = true;
    while (keep_scanning) {
      lastButtonState = buttonState;
      buttonState = digitalRead(buttonPin);
      // if button is pressed reset the lidar and break the loop
      if(lastButtonState == HIGH && buttonState == LOW){
        Serial.println("Button pressed");
        // reset the lidar
        client.get("/api/lidar/stop");
        keep_scanning = false;
        break;
      }
      lidar.scan();  // Scan the environment and store the results in the point buffer

      if (lidar.get_buffer_fill() >= 0.5) {
        Serial.println("Scan finished");
        Serial.print("Posting data: ");
        Serial.println(lidar.points_added());
        post_data(lidar.begin_points(), lidar.points_added());
        lidar.clear_buffer();
      }
    }
  }
}