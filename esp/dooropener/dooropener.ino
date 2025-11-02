#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

/* Put your SSID & Password */
const char* ssid = "Garage Interface";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

const int DOOR_STATUS_PIN = D5;
const int CAR_STATUS_PIN = D6;
const int DOOR_OVERRIDE_PIN = D7;

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

ESP8266WebServer server(80);


void setup() {
	Serial.begin(115200);
	pinMode(DOOR_STATUS_PIN, INPUT);
	pinMode(CAR_STATUS_PIN, INPUT);
	pinMode(DOOR_OVERRIDE_PIN, OUTPUT);

	WiFi.softAP(ssid, password, 1, 1);
	WiFi.softAPConfig(local_ip, gateway, subnet);
	delay(100);
	
	server.on("/", handle_root);
	server.on("/door_override", handle_door_override);
	server.onNotFound(handle_NotFound);
	
	server.begin();
	Serial.println("HTTP server started");
}
void loop() {
	server.handleClient();
}

void handle_root() {
	server.send(200, "text/html", SendHTML()); 
}

void handle_door_override() {
	digitalWrite(DOOR_OVERRIDE_PIN, HIGH);
	delay(200);
	digitalWrite(DOOR_OVERRIDE_PIN, LOW);
	server.send(200, "text/html", SendHTML()); 
}

void handle_NotFound() {
	server.send(404, "text/plain", "Not found");
}

String SendHTML() {
	int door_status = digitalRead(DOOR_STATUS_PIN);
	int car_status = digitalRead(CAR_STATUS_PIN);

	String ptr = "<!DOCTYPE html> <html>\n";
	ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
	ptr +="<title>Garage Control</title>\n";
	ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
	ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
	ptr +=".button {display: block;width: 120px;background-color: #1abc9c;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
	ptr +=".button-on {background-color: #1abc9c;}\n";
	ptr +=".button-on:active {background-color: #16a085;}\n";
	ptr +=".button-off {background-color: #34495e;}\n";
	ptr +=".button-off:active {background-color: #2c3e50;}\n";
	ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
	ptr +="</style>\n";
	ptr +="</head>\n";
	ptr +="<body>\n";
	ptr +="<h1>Garage Door Interface 1.0</h1>\n";
	ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
	
	if(door_status) {
		ptr +="<p>DOOR: OPEN</p>\n";
	} else {
		ptr +="<p>DOOR: CLOSED</p>\n";
	}

	if(car_status) {
		ptr +="<p>CAR: PRESENT</p>\n";
	} else {
		ptr +="<p>CAR: MISSING</p>\n";
	}

	ptr += "<a class=\"button button-on\" href=\"/door_override\">DOOR OVERRIDE</a>\n";

	ptr +="</body>\n";
	ptr +="</html>\n";
	return ptr;
}