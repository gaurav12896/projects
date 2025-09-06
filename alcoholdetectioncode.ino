#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Pin Definitions
#define ALCOHOL_SENSOR A0  // MQ3 sensor connected to A0
#define BUZZER 8           // Buzzer connected to digital pin 8
#define RELAY 9            // Relay connected to digital pin 9
#define GPS_RX_PIN 10      // GPS TX pin connected to Arduino pin 10
#define GPS_TX_PIN 11      // GPS RX pin (not used, can be left unconnected)
#define GSM_RX_PIN 7       // GSM TX pin connected to Arduino pin 7
#define GSM_TX_PIN 6       // GSM RX pin connected to Arduino pin 6

// Create Software Serial for GPS and GSM
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
SoftwareSerial gsmSerial(GSM_RX_PIN, GSM_TX_PIN);
TinyGPSPlus gps;

int threshold = 300;       // Threshold for alcohol level, adjust as needed
int debounceDelay = 1000;  // 1-second debounce delay
int sampleCount = 20      // Number of samples to average

unsigned long lastDetectionTime = 0;  // Last time alcohol was detected

void setup() {
  // Set up pin modes
  pinMode(ALCOHOL_SENSOR, INPUT); // MQ3 sensor is an input
  pinMode(BUZZER, OUTPUT);        // Buzzer is an output
  pinMode(RELAY, OUTPUT);         // Relay is an output to control motor

  // Initialize outputs to avoid unintended signals
  digitalWrite(BUZZER, LOW);      // Keep buzzer off initially
  digitalWrite(RELAY, HIGH);      // Keep relay in ON state initially (motor running)

  // Initialize Serial Monitor for debugging purposes
  Serial.begin(9600);
  gpsSerial.begin(9600);          // Start GPS serial communication
  gsmSerial.begin(9600);          // Start GSM serial communication

  // Setup GSM module
  sendSMS("GSM Module Initialized. Ready to detect alcohol.");
}

void loop() {
  int total = 0;

  // Take multiple samples for averaging
  for (int i = 0; i < sampleCount; i++) {
    total += analogRead(ALCOHOL_SENSOR);
    delay(10); // Short delay between samples
  }

  // Calculate the average alcohol level
  int averageAlcoholLevel = total / sampleCount;
  Serial.print("Average Alcohol Level: ");
  Serial.println(averageAlcoholLevel);

  // Check if alcohol is detected consistently above the threshold
  if (averageAlcoholLevel > threshold) {
    lastDetectionTime = millis(); // Update the detection time
    digitalWrite(BUZZER, HIGH);   // Buzzer on
    digitalWrite(RELAY, LOW);     // Relay off, stopping motor

    // GPS data reading
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read()); // Encode incoming data from GPS
    }

    // Check if GPS has a valid location fix
    if (gps.location.isUpdated()) {
      // Prepare the message with alcohol detection alert and location
      String message = "Alcohol Detected! Location: ";
      message += "Latitude: ";
      message += String(gps.location.lat(), 6);
      message += ", Longitude: ";
      message += String(gps.location.lng(), 6);

      // Send SMS with the detection alert and GPS coordinates
      sendSMS(message);

      // Print to Serial Monitor for debugging
      Serial.println(message);
    } else {
      Serial.println("Waiting for GPS fix...");
    }
  } 
  else if (millis() - lastDetectionTime > debounceDelay) {
    // If no alcohol detected and debounce delay has passed
    digitalWrite(BUZZER, LOW);    // Buzzer off
    digitalWrite(RELAY, HIGH);    // Relay on, motor running
  }

  delay(500); // Adjust this delay if needed for system response
}

// Function to send SMS using GSM module
void sendSMS(String message) {
  gsmSerial.println("AT+CMGF=1");    // Set SMS text mode
  delay(100);
  gsmSerial.println("AT+CMGS=\"+918091285980\""); // Replace with your phone number
  delay(100);
  gsmSerial.print(message);          // Send message content
  delay(100);
  gsmSerial.write(26);               // ASCII code for CTRL+Z to send SMS
  delay(1000);
}
