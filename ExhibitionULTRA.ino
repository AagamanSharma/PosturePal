#define BLYNK_TEMPLATE_ID "TMPL6rf2MoXmA"
#define BLYNK_TEMPLATE_NAME "imu"
#define BLYNK_AUTH_TOKEN "axHlSwIS2vEvjp2a4uu4Pd7aU8J1_hQt"

#include <esp_now.h>
#include <WiFi.h>
#include <NewPing.h>
#include <BlynkSimpleEsp32.h> // Include Blynk library

#define TRIGGER_PIN 4
#define ECHO_PIN 2
#define MAX_DISTANCE 200
#define BUZZER_PIN 5 // Define the buzzer pin

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Blynk authentication token
char ssid[] = "Nonet"; // Your Wi-Fi SSID
char pass[] = "123456789"; // Your Wi-Fi password

// Virtual Pin for distance and control
#define DISTANCE_VPIN V3
#define CONTROL_VPIN V4

uint8_t broadcastAddress[] = {0xCC, 0xDB, 0xA7, 0x30, 0x56, 0x8C}; 

typedef struct struct_message {
    int distance;
} struct_message;

struct_message myData;
bool isSystemOn = false; // Variable to store the state of the system

void sendData() {
    if (isSystemOn) {
        // Read distance from the ultrasonic sensor
        myData.distance = sonar.ping_cm();

        // Send data via ESP-NOW
        esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

        // Send distance to Blynk app
        Blynk.virtualWrite(DISTANCE_VPIN, myData.distance);
        
        Serial.print("Sent Distance: ");
        Serial.print(myData.distance);
        Serial.println(" cm");

        // Check if distance exceeds 15 cm and trigger event
        if (myData.distance > 15) {
            Serial.println("BAD POSTURE DETECTED, triggering Blynk event...");
            Blynk.logEvent("sit_st", "!!!HUMAN QUESTION MARK IN THE MAKING!!!"); 

            // Activate the buzzer
            digitalWrite(BUZZER_PIN, HIGH);
        } else {
            Serial.println("POSTURE ACCURATE.");
            
            // Deactivate the buzzer
            digitalWrite(BUZZER_PIN, LOW);
        }
    } else {
        // Turn off the buzzer if the system is off
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("System is OFF.");
    }
}

// Blynk function to turn the system on or off
BLYNK_WRITE(CONTROL_VPIN) {
    isSystemOn = param.asInt(); // Get the button state: 1 for ON, 0 for OFF
    if (isSystemOn) {
        Serial.println("System is ON");
    } else {
        Serial.println("System is OFF");
    }
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    Serial.println("Data received");
    memcpy(&myData, data, sizeof(myData));
    Serial.print("Received Distance: ");
    Serial.print(myData.distance);
    Serial.println(" cm");
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting setup...");

    // Initialize Wi-Fi in station mode
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Initialize peer info structure
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));  // Zero out the structure
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Register the receive callback function (if needed)
    esp_now_register_recv_cb(onDataRecv);

    // Initialize Blynk
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

    // Initialize the buzzer pin as an output
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Make sure the buzzer is off initially

    Serial.println("Setup complete");
}

void loop() {
    Blynk.run(); // Run Blynk
    sendData();
    delay(2000); // Send data every 2 seconds
}
