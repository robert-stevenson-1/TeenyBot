#include <esp_now.h>
#include <WiFi.h>


struct ControllerData {
    int x;
    int y;
    bool button1;
    bool button2;
    bool button3;
};



//String deviceName = "ESP 1";
String deviceName = "ESP Receiver";

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcasterAddress[] = {0x94, 0xB9, 0x7E, 0xD9, 0xF9, 0x3C};

String success;

esp_now_peer_info_t peerInfo;

ControllerData received;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t  *incomingData, int len) {
  memcpy(&received, incomingData, sizeof(received));
  Serial.print("Bytes received: ");
  Serial.println(len);
  
  // Print the controller data
  Serial.print(received.button1);
  Serial.print(",");
  Serial.print(received.button2);
  Serial.print(",");
  Serial.print(received.button3);
  Serial.print(",");
  Serial.print(received.x);
  Serial.print(",");
  Serial.print(received.y);
  Serial.println();
}

void setup() {
  // Initialize Serial and ESP-Now
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcasterAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    // Perform other tasks in the main loop here

    // Delay or other operations as needed
    delay(10);
}
