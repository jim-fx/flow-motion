#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "MPU6050.h"

//BLE server name
#define bleServerName "ESP32"

int16_t ax, ay, az;
int16_t gx, gy, gz;

bool deviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

MPU6050 accelgyro;
BLEServer *pServer;
BLECharacteristic bmeTemperatureCelsiusCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeTemperatureCelsiusDescriptor(BLEUUID((uint16_t)0x2902));

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device Connected");
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device Disconnected");
  }
};


void setup() {
  // Start serial communication 
  Serial.begin(115200);
  
  // setup software i2c
  Wire.begin();
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
  bmeService->addCharacteristic(&bmeTemperatureCelsiusCharacteristics);
  bmeTemperatureCelsiusDescriptor.setValue("BME temperature Celsius");
  bmeTemperatureCelsiusCharacteristics.addDescriptor(&bmeTemperatureCelsiusDescriptor);
  
  // Start the service
  bmeService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

int idx = 0;

void serialize_int16_t(uint8_t* buffer, int16_t* values, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        buffer[i * 2] = (uint8_t)(values[i] & 0xFF);
        buffer[i * 2 + 1] = (uint8_t)((values[i] >> 8) & 0xFF);
    }
}


void loop() {

  idx++;

  if (deviceConnected) {

    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 

    int16_t values[] = {ax, ay, az, gx, gy, gz};
    size_t count = sizeof(values) / sizeof(values[0]);

    // Serialize
    uint8_t serializedBuffer[count * 2];
    serialize_int16_t(serializedBuffer, values, count);

    // Set characteristic value and notify connected client
    bmeTemperatureCelsiusCharacteristics.setValue(serializedBuffer, count * 2);
    bmeTemperatureCelsiusCharacteristics.notify();
    if (idx % 20 == 0 && false) {
      // display tab-separated accel/gyro x/y/z values
      Serial.print("a/g:\t");
      Serial.print(ax);
      Serial.print("\t");
      Serial.print(ay);
      Serial.print("\t");
      Serial.print(az);
      Serial.print("\t");
      Serial.print(gx);
      Serial.print("\t");
      Serial.print(gy);
      Serial.print("\t");
      Serial.println(gz);
    }

  }

    // Check for a reset command from serial
  if (Serial.available() > 0) {
    char command = Serial.read();

    // Check if the received command is 'R'
    if (command == 'R' || command == 'r') {
      Serial.println("Resetting...");
      delay(1000); // Optional delay for visibility in the Serial Monitor
      ESP.restart(); // This function resets the ESP32
    }
  }

  delay(20);
}
