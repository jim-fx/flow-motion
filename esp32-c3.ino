#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "MPU6050.h"
#include "Wire.h"

#define SERVICE_UUID "9b9f77c6-7e68-4109-b987-b096233d9525"
#define CHARACTERISTIC_UUID "1ab2c9f4-19c0-48dd-8932-ed72558ec593"

MPU6050 accelgyro;
BLECharacteristic *pCharacteristic;
BLEServer *pServer;
bool deviceConnected = false;

int16_t ax, ay, az;
int16_t gx, gy, gz;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Device connected");
    deviceConnected = true;
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected");
    deviceConnected = false;
    BLEDevice::startAdvertising();  // Restart advertising
  }
};

void setup_ble() {
  BLEDevice::init("ESP32");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  uint8_t gyroData[] = { 1, 2, 3, 4, 5, 6 };
  pCharacteristic->setValue(gyroData, sizeof(gyroData));
  pCharacteristic->notify();
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void setup() {
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values
  Serial.println("Updating internal sensor offsets...");
  // -76	-2359	1688	0	0	0
  Serial.print(accelgyro.getXAccelOffset());
  Serial.print("\t");  // -76
  Serial.print(accelgyro.getYAccelOffset());
  Serial.print("\t");  // -2359
  Serial.print(accelgyro.getZAccelOffset());
  Serial.print("\t");  // 1688
  Serial.print(accelgyro.getXGyroOffset());
  Serial.print("\t");  // 0
  Serial.print(accelgyro.getYGyroOffset());
  Serial.print("\t");  // 0
  Serial.print(accelgyro.getZGyroOffset());
  Serial.print("\t");  // 0
  Serial.print("\n");
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  Serial.print(accelgyro.getXAccelOffset());
  Serial.print("\t");  // -76
  Serial.print(accelgyro.getYAccelOffset());
  Serial.print("\t");  // -2359
  Serial.print(accelgyro.getZAccelOffset());
  Serial.print("\t");  // 1688
  Serial.print(accelgyro.getXGyroOffset());
  Serial.print("\t");  // 0
  Serial.print(accelgyro.getYGyroOffset());
  Serial.print("\t");  // 0
  Serial.print(accelgyro.getZGyroOffset());
  Serial.print("\t");  // 0
  Serial.print("\n");


  setup_ble();
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

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

  if (deviceConnected) {

    uint8_t encodedData[12];

    encodedData[0] = static_cast<uint8_t>(ax & 0xFF);
    encodedData[1] = static_cast<uint8_t>((ax >> 8) & 0xFF);

    encodedData[2] = static_cast<uint8_t>(ay & 0xFF);
    encodedData[3] = static_cast<uint8_t>((ay >> 8) & 0xFF);

    encodedData[4] = static_cast<uint8_t>(az & 0xFF);
    encodedData[5] = static_cast<uint8_t>((az >> 8) & 0xFF);

    encodedData[6] = static_cast<uint8_t>(gx & 0xFF);
    encodedData[7] = static_cast<uint8_t>((gx >> 8) & 0xFF);

    encodedData[8] = static_cast<uint8_t>(gy & 0xFF);
    encodedData[9] = static_cast<uint8_t>((gy >> 8) & 0xFF);

    encodedData[10] = static_cast<uint8_t>(gz & 0xFF);
    encodedData[11] = static_cast<uint8_t>((gz >> 8) & 0xFF);

    pCharacteristic->setValue(encodedData, sizeof(encodedData));
    pCharacteristic->notify();
  }


  delay(50);
}
