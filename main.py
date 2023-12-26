import asyncio
from bleak import BleakClient

# Specify the MAC address and service UUID
device_address = "64:E8:33:DA:A2:B6"
service_uuid = "9b9f77c6-7e68-4109-b987-b096233d9525"
characteristic_uuid = "1ab2c9f4-19c0-48dd-8932-ed72558ec593"

async def connect_to_ble_device(address, uuid):
    async with BleakClient(address) as client:
        print(f"Connected: {client.is_connected}")

        # Discover services
        services = await client.get_services()
        for service in services:
            if service.uuid.lower() == uuid.lower():
                print(f"Found Service: {service.uuid}")

                # You can further explore characteristics within the service if needed
                for char in service.characteristics:
                    if char.uuid.lower() == characteristic_uuid.lower():
                        print(f"Found Characteristic: {char.uuid}")

                        while True:
                            # Read the value of the characteristic
                            value = await client.read_gatt_char(char.uuid)
                            integers = [int(byte) for byte in value]
                            print(f"Characteristic Value as Integers: {integers}")

                            # Wait for the specified interval
                            await asyncio.sleep(10 / 1000.0)

async def main():
    await connect_to_ble_device(device_address, service_uuid)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())

