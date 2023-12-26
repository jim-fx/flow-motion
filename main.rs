use btleplug::api::{Central, CharPropFlags, Manager as _, Peripheral, ScanFilter};
use btleplug::platform::Manager;
use futures::stream::StreamExt;
use std::error::Error;
use std::time::Duration;
use tokio::time;
use uuid::Uuid;

const DEVICE_ADDR: &str = "64:E8:33:DA:A2:B6";
const NOTIFY_CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x1ab2c9f4_19c0_48dd_8932_ed72558ec593);

async fn find_adapter() -> Result<impl Central, Box<dyn Error>> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    for adapter in adapters {
        return Ok(adapter);
    }
    Err("No powered adapters found".into())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let adapter = find_adapter().await?;

    println!("Starting scan...");
    adapter
        .start_scan(ScanFilter::default())
        .await
        .expect("Can't scan BLE adapter for connected devices...");
    time::sleep(Duration::from_secs(2)).await;
    let peripherals = adapter.peripherals().await?;

    if peripherals.is_empty() {
        eprintln!("->>> BLE peripheral devices were not found, sorry. Exiting...");
        return Ok(());
    }

    if let Some(esp32) = peripherals
        .iter()
        .find(|peripheral| peripheral.address().to_string() == DEVICE_ADDR)
    {
        let properties = esp32.properties().await?;
        let is_connected = esp32.is_connected().await?;
        // let address = esp32.address().to_string();
        let local_name = properties
            .unwrap()
            .local_name
            .unwrap_or(String::from("(peripheral name unknown)"));

        println!("Found matching peripheral {:?}...", "ESP32");
        if !is_connected {
            // Connect if we aren't already connected.
            if let Err(err) = esp32.connect().await {
                eprintln!("Error connecting to peripheral, skipping: {}", err);
            }
        }
        let is_connected = esp32.is_connected().await?;
        println!(
            "Now connected ({:?}) to peripheral {:?}.",
            is_connected, &local_name
        );

        println!("Discover peripheral {:?} services...", local_name);
        esp32.discover_services().await?;
        for characteristic in esp32.characteristics() {
            println!("Checking characteristic {:?}", characteristic);
            // Subscribe to notifications from the characteristic with the selected
            // UUID.
            if characteristic.uuid == NOTIFY_CHARACTERISTIC_UUID
                && characteristic.properties.contains(CharPropFlags::NOTIFY)
            {
                println!("Subscribing to characteristic {:?}", characteristic.uuid);
                esp32.subscribe(&characteristic).await?;
                let mut notification_stream = esp32.notifications().await?;
                let mut old_time = time::Instant::now();
                while let Some(data) = notification_stream.next().await {
                    let delta = time::Instant::now() - old_time;
                    let delta_ms = delta.as_millis() as f64; // Convert Duration to milliseconds
                    let rounded_delta = (delta_ms * 10.0).round() / 10.0; // Round to one decimal place
                    println!(
                        "Received data from {:?} [{:?}ms]: {:?}",
                        local_name, rounded_delta, data.value
                    );
                    old_time = time::Instant::now();
                }
            }
        }
        println!("Disconnecting from peripheral {:?}...", local_name);
        esp32.disconnect().await?;
    } else {
        println!("ESP Not found")
    }

    Ok(())
}
