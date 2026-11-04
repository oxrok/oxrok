// Copyright (C) 2026 Miguel Angel Ajo Pelayo <miguelangel@ajo.es>

//! Lists the USB devices the backend sees.

fn main() {
    let backend = oxrok::usb::Backend::current().map_or("none", |b| b.name());
    match oxrok::usb::list_devices() {
        Ok(devices) => {
            println!("backend {backend}, {} devices", devices.len());
            for d in devices {
                println!(
                    "{} {:>7} v{:04x} {} {} {}",
                    d.id(),
                    d.conn(),
                    d.device_version,
                    d.manufacturer.as_deref().unwrap_or("-"),
                    d.product.as_deref().unwrap_or("-"),
                    d.serial.as_deref().unwrap_or("-")
                );
            }
        }
        Err(e) => eprintln!("{e}"),
    }
}
