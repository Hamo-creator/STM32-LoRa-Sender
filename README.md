# 📡 STM32 LoRa Sender (with Receive Capability)

This project implements a **LoRa-based wireless communication system** using an STM32 microcontroller.
It is primarily designed as a **sender node**, but also includes **receive functionality** for bidirectional communication.

---

## 🚀 Features

* 📤 LoRa data transmission (Sender mode)
* 📥 LoRa data reception (Receiver mode)
* 🔄 Bidirectional communication support
* ⚡ Based on STM32 HAL drivers
* 📡 Uses SX1278 LoRa transceiver (433 MHz)
* 🧩 Easily extendable for IoT applications

---

## 🧠 Project Overview

This project demonstrates how to interface an **SX1278 LoRa module** with an STM32 microcontroller using **SPI communication**.

LoRa enables **long-range, low-power wireless communication**, making it ideal for IoT and embedded systems. 

The system can:

* Send messages wirelessly to another node
* Receive incoming messages from other LoRa devices
* Be configured as sender or receiver depending on logic

---

## 🛠️ Hardware Requirements

* STM32 development board (e.g., STM32F103, Nucleo, etc.)
* SX1278 LoRa module (Ra-02 recommended)
* 433 MHz antenna ⚠️ (important for proper operation)
* ST-Link programmer/debugger
* Jumper wires / breadboard

---

## 🔌 Wiring (STM32 ↔ SX1278)

| STM32 Pin | LoRa Module Pin | Description |
| --------- | --------------- | ----------- |
| SPI NSS   | NSS             | Chip Select |
| SPI SCK   | SCK             | Clock       |
| SPI MISO  | MISO            | Data Out    |
| SPI MOSI  | MOSI            | Data In     |
| GPIO      | DIO0            | Interrupt   |
| GPIO      | RST             | Reset       |
| 3.3V      | VCC             | Power       |
| GND       | GND             | Ground      |

The SX1278 communicates via **SPI interface**, which is supported by STM32 MCUs. ([How To Electronics][1])

---

## ⚙️ Software Requirements

* STM32CubeIDE
* STM32 HAL drivers
* Git (optional, for version control)

---

## 📁 Project Structure

```
Core/
 ├── Src/        # Application source files
 ├── Inc/        # Header files

Drivers/
 ├── CMSIS/
 ├── STM32_HAL_Driver/

LoRa/
 ├── LoRa driver files

.ioc             # STM32CubeMX configuration
```

---

## ▶️ Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/Hamo-creator/STM32-LoRa-Sender.git
cd STM32-LoRa-Sender
```

---

### 2. Open in STM32CubeIDE

* Open STM32CubeIDE
* Go to **File → Open Projects from File System**
* Select the project folder

---

### 3. Build the Project

* Click **Build (🔨)** or press `Ctrl + B`

---

### 4. Flash to STM32

* Connect your board via ST-Link
* Click **Run (▶)** to upload the firmware

---

## 🔄 How It Works

### Sender Mode

* Initializes LoRa module
* Sends packets periodically
* Can transmit sensor or custom data

### Receiver Mode

* Listens for incoming LoRa packets
* Reads and processes received data

> Depending on implementation, the node may switch between modes or support both simultaneously.

---

## 🧪 Example Use Cases

* 📡 Wireless sensor networks
* 🌱 Smart agriculture (temperature/humidity nodes)
* 🏠 Home automation
* 🚗 Remote monitoring systems

---

## ⚠️ Notes & Tips

* Always use a **433 MHz antenna** — running without one can damage the module.
* Ensure both nodes use the **same frequency, spreading factor, and bandwidth**.
* Keep SPI connections short for reliable communication.
* Power the module with **3.3V only** (NOT 5V).

---

## 🔧 Future Improvements

* Add encryption support
* Implement LoRaWAN compatibility
* Add sensor integration (BME280, DHT11, etc.)
* Low-power (sleep mode) optimization
* Multi-node communication

---

## 🤝 Contributing

Contributions are welcome!

If you'd like to improve this project:

1. Fork the repository
2. Create a new branch
3. Make your changes
4. Submit a Pull Request

---

## 📜 License

This project is open-source and available under the MIT License.

---

## 👤 Author

**Hamo-creator**

GitHub: https://github.com/Hamo-creator

---

## ⭐ Support

If you find this project useful, consider giving it a ⭐ on GitHub!

[1]: https://how2electronics.com/interfacing-lora-sx1278-stm32-sender-receiver/?utm_source=chatgpt.com "Interfacing LoRa SX1278 with STM32 - Sender & Receiver"

## Notice
The LoRa Library used in this example is from:
https://github.com/SMotlaq/LoRa.
