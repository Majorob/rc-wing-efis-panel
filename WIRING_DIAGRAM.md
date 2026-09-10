# RC Wing EFIS Wiring Diagram

## System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         EFIS SYSTEM ARCHITECTURE                     │
└─────────────────────────────────────────────────────────────────────┘

    SpeedyBee F405 Wing              GPS M10 Module       Airspeed Sensor
    (Flight Controller)              (UART Telemetry)     (ADC/I2C)
            │                               │                   │
            │ MAVLink Telemetry            │                   │
            │ (57600 baud UART)            │                   │
            └───────────────┬──────────────┴───────────────────┘
                            │
                    ┌───────▼────────┐
                    │   ESP32-32      │
                    │   N16P8 (EFIS)  │
                    └───────┬────────┘
                            │
                    ┌───────▼────────┐
                    │  4" LCD Screen  │
                    │ (ILI9488 or     │
                    │  ST7796 SPI)    │
                    └────────────────┘
```

---

## Detailed Pin Connections

### **ESP32-32 N16P8 Pinout**

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32-32 N16P8 PINOUT                     │
├──────────────┬──────────────────┬──────────────┬─────────────┤
│ Function     │ ESP32 Pin        │ Component    │ Notes       │
├──────────────┼──────────────────┼──────────────┼─────────────┤
│ UART0 RX     │ GPIO3 (U0RX)     │ Debug Serial │ Default     │
│ UART0 TX     │ GPIO1 (U0TX)     │ Debug Serial │ Default     │
│              │                  │              │             │
│ UART1 RX     │ GPIO9            │ SpeedyBee TX │ MAVLink In  │
│ UART1 TX     │ GPIO10           │ SpeedyBee RX │ MAVLink Out │
│              │                  │              │             │
│ UART2 RX     │ GPIO16           │ GPS M10 TX   │ GPS Data    │
│ UART2 TX     │ GPIO17           │ GPS M10 RX   │ GPS Config  │
│              │                  │              │             │
│ SPI CLK      │ GPIO18           │ LCD/Sensor   │ SPI Clock   │
│ SPI MOSI     │ GPIO23           │ LCD/Sensor   │ SPI Data Out│
│ SPI MISO     │ GPIO19           │ LCD/Sensor   │ SPI Data In │
│ SPI CS (LCD) │ GPIO5            │ LCD          │ Chip Select │
│ LCD DC       │ GPIO2            │ LCD          │ Data/Cmd    │
│ LCD RESET    │ GPIO4            │ LCD          │ Reset Line  │
│              │                  │              │             │
│ I2C SDA      │ GPIO21           │ Sensors      │ I2C Data    │
│ I2C SCL      │ GPIO22           │ Sensors      │ I2C Clock   │
│              │                  │              │             │
│ ADC0         │ GPIO36 (VP)      │ Airspeed     │ Analog In   │
│ ADC1         │ GPIO39 (VN)      │ Battery Volt │ Analog In   │
│              │                  │              │             │
│ GND          │ GND (Multiple)   │ All Devices  │ Common Gnd  │
│ +3V3         │ 3V3              │ Logic Devices│ 3.3V Supply │
│ +5V          │ VUSB/Vin         │ Motors/Power │ 5V Supply   │
└──────────────┴──────────────────┴──────────────┴─────────────┘
```

---

## Connection Diagrams

### **1. SpeedyBee F405 Wing → ESP32 (MAVLink Telemetry)**

```
SpeedyBee F405 Wing                          ESP32-32
┌─────────────────────┐                  ┌──────────────┐
│                     │                  │              │
│  UART Telem Port    │                  │  UART1       │
│  ┌───────────────┐  │                  │  ┌────────┐  │
│  │ TX ───────────┼──┼──────────────────┼─►│ RX (9) │  │
│  │               │  │                  │  └────────┘  │
│  │ RX ───────────┼──┼──────────────────┼─◄│ TX (10)│  │
│  │               │  │                  │  └────────┘  │
│  │ GND ──────────┼──┼──────────────────┼─►│ GND    │  │
│  └───────────────┘  │                  │  └────────┘  │
│                     │                  │              │
└─────────────────────┘                  └──────────────┘

Baud Rate: 57600
Protocol: MAVLink 2.0
Signal Level: 3.3V (add resistor divider if 5V output)
```

### **2. GPS M10 Module → ESP32**

```
GPS M10 Module                             ESP32-32
┌─────────────────┐                   ┌──────────────┐
│                 │                   │              │
│  UART Port      │                   │  UART2       │
│  ┌───────────┐  │                   │  ┌────────┐  │
│  │ TX ───────┼──┼───────────────────┼─►│ RX(16) │  │
│  │           │  │                   │  └────────┘  │
│  │ RX ───────┼──┼───────────────────┼─◄│ TX(17) │  │
│  │           │  │                   │  └────────┘  │
│  │ GND ──────┼──┼───────────────────┼─►│ GND    │  │
│  │ +5V ──────┼──┼───────────────────┼─►│ +5V    │  │
│  └───────────┘  │                   │  └────────┘  │
│                 │                   │              │
└─────────────────┘                   └──────────────┘

Baud Rate: 38400 (standard M10)
Protocol: NMEA 0183
Power: 5V from USB or external supply
```

### **3. Airspeed Sensor → ESP32**

**Option A: Analog ADC Connection (Recommended for simplicity)**
```
Airspeed Sensor                        ESP32-32
┌─────────────────┐                ┌──────────────┐
│                 │                │              │
│  Pitot Tube     │                │   ADC        │
│  ┌───────────┐  │                │  ┌────────┐  │
│  │ Signal ───┼──┼────────────────┼─►│ GPIO36 │  │
│  │ (0-5V)    │  │                │  │ (VP)   │  │
│  │           │  │                │  └────────┘  │
│  │ GND ──────┼──┼────────────────┼─►│ GND    │  │
│  │ +5V ──────┼──┼────────────────┼─►│ +5V    │  │
│  └───────────┘  │                │  └────────┘  │
│                 │                │              │
└─────────────────┘                └──────────────┘

Note: Add voltage divider (3.3k/2.2k) for 5V output to 3.3V ADC
Voltage divider formula: ADC_voltage = Signal_voltage * (2.2k / (3.3k + 2.2k))
```

**Option B: I2C Connection (for digital airspeed sensors)**
```
Airspeed Sensor (I2C)                ESP32-32
┌─────────────────┐                ┌──────────────┐
│                 │                │              │
│  ┌───────────┐  │                │   I2C        │
│  │ SCL ──────┼──┼────────────────┼─►│ GPIO22 │  │
│  │           │  │                │  │ (SCL)  │  │
│  │ SDA ──────┼──┼────────────────┼─►│ GPIO21 │  │
│  │           │  │                │  │ (SDA)  │  │
│  │ GND ──────┼──┼────────────────┼─►│ GND    │  │
│  │ +3.3V ────┼──┼────────────────┼─►│ +3.3V  │  │
│  └───────────┘  │                │  └────────┘  │
│                 │                │              │
└─────────────────┘                └──────────────┘

I2C Address: Typically 0x76 or 0x77 (check datasheet)
Add 4.7kΩ pull-up resistors on SDA/SCL to 3.3V
```

### **4. 4" LCD Screen → ESP32 (SPI Connection)**

**ILI9488 or ST7796 SPI Wiring**
```
4" LCD Screen                          ESP32-32
┌──────────────────────┐           ┌──────────────┐
│  Display Driver      │           │              │
│  ┌──────────────┐    │           │   SPI        │
│  │ GND ─────────┼────┼───────────┼─►│ GND    │  │
│  │              │    │           │  └────────┘  │
│  │ +3.3V ───────┼────┼───────────┼─►│ +3.3V  │  │
│  │              │    │           │  └────────┘  │
│  │ CLK (SCK) ───┼────┼───────────┼─►│ GPIO18 │  │
│  │              │    │           │  │ (SCLK) │  │
│  │ MOSI (DIN) ──┼────┼───────────┼─►│ GPIO23 │  │
│  │              │    │           │  │ (MOSI) │  │
│  │ MISO ────────┼────┼───────────┼─►│ GPIO19 │  │
│  │              │    │           │  │ (MISO) │  │
│  │ CS (SS) ─────┼────┼───────────┼─►│ GPIO5  │  │
│  │              │    │           │  │ (CS)   │  │
│  │ DC (RS/A0) ──┼────┼───────────┼─►│ GPIO2  │  │
│  │              │    │           │  │ (DC)   │  │
│  │ RESET ───────┼────┼───────────┼─►│ GPIO4  │  │
│  │              │    │           │  │ (RST)  │  │
│  │ BACKLIGHT ───┼────┼───────────┼─►│ GPIO15 │  │
│  │ (Optional)   │    │           │  │ (PWM)  │  │
│  └──────────────┘    │           │  └────────┘  │
│                      │           │              │
└──────────────────────┘           └──────────────┘

SPI Frequency: 40 MHz recommended
Backlight: Connect to GPIO15 for PWM brightness control
```

### **5. Battery Voltage Monitor → ESP32**

```
LiPo Battery (via BEC/Regulator)      ESP32-32
┌──────────────┐                  ┌──────────────┐
│              │                  │              │
│ +3.3V Line   │                  │   ADC        │
│ (with voltage│                  │  ┌────────┐  │
│  divider)    │                  │  │ GPIO39 │  │
│ ┌──────────┐ │                  │  │ (VN)   │  │
│ │ Signal ──┼─┼──────────────────┼─►│        │  │
│ │ (0-3.3V) │ │                  │  └────────┘  │
│ │          │ │                  │              │
│ │ GND ─────┼─┼──────────────────┼─►│ GND    │  │
│ └──────────┘ │                  │  └────────┘  │
│              │                  │              │
└──────────────┘                  └──────────────┘

Voltage Divider (for monitoring 6S LiPo):
Raw Voltage: 6S = 25.2V max
Divided Voltage: 25.2V * (R2 / (R1 + R2)) = 3.3V max
Recommended: R1 = 6.8kΩ, R2 = 1.5kΩ
Result: 25.2V input → 3.3V ADC input
```

---

## Power Distribution

```
┌─────────────────────────────────────────────────────┐
│           POWER DISTRIBUTION DIAGRAM                 │
├─────────────────────────────────────────────────────┤
│                                                      │
│  Battery (LiPo/NiMh)                               │
│         │                                            │
│         ├──► BEC 5V ──► SpeedyBee F405             │
│         │               (Flight Controller)          │
│         │                                            │
│         └──► BEC 3.3V ──┬──► ESP32-32              │
│                         │    (Main EFIS)            │
│                         │                            │
│                         ├──► GPS M10 (5V option)    │
│                         │                            │
│                         ├──► Airspeed Sensor        │
│                         │                            │
│                         └──► 4" LCD Screen          │
│                              (3.3V + Backlight)     │
│                                                      │
│  Note: Use separate BECs for analog noise immunity  │
│        Keep signal cables away from power lines      │
└─────────────────────────────────────────────────────┘
```

---

## Cable Requirements

```
┌────────────────────────────────────────────────────────┐
│              RECOMMENDED CABLE TYPES                    │
├────────────┬──────────────┬──────────────────────────┤
│ Connection │ Cable Type   │ Specifications           │
├────────────┼──────────────┼──────────────────────────┤
│ UART       │ Shielded     │ 22-24 AWG, twisted pair │
│ Telemetry  │ Twisted Pair │ Shield to GND            │
│            │              │ Keep <1 meter           │
├────────────┼──────────────┼──────────────────────────┤
│ GPS        │ Shielded     │ 22 AWG twisted pair      │
│            │ Twisted Pair │ Away from power         │
├────────────┼──────────────┼──────────────────────────┤
│ SPI LCD    │ Shielded     │ 24 AWG (short runs)     │
│            │              │ Keep <30 cm             │
├────────────┼──────────────┼──────────────────────────┤
│ Power      │ Unshielded   │ 16 AWG for high current │
│ Dist.      │ (as needed)  │ Minimize inductance     │
├────────────┼──────────────┼──────────────────────────┤
│ I2C        │ Twisted Pair │ 22 AWG with pull-ups    │
│            │              │ 4.7kΩ to 3.3V           │
└────────────┴──────────────┴──────────────────────────┘
```

---

## Voltage Divider Calculations

### For 5V ADC Input to 3.3V ESP32 ADC (Airspeed Sensor)
```
Formula: V_out = V_in × (R2 / (R1 + R2))

Target: 5V in → 3.3V out

Solution 1 (Recommended):
  R1 = 3.3kΩ
  R2 = 2.2kΩ
  Ratio = 2.2 / (3.3 + 2.2) = 0.4
  5V × 0.4 = 2.0V (Safe!)
  3.3V × 0.4 = 1.32V (Safe!)

Solution 2 (Alternative):
  R1 = 4.7kΩ
  R2 = 3.3kΩ
  Ratio = 3.3 / (4.7 + 3.3) = 0.412
  Result: ~2.06V @ 5V input
```

---

## Signal Level Translation (if needed)

**SpeedyBee to ESP32 (3.3V Logic)**

Most modern flight controllers output 3.3V logic, so direct connection is fine.
If your SpeedyBee outputs 5V:
- Use a resistor divider (as shown above)
- Or use a dedicated level shifter IC (TXS0102, BSS138)

---

## Testing Connections

Before powering up the full system:

1. **Check all ground connections** - Use continuity tester
2. **Verify voltage levels** - Use multimeter
3. **Inspect for shorts** - Between power and ground traces
4. **Dry-run with USB power** - Test ESP32 and LCD separately before connecting flight controller
5. **Monitor serial output** - Use PlatformIO serial monitor to verify data flow

---

## Troubleshooting Connection Issues

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| LCD displays garbage | Wrong SPI freq | Reduce to 20 MHz in TFT_eSPI setup |
| No GPS data received | Wrong baud rate | Verify M10 @ 38400 baud |
| MAVLink not decoding | Crossed TX/RX | Swap RX and TX wires |
| Airspeed reading 0 | ADC not connected | Check voltage divider, verify GPIO36 |
| Frequent resets | Noise/power issue | Add 100µF capacitor near ESP32 power |

---

## Complete Shopping List

```
CONNECTORS & CABLES:
□ Micro USB cable (ESP32 programming)
□ 2.54mm JST connectors (sensor connections)
□ Dupont wires (breadboarding/prototyping)
□ Shielded twisted pair cable (telemetry)

PASSIVE COMPONENTS:
□ 4.7kΩ resistors (I2C pull-ups) - 2x
□ 3.3kΩ resistor (voltage divider) - 1x
□ 2.2kΩ resistor (voltage divider) - 1x
□ 100µF capacitor (ESP32 power) - 2x
□ 10µF capacitor (SPI lines) - 2x
□ 0.1µF capacitor (decoupling) - 4x

TOOLS RECOMMENDED:
□ Soldering iron & solder
□ Heat shrink tubing
□ Multimeter
□ Logic analyzer (optional, for debugging)
□ Breadboard or perfboard
```

---

**Next Steps:**
1. Review the wiring diagram carefully
2. Gather all components
3. Test connections with multimeter before power-on
4. Proceed to firmware installation (see main README)
