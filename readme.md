## Hardware Specifications

### 1. Directional Antenna System

| Parameter | Specification |
|---|---|
| Antenna Type | Yagi-Uda Directional Antenna |
| Operating Frequency | 2.4 GHz |
| Number of Antennas | 4 |
| Antenna Orientation | 0°, 90°, 180°, 270° |
| Coverage | 360° Horizontal Direction Finding |
| Measurement Parameter | RSSI |
| Function | Directional Wi-Fi Signal Reception |

### 2. Antenna Nodes

| Parameter | Specification |
|---|---|
| Microcontroller | ESP32-WROOM-32U |
| Number of Nodes | 4 |
| Operating Frequency | 2.4 GHz |
| Wi-Fi Standard | IEEE 802.11 b/g/n |
| Communication Protocol | ESP-NOW |
| Signal Measurement | RSSI |
| Scanning Method | Wi-Fi Network Scanning |
| Data Transmission | Wireless |
| Function | Wi-Fi Signal Detection and RSSI Reporting |

### 3. Master Controller

| Parameter | Specification |
|---|---|
| Microcontroller | ESP32 |
| Communication Protocol | ESP-NOW |
| Connected Nodes | 4 |
| Communication Interface | USB Serial |
| Baud Rate | 115200 |
| Target Identification | BSSID Matching |
| Processing | RSSI Comparison |
| Output | Estimated Direction and Angle |

### 4. Pan Mechanism

| Parameter | Specification |
|---|---|
| Motor Type | Stepper Motor |
| Motor Model | 23HD65001Y-21L |
| Rated Current | 2.8 A/Phase |
| Configured Resolution | 400 Pulses/Revolution |
| Motor Driver | TB6600 |
| Driver Current Rating | Up to 4 A |
| Driver Voltage Range | 9–42 V |
| Controller | ESP32 |
| STEP Pin | GPIO 18 |
| DIR Pin | GPIO 19 |
| ENABLE Pin | GPIO 17 |
| Home Switch | GPIO 27 |
| Function | Horizontal Directional Positioning |

### 5. Software Specifications

| Component | Technology |
|---|---|
| Firmware Development | Arduino IDE |
| Programming Language | C/C++ |
| Microcontroller Platform | ESP32 |
| Wireless Communication | ESP-NOW |
| Signal Detection | Wi-Fi Scanning |
| Signal Measurement | RSSI |
| Target Identification | BSSID |
| Dashboard | Python |
| Serial Communication | USB Serial |
| Motor Control | Step/Direction Control |

### 6. System Communication

| Communication Link | Technology |
|---|---|
| Antenna Nodes to Master | ESP-NOW |
| Master to Dashboard | USB Serial |
| Dashboard to Master | USB Serial |
| Master to Pan Controller | ESP-NOW |
| Pan Controller to Motor Driver | STEP/DIR Signals |

#### 1.1 Antenna Fabrication and Design

Four custom-built 2.4 GHz Yagi-Uda directional antennas were designed and fabricated using 3 mm solid copper wire elements mounted on an acrylic/PVC boom.

Each antenna consists of eight elements: one reflector, one driven element, and six directors.

**Antenna Design Specifications**

| Parameter | Specification |
|---|---|
| Operating Frequency | 2.44 GHz |
| Antenna Type | 8-Element Yagi-Uda |
| Number of Elements | 8 |
| Reflector | 1 |
| Driven Element | 1 |
| Directors | 6 |
| Boom Length | 300 mm |
| Boom Material | Acrylic / PVC |
| Element Material | 3 mm Solid Copper Wire |
| Expected Gain | Approximately 11 dBi |
| Expected Beamwidth | 40–50° |
| RF Connector | SMA |

**Element Dimensions**

| Element | Length (mm) | Position on Boom (mm) |
|---|---:|---:|
| Reflector | 61.0 | 0 |
| Driven Element | 58.0 (29 + 29) | 30.5 |
| Director 1 | 53.8 | 49 |
| Director 2 | 52.8 | 86 |
| Director 3 | 51.9 | 128 |
| Director 4 | 51.3 | 174.5 |
| Director 5 | 50.8 | 225 |
| Director 6 | 50.3 | 279 |

**Fabrication Procedure**

1. **Copper Element Preparation:** Cut 3 mm solid copper wire into the required lengths for the reflector, driven element, and six directors.
2. **Boom Preparation:** Prepare a 300 mm acrylic/PVC boom and mark the element positions using a digital vernier caliper.
3. **Driven Element Assembly:** Divide the driven element into two equal 29 mm sections. Solder one section to the SMA connector's centre pin and the other to its outer body, maintaining a 1–1.5 mm gap.
4. **Element Installation:** Insert all eight copper elements through the marked positions on the boom, ensuring that they are centred and perpendicular to the boom.
5. **Final Assembly:** Verify element lengths, spacing, and alignment before securing the elements using epoxy or hot glue.
6. **Antenna Replication:** Fabricate all four antennas using the same dimensions and construction procedure to maintain consistency across the direction-finding array.

**Antenna Integration**

The four fabricated Yagi-Uda antennas are arranged at 0°, 90°, 180°, and 270° orientations. Each antenna is connected to an ESP32-WROOM-32U module for Wi-Fi signal scanning and RSSI measurement.

The measured signal strengths from the four directions are compared to estimate the direction of the selected Wi-Fi signal.
