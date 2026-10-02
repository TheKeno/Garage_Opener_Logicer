# Garage controller board (rev A)

A barebones board for `Garage_Opener_Logicer_R6`: an ATmega328P on a 16 MHz
crystal, powered from USB-C, with connectors for everything in the garage. The
chip is pinned out exactly like an Uno, so the sketch runs unchanged.

Open `garage_controller.kicad_pro` in KiCad 10. The schematic passes ERC with
no errors or warnings. The PCB has its outline, all footprints placed and every
pad on its net, and it matches the schematic exactly (DRC with schematic
parity). **It is not routed yet**: the only DRC items left are the 141
unrouted connections.

| File | |
|---|---|
| `garage_controller.kicad_sch` | schematic (A3, one sheet): each block drawn with wires, net labels only between blocks |
| `garage_controller.kicad_pcb` | 120 × 90 mm, 2 layers, placed, unrouted |
| `garage_controller.pdf`, `bom.csv`, `placement.png` | snapshots for reading without KiCad; re-export after edits |

## Connectors

| | Connector | Pins | Notes |
|---|---|---|---|
| J1 | USB-C (power only) | | 5.1k CC pull-downs, so any USB-C charger gives 5 V; 0.75 A polyfuse |
| J4 | LCD, JST-XH 4 | GND, 5V, SDA, SCL | standard PCF8574 backpack order; 4.7k pull-ups on board |
| J5 | Encoder, JST-XH 5 | CLK, DT, SW, +, GND | KY-040 order; SW is wired to D2 with the GUI button, so the knob's push works too |
| J6 | GUI button, JST-XH 2 | BTN, GND | to ground, D2 |
| J7 | Sonar, JST-XH 4 | VCC, TRIG, ECHO, GND | HC-SR04 order; series resistors for the long cable |
| J8 | Door microswitch, JST-XH 2 | SW, GND | 4.7k pull-up plus 1k/100 nF RC against noise on the long run |
| J9 | LDR, JST-XH 2 | 5V, A0 | 10k to ground on board, so more light reads higher |
| J10 | Opener, screw terminal 3 | NC, NO, COM | relay contacts: wire COM (pin 3) and NO (pin 2) across the opener's wall-button terminals |
| J11 | Park assist, JST-XH 5 | 5V, BUZ−, RED, GREEN, GND | 330 Ω on board for the LEDs; an active 5 V buzzer between 5V and BUZ− |
| J12 | ESP8266, 1×7 header | GND, 5V, TX, RX, DOOR_REQ, DOOR_STAT, CAR_STAT | 3.3 V signals; 5 V feeds the ESP board's own regulator |
| J2 | ISP, 2×3 | standard AVR ISP | for burning the bootloader |
| J3 | FTDI, 1×6 | GND, CTS, VCC, TX, RX, DTR | VCC is not connected, so power the board from USB-C while uploading |

## ESP8266 link

The ESP runs at 3.3 V and the ATmega at 5 V, so every line is shifted:

- **ESP TX → ATmega RX:** a BAT85 diode pulls a 10k-pulled-up line low. The idle
  high sits at about 3.6 V, comfortably above the ATmega's 3.0 V threshold.
- **ATmega → ESP** (RX, DOOR_STAT, CAR_STAT): 2.2k/3.3k dividers bring 5 V down
  to 3.0 V.
- **DOOR_REQ:** an NPN transistor plays the push button that R6 expects on D11
  (an active-low input with the internal pull-up). The ESP drives the pin
  **high** to press it. An unplugged ESP reads as "not pressed".

Two things in `esp/dooropener` disagree with R6 and need fixing on the ESP side:

- Its web page shows `DOOR: OPEN` when DOOR_STAT is high, but R6 drives
  `doorStatus` high when the door is **closed**.
- R6's comments describe the ESP as talking over the UART (OPEN / CLOSE /
  STATE …). The ESP sketch in the repo only uses the three pins.

## Building and flashing

1. Fit a **DIP-28 socket**. The footprint is sized for one, but the socket
   isn't in the BOM, so order it separately. Also order JST-XH housings and
   crimps for the cables.
2. Burn the bootloader once over J2. Use a USBasp, or a spare Uno running
   "Arduino as ISP". In the Arduino IDE, pick the **Arduino Uno** board and run
   *Burn Bootloader*. This also sets the fuses for the 16 MHz crystal.
3. Upload R6 over J3 with a 5 V FTDI adapter (DTR resets the chip
   automatically), or keep using the ISP header with *Upload Using Programmer*.
4. **Unplug the ESP for any programming.** It shares the UART with J3, and its
   DOOR_REQ transistor sits on D11, which is MOSI on the ISP header.

## Routing notes

- **Net classes:** a *Power* class (0.8 mm) covers +5V, GND, VBUS and the relay
  nets. Everything else is *Default* (0.3 mm).
- **Ground pour:** a GND zone covers the whole back layer but is left unfilled.
  Press `B` once the tracks are in.
- **Short traces:** keep the crystal (Y1, C6, C7) and the decoupling caps
  (C3, C4) short and close to U1.
- **Relay contacts:** J10's contacts only switch the opener's low-voltage
  button line, so normal clearances are fine. Don't use it for mains.

## Check before ordering

- **USB-C connector:** J1 is the one SMD part. It has six large pads and is
  easy to solder with an iron. Swap it for `Connector:USB_B` with
  `USB_B_OST_USB-B1HSxx_Horizontal` if you'd rather stay fully through-hole.
- **J10 orientation:** check in the 3D viewer that the wire entries face the
  board edge.
- **LDR resistor:** R12 (10k) has to match the resistor in your current build,
  or the light thresholds stored in EEPROM will be off.
- **Power budget:** about 0.6 A peak with the ESP, relay, LCD backlight and
  buzzer all on. Use a charger rated for at least 1 A.
