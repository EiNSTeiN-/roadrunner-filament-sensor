# BOM

### Printed parts

Recommended settings:
* 0.2mm layer height
* 0.5mm (250%) layer width
* Cubic infill, 60%
* Supports enabled

| Part | Quantity | Notes |
|---|---|---|
| [a]_cover.stl | 1 | Print in accent color
| [c]_led_diffuser.stl | 1 | Print in clear material
| body_part_a.stl | 1 | Print in main color
| body_part_b.stl | 1 | Print in main color
| idler.stl | 1 | Print in main color
| magnet_holder.stl | 1 | Print in main color
| wire_cover.stl | 1 | Print in main color

### Electronics

Order the cheapest you can find of the following items:

| Qty. | Part | Est. Price | Description | Link |
|----|----|----|----|----|
| 1 | AS5600⁽¹⁾ | 1$ | Magnetic encoder breakout board, used for rotation detection | [aliexpress](https://vi.aliexpress.com/item/1005003080438185.html)
| 1 | MK3S IR Sensor⁽²⁾ | 5$ | IR sensor for filament presence detection | [aliexpress](https://vi.aliexpress.com/item/1005002551124258.html)
| 1 | Waveshare RP2040-Zero | 3$ | Order the board only without pre-soldered headers | [aliexpress](https://vi.aliexpress.com/item/1005004281549886.html)
| 1 | Wires | | Some lengths of wire to connect everything together (soldering required) | |
| 1 | JST-XH male 4-pin | | (Optional) Fits in the provided bottom cover | |

Notes:
1. The linked AS5600 breakout board comes with a 3mm diameter magnet. If you must purchase a magnet separately, make sure it has its poles oriented across its diameter, not across the thickness.
2. The IR sensor does not require any of the usual MK3S hardware although many kits on Aliexpress are sold with the extra hardware. Only a small 2mm self-tapping screw is needed to attach the PCB is needed for the build.

### Hardware

| Qty. | Part | Est. Price | Description | Link |
|----|----|----|----|----|
| 1 | BMG extuder kit⁽¹⁾ | 5$ | The kit should include the thumb screw assembly, drive gears, roller bearings, needle bearings, 3 x 20mm and 3 x 30mm shafts. | [aliexpress](https://vi.aliexpress.com/item/1005005443742333.html)
| 1 | M5 x 25mm rod⁽¹⁾ | 2$ | Order a longer length and cut up to size between 24 and 26mm | [aliexpress](https://vi.aliexpress.com/item/1005005041338002.html)
| 2 | M3 x 16mm SHCS | | Connects sensor body part A and B. Socket head screws recommended | |
| 6 | M3 x 6mm BHCS | | For AS5600, top and bottom covers. Button head screws required for clearance | |
| 1 | M3 x 10mm BHCS | | For bottom cover. | |
| 10 | M3 X D4.2 X L4.0 heatset inserts | | These are smaller than Voron standard inserts! | [aliexpress](https://vi.aliexpress.com/item/4000761483243.html) |
| 2 | PC4-M5 fitting | | To connect with PTFE tubes on top and bottom | [aliexpress](https://vi.aliexpress.com/item/1005005646446620.html) |

Notes:
1. It's possible to use the shaft of the large plastic gear that comes with the BMG extruder kit instead of purchasing a separate 5mm rod. Simply push out the plastic gear and grind down the theethed part of the shaft so that it fits properly in the 5mm bearings.