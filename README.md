# ESP32-S3 companion

This project can be built independently. Its UART protocol headers are also
present in the sibling display project; keep both copies byte-identical.

Connect display GPIO43 TX to GPIO44 RX, display GPIO44 RX to GPIO43 TX, and
GND to GND. All signals are 3.3 V. GPIO1–4 drive LEDs active-high through
individual current-limiting resistors. GPIO3 is a boot-strapping pin: ensure
the connected circuit does not interfere with its startup level.

Outputs start LOW and go LOW after two seconds without a valid command.
After a receiver restart or timeout, an all-off command is required before
outputs can be switched on again. Frames use CRC16-CCITT-FALSE for error
detection, not authentication.

GPIO5 and GPIO6 are configured as analog inputs; GPIO6 is reserved and not
yet read or transmitted. For display testing, connect a potentiometer between
3.3 V and GND with its wiper on GPIO5. Never apply a voltage outside 0–3.3 V.
GPIO5 is averaged over four 12-bit samples and a rolling window of eight
updates (roughly 128 ms), then mapped from raw ADC values 0–4095 to 0–10 bar.
The first reading is sent without waiting for the window to fill; after an ADC
error or interruption the average starts fresh. This direct mapping is
approximate, not a calibrated
pressure measurement: the ESP32-S3 ADC's specified calibrated range at the
highest attenuation ends around 2.9 V, and the upper range can saturate early.
Floating inputs have no defined reading. When subscribed by the display the
companion sends CRC-protected centibar values approximately every 16 ms,
skipping a sample if the UART transmit buffer is full. Streaming stops when
the subscription expires or Mainboard OTA begins.

The display can send a firmware image over the same UART. The receiver forces
all outputs LOW, writes to the inactive OTA partition, checks the SHA-256,
validates the image and only then selects it for boot. A timeout or failed
check aborts the transfer without selecting the new partition. CRC16 protects
individual frames, but UART access is not authenticated. Do not expose the
UART to untrusted devices.

The release workflow builds a tag `vX.Y.Z`, publishes `firmware.bin`, and
updates `ota/manifest.json` on `main`. Publish this project and a release
before the display can download a Mainboard update. Set the three version
components in `include/mainboard_version.h` to match the release tag; the
workflow rejects a mismatched tag. Each component must be between 0 and 255.
The display reads the installed version over UART; older firmware does not
report a version and is shown as unknown. The initial OTA-capable
firmware must first be flashed over USB.
