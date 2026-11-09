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

The display can send a firmware image over the same UART. The receiver forces
all outputs LOW, writes to the inactive OTA partition, checks the SHA-256,
validates the image and only then selects it for boot. A timeout or failed
check aborts the transfer without selecting the new partition. CRC16 protects
individual frames, but UART access is not authenticated. Do not expose the
UART to untrusted devices.

The release workflow builds a tag `vX.Y.Z`, publishes `firmware.bin`, and
updates `ota/manifest.json` on `main`. Publish this project and a release
before the display can download a Mainboard update. The display offers the
published version for installation even if it is already installed; it does
not yet read the installed version from the Mainboard. The initial OTA-capable
firmware must first be flashed over USB.
