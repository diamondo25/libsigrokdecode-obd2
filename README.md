# libsigrokdecode-obd2

Internally called k-line, so don't be suprised if you cannot find OBD2. I only chose the name as its more common. Might introduce other decoders later.

Tested with PulseView 0.4.2 (as newer version didnt play nice with fx2lafw decoder) on Windows.

## Features
- Checksum validation
- Python API for additional processing (eg explaining what the K-Line command contains for the particular ECU)


## Usage

1. git clone this repo your PulseView decoders folder:
```
cd C:\Program Files (x86)\sigrok\PulseView\share\libsigrokdecode\decoders
git clone https://github.com/diamondo25/libsigrokdecode-obd2.git
```
2. Open PulseView, and add a K-Line decoder in combination with a UART connection. Common baudrates are 10400 and 9600.
