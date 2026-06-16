# LARA_V2

## Raspberry Pi Initial Configuration

### Step 1: Raspberry Pi OS

First flash the latest Raspberry Pi OS image to your preferred storage device.
The official Raspberry Pi Imager can be used for that.

Then perform an update
```bash
sudo apt update
```
### Step 2: Configure the CAN Hat

Add the following lines to /boot/firmware/config.txt
```bash
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25,spimaxfrequency=1000000
```

Reboot
```bash
sudo reboot
```

See if initialization was successful
```bash
dmesg | grep -i '\(can\|spi\)'
```
It should output something like this:
```bash
[   20.892310] CAN device driver interface
[   20.915484] mcp251x spi0.0 can0: MCP2515 successfully initialized
```

Activate the CAN interface (example: 500000 baud)
```bash
sudo ip link set can0 up type can bitrate 500000
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can0 up
```

Optional: Activate CAN interface on boot

Add the following to ~/.bashrc
```bash
sudo ip link set can0 up type can bitrate 500000
sudo ifconfig can0 txqueuelen 65536
sudo ifconfig can0 up
```

### Stap 3: Configure the serial port

## Steadywin GIM6010 Initial Configuration

### With ODriveTool

```bash
odrv0.axis0.motor.config.current_lim = 3
odrv0.config.dc_bus_undervoltage_trip_level = 10

odrv0.axis0.config.can.heartbeat_rate_ms = 0
odrv0.axis0.config.can.encoder_rate_ms = 0

odrv0.save_configuration()
```
