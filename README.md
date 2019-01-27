# SX1257 PMOD

This is a PMOD built around the SX1257 tranceiver. It uses an I2C-SPI bridge (SC18IS602B) since only 8 io pins are used.

RevA Pin-out:
- 1: I2C CLK
- 2: I2C SDA
- 3: CLK_IN (32 or 36 MHz)
- 4: CLK_OUT (36 MHz)
- 5: I_IN
- 6: Q_IN
- 7: Q_OUT
- 8: I_OUT


## RevA errata
- Change crystal footprint.
- Add 2k pull-ups on i2c sck and sda.
- Maybe swap pin 7 and 8, makese sense to keep in and out symmetric.