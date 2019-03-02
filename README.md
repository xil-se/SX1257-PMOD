# SX1257 PMOD

This is a PMOD built around the SX1257 tranceiver. It uses an I2C-SPI bridge (SC18IS602B) since only 8 io pins are used.

RevA Pin-out:
- 1: I2C SCL
- 2: I2C SDA
- 3: CLK_IN (32 or 36 MHz)
- 4: CLK_OUT (36 MHz)
- 5: I_IN
- 6: Q_IN
- 7: Q_OUT
- 8: I_OUT


## RevA errata
- Change crystal footprint.
- Add 2k pull-ups on i2c scl and sda.
- Maybe swap pin 7 and 8, makese sense to keep in and out symmetric.

# RevB

RevB Pin-out:
- 1: I_IN
- 2: Q_IN
- 3: Q_OUT
- 4: I_OUT
- 5: I2C SCL
- 6: I2C SDA
- 7: CLK_IN (32 or 36 MHz)
- 8: CLK_OUT (36 MHz)

Changes/TODO:

- [ ] Fix crystal footprint
- [x] Swap PMOD rows so CLK_OUT actually ties to P1A10/IOB_3B_G6 on the Icebreaker
- [x] Add 2k2 pull-ups on I2C SCL and SDA.
- [ ] Add optional SI514 I2C-programmable oscillator