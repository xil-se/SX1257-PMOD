# SX1257 PMOD

This is a PMOD built around the SX1257 transceiver. It uses an I2C-SPI bridge (SC18IS602B) since only 8 io pins are used.

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

## RevB errata

The pins VR_PA, VR_DIG, VR_ANA1 and VR_ANA2 should *not* be tied to VBAT/VCC, which they are. These are pins for decoupling for the internal voltage regulators, i.e. they should *only* be connected to single decoupling capacitors as stated in the datasheet.

- VR_PA: 10nF
- VR_DIG: 100nF
- VR_ANA1: 100nF
- VR_ANA2: 100nF

RX performance is poor. Why? Maybe related to the above, or bad matching circuit?

In the build of 10 units, 2 failed completely and 2 had very bad RX performance. Maybe related to the issue above.


## Changes for RevC / TODO:

- [x] Fix SX1257 Vreg decoupling issue. Done in #1 / 176024d
- [x] Change to 0402 components. Done in #1 / 176024d
- [ ] Add optional SI514 I2C-programmable oscillator (it's WIP on the branch si514)
