# Variegated Controller Gear Pump Silvia

A proof-of-concept controller using Variegated-rs for my dev rig – a Rancilio Silvia that's been upgraded
with a gear pump, a flow meter, a pressure transducer, a PT100 temperature sensor, a SSD1309 OLED display
and a rotary encoder - all brought together by the APEC SoM and a custom carrier board.

While this is a proof-of-concept, it's also the actual controller for my Silvia. It's not *just* a proof-of-concept.

## Status

This is being developed in lockstep with the Variegated-rs library, but is split out from the repository
for to make that repository cleaner. Things can and will change, and things may move from this repository to
Variegated-rs. The overall goal is to be able to make this repository as small as possible, by moving 
general functionality to Variegated-rs.

Expect this code to be messier than the one in Variegated-rs.