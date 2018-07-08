# **Electric Eagles(1781) FRC 2018 RoboRio Code**
The Eclipse project should be in this folder along with any documentation.  This includes sketches of algorithms, Hardware Mappings and technical drawings required to program the RoboRio.

**JAVA Programming**
* https://wpilib.screenstepslive.com/s/currentCS/m/java

**Git Ignore for eclipse Projects**
* https://gist.github.com/chhh/4961200

**Talon SRX Firmware**
* http://www.ctr-electronics.com/downloads/firm/TalonSRX-Application-3.3-FRC2018.zip

**Phoenix Life Boat**
* http://www.ctr-electronics.com/downloads/installers/CTRE%20Phoenix%20Framework%20v5.1.3.1.zip

## FRC 2018 IP Addressing 

##### Radio: 10.17.81.1 

##### RoboRIO should be found through mDNS:
- roboRIO-1781-FRC.local

##### Everything else is assigned by robot radio:
- DHCP range: 10.17.81.20 - 10.17.81.199

### Out static IP alternive (all with netmask of /24)
- Radio: 10.17.81.1
- RoboRIO: 10.17.81.2
- Driver Station: 10.17.81.5 *(subnet mask must be /8)*
- Raspberry Pi: 10.17.81.13
- Nvidia Jetson: 10.17.81.7
- Other devices: 10.17.81.6 - 10.17.81.19
    - If we get an IP camera we'll use: 10.17.81.11
