# BNO055

Adafruit's BNO055 sensor is a very accurate and powerful sensor, but Adafruit's own code for it doesn't work. The output of their Euler angles using event.orientation.x() doesn't give accurate readings. You have to set the sensor mode to NDOF mode, use quaternions and convert them to euler angles for accurate readings (the datasheet goes into detail about modes in section 3.3). This codes implements Adafruit's persistent calibration data using the EEPROM, it gives accurate readings using the method I just described, and it fixes Adafruit's bunny example (no need to change the Processing code!)

## Recommended Resources
* [Adafruit's documentation](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
* [Adafruit's Github](https://github.com/adafruit/Adafruit_BNO055)
* [Bosch's Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
