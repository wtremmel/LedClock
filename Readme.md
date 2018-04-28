# LedClock
## Idea
What else to do with these nice addressable RGB LEDs?
Since they are also available as a quarter circle, and strangely enough 15 of them (so 60 for a full circle) the idea of building a clock was born.

## Hardware
All hardware is cheaply available at the usual places. There are alternatives for some parts possible (see list below)

* CPU: **ESP8266**
  * I am using the D1 mini form factor here
  * alternatively an Arduino can also be used (without WIFI then)
* LEDs:
  * 60 **WS2812** LEDs. You can either get a circle pre assembled or (cheaper) 4 quarter circles with 15 LEDs each
* Sensors:
  If you want the clock also to display temperature and humidity you need a sensor for that. Also if the brightness should auto-adjust you need another sensor.
  * **BME280** for temperature, humidity (and air pressure)
  * **TSL2561** for light level
* Real time clock: To keep the time a battery buffered real time clock is needed. There are several options.
  * **DS3231** real time clock
* Touchpad: To operate the clock by touch of a finger I found this nice breakout board where you can attach 12 touch points:
  * **MPR121** touch controller

Also you need something to mount the clock on. Thats up to you and your skills. I used a cheap plastic transparent sheet holder for that.

## Software
