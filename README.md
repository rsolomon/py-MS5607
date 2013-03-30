MS5607.py
=====================
A python port of the MS5607 wrapper found here:
https://github.com/cypherkey/RaspberryPi.Net

version
---------------------
0.0.1

usage
--------------------
```python
address = 0x76
sensor = MS5607(0x76)
alt = sensor.get_altitude(128)
print(str(alt))
```
