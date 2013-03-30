import quick2wire.i2c as i2c
from quick2wire.i2c import I2CMaster, writing_bytes, reading
import time
import math

class MS5607:
	"""
	Instantiable class representing a MS5607 sensor.
	Get the latest at:
	https://github.com/rsolomon/py-MS5607

	Ported from http://code.google.com/p/ardroneme/
	"""

	_CMD_RESET = 0x1E
	_CMD_ADC_READ = 0x00
	_CMD_ADC_CONV = 0x40
	_CMD_ADC_D1 = 0x00
	_CMD_ADC_D2 = 0x10
	_CMD_ADC_256 = 0x00
	_CMD_ADC_512 = 0x02
	_CMD_ADC_1024 = 0x04
	_CMD_ADC_2048 = 0x06
	_CMD_ADC_4096 = 0x08
	_CMD_PROM_RD = 0xA0

	def __init__(self, address):
		""" MS5607 sensor constructor.

		Keyword arguments:
		address -- i2c address of the MS5607 sensor.
		"""

		self._address = address
		self.reset_sensor()
		self._coefficients = self.__read_coefficients()

	def __read_coefficient(self, coef_num):
		with I2CMaster() as master:
			results = master.transaction(
				writing_bytes(self._address, MS5607._CMD_PROM_RD + coef_num * 2), # Send PROM READ command
				reading(self._address, 2))

		rC = 256 * results[0][0] # read MSB
		rC = rC + results[0][1]  # read LSB
		return rC

	def __read_coefficients(self):
		coefficients = [0] * 6
		for n in range(0, 6):
			coefficients[n] = self.__read_coefficient(n + 1)
		return coefficients

	def __read_adc(self, cmd):
		with I2CMaster() as master:
			master.transaction(
				writing_bytes(self._address, (MS5607._CMD_ADC_CONV | cmd)))  # Send conversion command

		# Map of times to delay for conversions
		delay_time = {}
		delay_time[MS5607._CMD_ADC_256] = 0.001
		delay_time[MS5607._CMD_ADC_512] = 0.003
		delay_time[MS5607._CMD_ADC_1024] = 0.004
		delay_time[MS5607._CMD_ADC_2048] = 0.006
		delay_time[MS5607._CMD_ADC_4096] = 0.01

		time.sleep(delay_time[cmd & 0x0f]) # Wait necessary conversion time

		with I2CMaster() as master:
			read_bytes = master.transaction(
				writing_bytes(self._address, MS5607._CMD_ADC_READ),
				reading(address, 3))

		tmp = 65536 * read_bytes[0][0] # Read MSB
		tmp = tmp + 256 * read_bytes[0][1] # Read byte
		tmp = tmp + read_bytes[0][2] # Read LSB
		return tmp

	def __convert_pressure_temperature(self, pressure, temperature):

		# Calculate 1st order pressure and temperature
		dT = temperature - self._coefficients[4] * 256

		# Offset at actual temperature
		off = self._coefficients[1] * 4 + ((float(dT) / 2048) * (float(self._coefficients[3]) / 1024))

		# Sensitivity at actual temperature
		sens = self._coefficients[0] * 2 + ((float(dT) / 4096) * (float(self._coefficients[2]) / 1024))

		# Temperature compensated pressure
		press = (float(pressure) / 2048) * (float(sens) / 1024) - off
		return press

	def __pascal_to_cm(self, pressure_pa):

		# Lookup table converting pressure in Pa to altitude in cm.
        # Each LUT entry is the altitude in cm corresponding to an implicit
        # pressure value, calculated as [PA_INIT - 1024*index] in Pa.
        # The table is calculated for a nominal sea-level pressure  = 101325 Pa.
		pzlut_entries = 77
		pa_init = 104908
		pa_delta = 1024

		lookup_table = [
			-29408, -21087, -12700,  -4244,   4279,
			12874,  21541,  30281,  39095,  47986,
			56953,  66000,  75126,  84335,  93628,
			103006, 112472, 122026, 131672, 141410,
			151244, 161174, 171204, 181335, 191570,
			201911, 212361, 222922, 233597, 244388,
			255300, 266334, 277494, 288782, 300204,
			311761, 323457, 335297, 347285, 359424,
			371719, 384174, 396795, 409586, 422552,
			435700, 449033, 462560, 476285, 490216,
			504360, 518724, 533316, 548144, 563216,
			578543, 594134, 609999, 626149, 642595,
			659352, 676431, 693847, 711615, 729752,
			748275, 767202, 786555, 806356, 826627,
			847395, 868688, 890537, 912974, 936037,
			959766, 984206]

		if pressure_pa > pa_init:
			return lookup_table[0]

		inx = int(pa_init - pressure_pa) >> 10
		if inx >= pzlut_entries - 1:
			return lookup_table[pzlut_entries - 1]

		pa1 = pa_init - (inx << 10)
		z1 = lookup_table[inx]
		z2 = lookup_table[inx + 1]
		return (z1 + ((int(pa1 - pressure_pa) * (z2 - z1)) >> 10))

	def reset_sensor(self):
		with I2CMaster() as master:
			master.transaction(
	    		writing_bytes(self._address, MS5607._CMD_RESET))

	def get_altitude(self, samples=48):
		accum = 0
		for n in range(0, samples):
			time.sleep(0.001)
			temperature = self.__read_adc(MS5607._CMD_ADC_D2 | MS5607._CMD_ADC_4096)
			pressure = self.__read_adc(MS5607._CMD_ADC_D1 | MS5607._CMD_ADC_4096)
			press_conv = self.__convert_pressure_temperature(pressure, temperature)
			accum += press_conv

		avg = accum / samples
		return self.__pascal_to_cm(avg)
