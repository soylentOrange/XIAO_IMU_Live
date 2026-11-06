/*
 * XIAO nRF54L15 Sense: IMU (LSM6DS3TR-C) auslesen und live als CSV
 * über die USB-Seriell-Schnittstelle (115200 Baud) ausgeben.
 *
 * Ausgabeformat, eine Zeile pro Messung:
 *   t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps
 * Zeilen, die mit '#' beginnen, sind Kommentare (Header, Fehler).
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#define SAMPLE_RATE_HZ   100   /* Ausgaberate */
#define IMU_ODR_HZ       208   /* interne Messrate des Sensors (>= 2x Ausgaberate) */
#define ACCEL_RANGE_G    16    /* +-2, 4, 8 oder 16 g; Fersenaufprall kann > 8 g sein */
#define GYRO_RANGE_DPS   2000  /* +-245, 500, 1000 oder 2000 Grad/s */

#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)
#define G_TO_MS2         9.80665
#define RAD_TO_DEG       57.29577951308232

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu0));

static int imu_configure(void)
{
	struct sensor_value v;
	int ret;

	/* Messbereiche (Treiber erwartet m/s^2 bzw. rad/s) */
	sensor_g_to_ms2(ACCEL_RANGE_G, &v);
	ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	if (ret) {
		printk("# Fehler: Accel-Messbereich (%d)\n", ret);
		return ret;
	}
	sensor_degrees_to_rad(GYRO_RANGE_DPS, &v);
	ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	if (ret) {
		printk("# Fehler: Gyro-Messbereich (%d)\n", ret);
		return ret;
	}

	/* Messrate: erst damit schaltet der Sensor ein */
	v.val1 = IMU_ODR_HZ;
	v.val2 = 0;
	ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
	if (ret) {
		printk("# Fehler: Accel-Messrate (%d)\n", ret);
		return ret;
	}
	ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
	if (ret) {
		printk("# Fehler: Gyro-Messrate (%d)\n", ret);
		return ret;
	}
	return 0;
}

int main(void)
{
	struct sensor_value acc[3], gyr[3];
	uint32_t n = 0;

	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}

	k_msleep(500); /* kurz warten, damit der Mac den Port geöffnet hat */

	if (!device_is_ready(imu)) {
		while (1) {
			printk("# Fehler: IMU nicht bereit\n");
			k_msleep(1000);
		}
	}
	if (imu_configure()) {
		return 0;
	}

	printk("# XIAO nRF54L15 IMU live, %d Hz, +-%d g, +-%d dps\n",
	       SAMPLE_RATE_HZ, ACCEL_RANGE_G, GYRO_RANGE_DPS);
	printk("# t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n");

	int64_t next = k_uptime_get();

	while (1) {
		int64_t t = k_uptime_get();

		if (sensor_sample_fetch(imu) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyr) == 0) {
			printk("%lld,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n", t,
			       sensor_value_to_double(&acc[0]) / G_TO_MS2,
			       sensor_value_to_double(&acc[1]) / G_TO_MS2,
			       sensor_value_to_double(&acc[2]) / G_TO_MS2,
			       sensor_value_to_double(&gyr[0]) * RAD_TO_DEG,
			       sensor_value_to_double(&gyr[1]) * RAD_TO_DEG,
			       sensor_value_to_double(&gyr[2]) * RAD_TO_DEG);
		} else {
			printk("# Lesefehler\n");
		}

		/* LED blinkt langsam als Lebenszeichen */
		if (++n % (SAMPLE_RATE_HZ / 2) == 0) {
			gpio_pin_toggle_dt(&led);
		}

		/* festen Takt halten, unabhängig von der Laufzeit der Schleife */
		next += SAMPLE_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
	return 0;
}
