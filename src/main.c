/*
 * XIAO nRF54L15 Sense: IMU (LSM6DS3TR-C) mit 100 Hz auslesen und
 *   1. als CSV über USB-Seriell ausgeben (115200 Baud, zum Aufzeichnen) und
 *   2. per Bluetooth LE live streamen (für die Live-Anzeige).
 *
 * Seriell, eine Zeile pro Messung:  t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps
 * Zeilen, die mit '#' beginnen, sind Kommentare.
 * Die Zeilen werden in Schüben zu SERIAL_BATCH Zeilen gesendet: Die USB-Brücke
 * des XIAO leitet Daten nur dann sofort weiter, wenn die Leitung zwischendurch
 * > ca. 10 ms ruht. Bei gleichmäßigen 100 Zeilen/s sammelt sie sonst 16 KiB
 * (ca. 4 s) bevor etwas am Mac ankommt.
 *
 * BLE: Gerätename "XIAO-IMU", Service IMU_SVC_UUID, Characteristic
 * IMU_DATA_UUID (notify). Ein Paket (little endian):
 *   uint32 t_ms        Zeitstempel der ersten Messung im Paket
 *   uint8  n           Anzahl Messungen (Abstand jeweils 1000/SAMPLE_RATE_HZ ms)
 *   n x 6 x int16      ax,ay,az in mg; gx,gy,gz in 0,1 Grad/s
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#define SAMPLE_RATE_HZ   100   /* Ausgaberate */
#define IMU_ODR_HZ       208   /* interne Messrate des Sensors (>= 2x Ausgaberate) */
#define ACCEL_RANGE_G    16    /* +-2, 4, 8 oder 16 g; Fersenaufprall kann > 8 g sein */
#define GYRO_RANGE_DPS   2000  /* +-245, 500, 1000 oder 2000 Grad/s */
#define BLE_BATCH        5     /* Messungen pro BLE-Paket -> 20 Pakete/s */
#define SERIAL_BATCH     2     /* Zeilen pro seriellem Schub, siehe unten */

#define SAMPLE_PERIOD_MS (1000 / SAMPLE_RATE_HZ)
#define G_TO_MS2         9.80665
#define RAD_TO_DEG       57.29577951308232
#define PKT_HDR          5
#define PKT_SAMPLE       12

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu0));

/* ---------------- Bluetooth ---------------- */

/* 7a1e0001-5c3b-4f0a-9d2e-3c8f1b6a4d01 */
#define IMU_SVC_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0001, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
#define IMU_DATA_UUID_VAL \
	BT_UUID_128_ENCODE(0x7a1e0002, 0x5c3b, 0x4f0a, 0x9d2e, 0x3c8f1b6a4d01)
static const struct bt_uuid_128 imu_svc_uuid = BT_UUID_INIT_128(IMU_SVC_UUID_VAL);
static const struct bt_uuid_128 imu_data_uuid = BT_UUID_INIT_128(IMU_DATA_UUID_VAL);

static struct bt_conn *cur_conn;
static atomic_t notify_on;
static atomic_t need_adv = ATOMIC_INIT(1);

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	atomic_set(&notify_on, value == BT_GATT_CCC_NOTIFY);
	printk("# BLE: Streaming %s\n", value == BT_GATT_CCC_NOTIFY ? "an" : "aus");
}

BT_GATT_SERVICE_DEFINE(imu_svc,
	BT_GATT_PRIMARY_SERVICE(&imu_svc_uuid),
	BT_GATT_CHARACTERISTIC(&imu_data_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, IMU_SVC_UUID_VAL),
};
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("# BLE: Verbindung fehlgeschlagen (0x%02x)\n", err);
		atomic_set(&need_adv, 1);
		return;
	}
	cur_conn = bt_conn_ref(conn);
	printk("# BLE: verbunden\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("# BLE: getrennt (0x%02x)\n", reason);
	atomic_set(&notify_on, 0);
	if (cur_conn) {
		bt_conn_unref(cur_conn);
		cur_conn = NULL;
	}
}

static void recycled(void)
{
	atomic_set(&need_adv, 1);   /* Verbindung freigegeben -> wieder werben */
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
};

static void adv_start_if_needed(void)
{
	if (!atomic_cas(&need_adv, 1, 0)) {
		return;
	}
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY) {
		printk("# BLE: Advertising-Fehler %d\n", err);
		atomic_set(&need_adv, 1);   /* beim nächsten Durchlauf erneut */
	} else {
		printk("# BLE: sichtbar als \"%s\"\n", CONFIG_BT_DEVICE_NAME);
	}
}

/* Messungen sammeln und als ein Paket verschicken */
static uint8_t pkt[PKT_HDR + BLE_BATCH * PKT_SAMPLE];
static uint8_t pkt_n;

static void ble_push(uint32_t t_ms, const int16_t v[6])
{
	if (!cur_conn || !atomic_get(&notify_on)) {
		pkt_n = 0;
		return;
	}
	/* Wie viele Messungen passen bei der ausgehandelten MTU in ein Paket? */
	uint16_t room = bt_gatt_get_mtu(cur_conn) - 3;
	uint8_t max_n = MIN(BLE_BATCH, (room - PKT_HDR) / PKT_SAMPLE);

	if (max_n == 0) {
		return;
	}
	if (pkt_n == 0) {
		sys_put_le32(t_ms, &pkt[0]);
	}
	uint8_t *p = &pkt[PKT_HDR + pkt_n * PKT_SAMPLE];

	for (int i = 0; i < 6; i++) {
		sys_put_le16((uint16_t)v[i], p + 2 * i);
	}
	if (++pkt_n >= max_n) {
		pkt[4] = pkt_n;
		int err = bt_gatt_notify(cur_conn, &imu_svc.attrs[1], pkt,
					 PKT_HDR + pkt_n * PKT_SAMPLE);
		if (err && err != -ENOMEM) {
			printk("# BLE: notify-Fehler %d\n", err);
		}
		pkt_n = 0;
	}
}

/* ---------------- IMU ---------------- */

/* serielle Zeilen sammeln und als Schub ausgeben */
static char ser_buf[SERIAL_BATCH * 64];
static size_t ser_len;
static uint8_t ser_n;

static void serial_push(uint32_t t, const double a[3], const double g[3])
{
	int len = snprintk(&ser_buf[ser_len], sizeof(ser_buf) - ser_len,
			   "%u,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n", t,
			   a[0], a[1], a[2], g[0], g[1], g[2]);

	if (len > 0 && ser_len + len < sizeof(ser_buf)) {
		ser_len += len;
	}
	if (++ser_n >= SERIAL_BATCH) {
		printk("%s", ser_buf);
		ser_len = 0;
		ser_n = 0;
		ser_buf[0] = '\0';
	}
}

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

	int err = bt_enable(NULL);

	if (err) {
		printk("# Fehler: Bluetooth-Start (%d), nur seriell\n", err);
		atomic_set(&need_adv, 0);
	}

	printk("# XIAO nRF54L15 IMU live, %d Hz, +-%d g, +-%d dps\n",
	       SAMPLE_RATE_HZ, ACCEL_RANGE_G, GYRO_RANGE_DPS);
	printk("# t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n");

	int64_t next = k_uptime_get();

	while (1) {
		adv_start_if_needed();

		uint32_t t = (uint32_t)k_uptime_get();

		if (sensor_sample_fetch(imu) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyr) == 0) {
			double a[3], g[3];
			int16_t raw[6];

			for (int i = 0; i < 3; i++) {
				a[i] = sensor_value_to_double(&acc[i]) / G_TO_MS2;
				g[i] = sensor_value_to_double(&gyr[i]) * RAD_TO_DEG;
				raw[i] = (int16_t)CLAMP(a[i] * 1000.0, INT16_MIN, INT16_MAX);
				raw[3 + i] = (int16_t)CLAMP(g[i] * 10.0, INT16_MIN, INT16_MAX);
			}
			ble_push(t, raw);
			serial_push(t, a, g);
		} else {
			printk("# Lesefehler\n");
		}

		/* LED: langsam blinken = wartet auf BLE, schnell = streamt */
		uint32_t blink = atomic_get(&notify_on) ? SAMPLE_RATE_HZ / 10 : SAMPLE_RATE_HZ / 2;

		if (++n % blink == 0) {
			gpio_pin_toggle_dt(&led);
		}

		/* festen Takt halten, unabhängig von der Laufzeit der Schleife */
		next += SAMPLE_PERIOD_MS;
		k_sleep(K_TIMEOUT_ABS_MS(next));
	}
	return 0;
}
