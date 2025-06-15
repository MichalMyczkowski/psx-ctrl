#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/spi.h>
#include <stdio.h>

// TIMER

const struct device *const counter_dev = DEVICE_DT_GET(DT_ALIAS(psx_timer));
struct counter_alarm_cfg alarm_cfg;
struct counter_alarm_cfg alarm_cfg2;

// GPIOS

const struct device *gpio_dev = DEVICE_DT_GET(DT_ALIAS(psx_gpio));

#define ACK_PIN 2
#define ACK_PIN_FLAGS GPIO_OUTPUT

atomic_t psx_bt_mask = ATOMIC_INIT(0x0000);

// SPI

#define SPI_MODE SPI_OP_MODE_SLAVE | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_LSB | \
                     SPI_WORD_SET(8) | SPI_LINES_SINGLE

static const struct device *spi_dev = DEVICE_DT_GET(DT_ALIAS(my_spi));
static const struct spi_config spi_cfg = {
    .operation = SPI_MODE};

// SPI buffers
#define TRANSMISSION_LEN 5
static uint8_t transmit[] = {0x00, 0xBE, 0xA5, 0x00, 0x00};
static uint8_t receive[] = {0x00, 0x00, 0x00, 0x00, 0x00};

struct spi_buf tx_bufs = {
    .buf = transmit,
    .len = sizeof(transmit),
};

struct spi_buf rx_bufs = {
    .buf = receive,
    .len = sizeof(receive),
};

static struct spi_buf_set txbs = {
    .buffers = &tx_bufs,
    .count = 1,
};

static struct spi_buf_set rxbs = {
    .buffers = &rx_bufs,
    .count = 1,
};

// ACK LINE handler

#define FALL_CHANNEL 0
#define RISE_CHANNEL 1

static void counter_rising_cb(const struct device *counter_dev,
                              uint8_t chan_id, uint32_t ticks,
                              void *user_data)
{
    alarm_cfg.ticks = counter_us_to_ticks(counter_dev, 25);
    gpio_pin_set(gpio_dev, ACK_PIN, 0);
    counter_set_channel_alarm(counter_dev, FALL_CHANNEL, &alarm_cfg);
}

static int ack_cnt = 0;
static void counter_falling_cb(const struct device *counter_dev,
                               uint8_t chan_id, uint32_t ticks,
                               void *user_data)
{
    if (ack_cnt < TRANSMISSION_LEN - 1)
    {
        ack_cnt += 1;
        gpio_pin_set(gpio_dev, ACK_PIN, 1);
        counter_set_channel_alarm(counter_dev, RISE_CHANNEL, &alarm_cfg2);
    }
    else
    {
        ack_cnt = 0;
    }
}

// ATTENTION

static struct gpio_callback attention_cb_data;

void attention_cb(const struct device *dev,
                  struct gpio_callback *cb,
                  uint32_t pins)
{
    atomic_val_t snapshot = atomic_get(&psx_bt_mask);
    transmit[3] = snapshot & 0xFF;
    transmit[4] = (snapshot >> 8) & 0xFF;
    spi_transceive_cb(
        spi_dev,  // const struct device * 	dev,
        &spi_cfg, // const struct spi_config * 	config,
        &txbs,    // const struct spi_buf_set * 	tx_bufs,
        &rxbs,    // const struct spi_buf_set * 	rx_bufs,
        NULL,
        NULL);
    alarm_cfg.ticks = counter_us_to_ticks(counter_dev, 20);
    counter_set_channel_alarm(counter_dev, FALL_CHANNEL, &alarm_cfg);
}

int psx_init()
{
    if (!device_is_ready(spi_dev))
    {
        printk("SPI 0 device not ready!\n");
        return -1;
    }
    if (!device_is_ready(gpio_dev))
    {
        printk("GPIO device not ready!\n");
        return -1;
    }

    if (!device_is_ready(counter_dev))
    {
        printk("COUNTER device not ready!\n");
        return -1;
    }

    // set up ATTENTION callback
    gpio_init_callback(&attention_cb_data, attention_cb, BIT(3));
    gpio_add_callback(gpio_dev, &attention_cb_data);
    gpio_pin_interrupt_configure(gpio_dev, 3, GPIO_INT_EDGE_FALLING);

    int ret = gpio_pin_configure(gpio_dev, ACK_PIN, GPIO_OUTPUT);
    if (ret)
    {
        printk("not working gpio %d", ret);
        return -1;
    }

    // SET UP COUNTER CALLBACKS
    counter_start(counter_dev);

    alarm_cfg.flags = 0;
    alarm_cfg.ticks = counter_us_to_ticks(counter_dev, 20);
    alarm_cfg.callback = counter_falling_cb;
    alarm_cfg.user_data = &alarm_cfg;
    alarm_cfg2.flags = 0;
    alarm_cfg2.ticks = counter_us_to_ticks(counter_dev, 3);
    alarm_cfg2.callback = counter_rising_cb;
    alarm_cfg2.user_data = &alarm_cfg2;

    return 0;
}

// -----------------------------------------------------------------------------
// BLE
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#define PSX_SERVICE_UUID \
    BT_UUID_128_ENCODE(0x104d5838, 0x18c2, 0x490b, 0x967e, 0x2b7ebb647fd5)

static struct bt_uuid_128 psx_service_uuid = BT_UUID_INIT_128(PSX_SERVICE_UUID);

static const struct bt_uuid_128 psx_buttons_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x908fd10a, 0x50a9, 0x44b4, 0x9cac, 0x9526c0f6b250));

static ssize_t write_psx(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    uint16_t btn_mask;
    if (offset != 0)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    else if (len > sizeof(btn_mask))
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    (void)memcpy(&btn_mask, buf, len);

    atomic_set(&psx_bt_mask, btn_mask);
    return len;
}

/* Simple IO Service Declaration */
BT_GATT_SERVICE_DEFINE(simple_io_svc,
                       BT_GATT_PRIMARY_SERVICE(&psx_service_uuid),
                       BT_GATT_CHARACTERISTIC(&psx_buttons_uuid.uuid,
                                              BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_psx, NULL),
                       BT_GATT_CUD("PSX button bitmap", BT_GATT_PERM_READ), );

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, PSX_SERVICE_UUID),
};

static void bt_ready(void)
{
    int err;

    printk("%s: Bluetooth initialized\n", __func__);

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err)
    {
        printk("%s: Advertising failed to start (err %d)\n", __func__, err);
        return;
    }

    printk("%s: Advertising successfully started\n", __func__);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        printk("%s: Connection failed (err 0x%02x)\n", __func__, err);
    }
    else
    {
        printk("%s: Connected\n", __func__);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("%s: Disconnected (reason 0x%02x)\n", __func__, reason);

    // start advertising again
    int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        printk("%s: Advertising failed to start (err %d)\n", __func__, err);
        return;
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

int main()
{

    int err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    bt_ready();

    psx_init();

    while (1)
    {
        k_sleep(K_FOREVER);
    }
    return 0;
}