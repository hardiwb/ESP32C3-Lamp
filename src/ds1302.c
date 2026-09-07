#include "ds1302.h"

#include <string.h>

#include "rom/ets_sys.h"

#define DS1302_WRITE_SECONDS 0x80
#define DS1302_WRITE_PROTECT 0x8e
#define DS1302_WRITE_TRICKLE 0x90
#define DS1302_WRITE_BURST 0xbe
#define DS1302_READ_BURST 0xbf

static gpio_num_t rtc_clk;
static gpio_num_t rtc_data;
static gpio_num_t rtc_rst;

static void pulse_delay(void)
{
    ets_delay_us(1);
}

static void transaction_start(void)
{
    gpio_set_level(rtc_clk, 0);
    gpio_set_direction(rtc_data, GPIO_MODE_OUTPUT);
    gpio_set_level(rtc_rst, 1);
    pulse_delay();
}

static void transaction_end(void)
{
    gpio_set_level(rtc_rst, 0);
    gpio_set_level(rtc_clk, 0);
    gpio_set_direction(rtc_data, GPIO_MODE_INPUT);
    pulse_delay();
}

static void write_byte(uint8_t value)
{
    gpio_set_direction(rtc_data, GPIO_MODE_OUTPUT);
    for (int bit = 0; bit < 8; bit++) {
        gpio_set_level(rtc_data, (value >> bit) & 1);
        pulse_delay();
        gpio_set_level(rtc_clk, 1);
        pulse_delay();
        gpio_set_level(rtc_clk, 0);
    }
}

static uint8_t read_byte(void)
{
    uint8_t value = 0;
    gpio_set_direction(rtc_data, GPIO_MODE_INPUT);
    for (int bit = 0; bit < 8; bit++) {
        if (gpio_get_level(rtc_data)) value |= (uint8_t)(1U << bit);
        gpio_set_level(rtc_clk, 1);
        pulse_delay();
        gpio_set_level(rtc_clk, 0);
        pulse_delay();
    }
    return value;
}

static void write_register(uint8_t command, uint8_t value)
{
    transaction_start();
    write_byte(command);
    write_byte(value);
    transaction_end();
}

static uint8_t int_to_bcd(int value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static bool bcd_to_int(uint8_t value, int maximum, int *result)
{
    int high = (value >> 4) & 0x0f;
    int low = value & 0x0f;
    int decoded = high * 10 + low;
    if (high > 9 || low > 9 || decoded > maximum) return false;
    *result = decoded;
    return true;
}

void ds1302_init(gpio_num_t clk_pin, gpio_num_t data_pin, gpio_num_t rst_pin)
{
    rtc_clk = clk_pin;
    rtc_data = data_pin;
    rtc_rst = rst_pin;

    gpio_reset_pin(rtc_clk);
    gpio_reset_pin(rtc_data);
    gpio_reset_pin(rtc_rst);
    gpio_set_direction(rtc_clk, GPIO_MODE_OUTPUT);
    gpio_set_direction(rtc_rst, GPIO_MODE_OUTPUT);
    gpio_set_level(rtc_clk, 0);
    gpio_set_level(rtc_rst, 0);

    write_register(DS1302_WRITE_PROTECT, 0x00);
    write_register(DS1302_WRITE_TRICKLE, 0x00); /* Never charge a CR2032. */
    write_register(DS1302_WRITE_PROTECT, 0x80);
}

bool ds1302_read_time(struct tm *local_time)
{
    uint8_t data[8];
    transaction_start();
    write_byte(DS1302_READ_BURST);
    for (size_t i = 0; i < sizeof(data); i++) data[i] = read_byte();
    transaction_end();

    if ((data[0] & 0x80) != 0 || (data[2] & 0x80) != 0) return false;
    int second, minute, hour, day, month, year;
    if (!bcd_to_int(data[0] & 0x7f, 59, &second) ||
        !bcd_to_int(data[1] & 0x7f, 59, &minute) ||
        !bcd_to_int(data[2] & 0x3f, 23, &hour) ||
        !bcd_to_int(data[3] & 0x3f, 31, &day) || day < 1 ||
        !bcd_to_int(data[4] & 0x1f, 12, &month) || month < 1 ||
        !bcd_to_int(data[6], 99, &year)) return false;

    memset(local_time, 0, sizeof(*local_time));
    local_time->tm_sec = second;
    local_time->tm_min = minute;
    local_time->tm_hour = hour;
    local_time->tm_mday = day;
    local_time->tm_mon = month - 1;
    local_time->tm_year = 100 + year;
    local_time->tm_isdst = -1;
    return true;
}

bool ds1302_write_time(const struct tm *local_time)
{
    if (local_time->tm_year < 100 || local_time->tm_year > 199) return false;
    write_register(DS1302_WRITE_PROTECT, 0x00);
    write_register(DS1302_WRITE_TRICKLE, 0x00);

    transaction_start();
    write_byte(DS1302_WRITE_BURST);
    write_byte(int_to_bcd(local_time->tm_sec) & 0x7f);
    write_byte(int_to_bcd(local_time->tm_min));
    write_byte(int_to_bcd(local_time->tm_hour));
    write_byte(int_to_bcd(local_time->tm_mday));
    write_byte(int_to_bcd(local_time->tm_mon + 1));
    write_byte(int_to_bcd(local_time->tm_wday + 1));
    write_byte(int_to_bcd(local_time->tm_year - 100));
    write_byte(0x80);
    transaction_end();

    struct tm verify;
    return ds1302_read_time(&verify) &&
           verify.tm_year == local_time->tm_year &&
           verify.tm_mon == local_time->tm_mon &&
           verify.tm_mday == local_time->tm_mday &&
           verify.tm_hour == local_time->tm_hour &&
           verify.tm_min == local_time->tm_min;
}
