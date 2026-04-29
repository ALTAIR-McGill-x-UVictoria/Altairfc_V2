#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#define INA3221_ADDR            0x40

/* Register map */
#define REG_CONFIG              0x00
#define REG_CH1_SHUNT           0x01
#define REG_CH1_BUS             0x02
#define REG_CH2_SHUNT           0x03
#define REG_CH2_BUS             0x04
#define REG_CH3_SHUNT           0x05
#define REG_CH3_BUS             0x06
#define REG_MANUF_ID            0xFE
#define REG_DIE_ID              0xFF

/*
 * Configuration: all 3 channels enabled, averaging 16 samples,
 * bus and shunt conversion time 1.1 ms, continuous mode.
 * [15]    RST=0
 * [14]    CH1_EN=1
 * [13]    CH2_EN=1
 * [12]    CH3_EN=1
 * [11:9]  AVG=001  (16 samples)
 * [8:6]   VBUS_CT=100 (1.1 ms)
 * [5:3]   VSH_CT=100  (1.1 ms)
 * [2:0]   MODE=111 (continuous shunt+bus)
 */
#define INA3221_CONFIG_DEFAULT  0x7127

/* LSBs per the datasheet */
#define SHUNT_LSB_UV            40.0f    /* 40 µV per LSB (shunt voltage) */
#define BUS_LSB_MV              8.0f     /* 8 mV per LSB (bus voltage) */

/* Shunt resistor value (ohms) */
#define SHUNT_OHMS              0.01f

typedef struct {
    float voltage_v[3];   /* bus voltage (V) per channel */
    float current_a[3];   /* current (A) per channel */
} INA3221Reading;

/* ------------------------------------------------------------------ */
/* I2C helpers — big-endian 16-bit register reads/writes              */
/* ------------------------------------------------------------------ */

static int reg_write(int fd, uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    struct i2c_msg msg = {
        .addr  = INA3221_ADDR,
        .flags = 0,
        .len   = 3,
        .buf   = buf,
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &xfer);
}

static int reg_read(int fd, uint8_t reg, uint16_t *out)
{
    uint8_t raw[2];
    struct i2c_msg msgs[2] = {
        { .addr = INA3221_ADDR, .flags = 0,        .len = 1, .buf = &reg },
        { .addr = INA3221_ADDR, .flags = I2C_M_RD, .len = 2, .buf = raw },
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
    if (ioctl(fd, I2C_RDWR, &xfer) < 0)
        return -1;
    *out = (uint16_t)((raw[0] << 8) | raw[1]);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int ina3221_open(const char *i2c_dev)
{
    int fd = open(i2c_dev, O_RDWR);
    if (fd < 0) return -1;

    if (ioctl(fd, I2C_SLAVE, INA3221_ADDR) < 0) {
        close(fd);
        return -1;
    }

    /* Verify manufacturer ID (should be 0x5449 = "TI") */
    uint16_t manuf_id;
    if (reg_read(fd, REG_MANUF_ID, &manuf_id) < 0 || manuf_id != 0x5449) {
        close(fd);
        return -1;
    }

    /* Apply configuration */
    if (reg_write(fd, REG_CONFIG, INA3221_CONFIG_DEFAULT) < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

/*
 * Read all three channels.
 * Returns  0: success, reading populated
 *         -1: I2C error
 */
int ina3221_read(int fd, INA3221Reading *reading)
{
    /* Shunt voltage registers for CH1–CH3 start at 0x01, stride 2 */
    static const uint8_t shunt_regs[3] = { REG_CH1_SHUNT, REG_CH2_SHUNT, REG_CH3_SHUNT };
    static const uint8_t bus_regs[3]   = { REG_CH1_BUS,   REG_CH2_BUS,   REG_CH3_BUS   };

    for (int ch = 0; ch < 3; ch++) {
        uint16_t shunt_raw, bus_raw;

        if (reg_read(fd, shunt_regs[ch], &shunt_raw) < 0) return -1;
        if (reg_read(fd, bus_regs[ch],   &bus_raw)   < 0) return -1;

        /* Shunt voltage: bits [15:3] are significant, sign-extended, LSB = 40 µV */
        int16_t shunt_signed = (int16_t)(shunt_raw & 0xFFF8);
        float shunt_uv = (float)(shunt_signed >> 3) * SHUNT_LSB_UV;

        /* Bus voltage: bits [15:3] are significant, unsigned, LSB = 8 mV */
        float bus_mv = (float)((bus_raw & 0xFFF8) >> 3) * BUS_LSB_MV;

        reading->voltage_v[ch] = bus_mv * 1e-3f;
        reading->current_a[ch] = (shunt_uv * 1e-6f) / SHUNT_OHMS;
    }
    return 0;
}

void ina3221_close(int fd)
{
    if (fd >= 0) close(fd);
}
