#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#define GPS_ADDR        0x42
#define REG_BYTES_HI    0xFD
#define REG_BYTES_LO    0xFE
#define REG_DATA        0xFF
#define MAX_I2C_READ    32

/* UBX-NAV-PVT: class=0x01, id=0x07, payload=92 bytes */
#define UBX_CLASS_NAV   0x01
#define UBX_ID_PVT      0x07
#define UBX_PVT_LEN     92

typedef struct {
    double   lat;          /* degrees, WGS84 */
    double   lon;          /* degrees */
    double   alt_msl;      /* meters MSL */
    float    speed_ms;     /* ground speed m/s */
    float    heading_deg;  /* course over ground, degrees */
    float    hdop;
    uint8_t  fix_type;     /* 0=no fix,1=DR,2=2D,3=3D,4=GNSS+DR */
    uint8_t  num_sv;
    uint8_t  valid;        /* 1 if gnssFixOK */
    uint8_t  _pad;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  time_valid;   /* UBX validFlags bitmask */
} GpsFix;

/* ------------------------------------------------------------------ */
/* Low-level I2C helpers                                               */
/* ------------------------------------------------------------------ */

static int i2c_write_byte(int fd, uint8_t reg, const uint8_t *data, int len)
{
    uint8_t buf[128];
    buf[0] = reg;
    if (len > 0 && len < (int)(sizeof(buf) - 1))
        memcpy(buf + 1, data, len);

    struct i2c_msg msg = {
        .addr  = GPS_ADDR,
        .flags = 0,
        .len   = (uint16_t)(1 + len),
        .buf   = buf,
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };
    return ioctl(fd, I2C_RDWR, &xfer);
}

static int i2c_read_reg(int fd, uint8_t reg, uint8_t *out, int len)
{
    struct i2c_msg msgs[2] = {
        { .addr = GPS_ADDR, .flags = 0,        .len = 1,   .buf = &reg },
        { .addr = GPS_ADDR, .flags = I2C_M_RD, .len = (uint16_t)len, .buf = out },
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
    return ioctl(fd, I2C_RDWR, &xfer);
}

static int ddc_bytes_available(int fd)
{
    uint8_t hi, lo;
    if (i2c_read_reg(fd, REG_BYTES_HI, &hi, 1) < 0) return -1;
    if (i2c_read_reg(fd, REG_BYTES_LO, &lo, 1) < 0) return -1;
    return (int)((hi << 8) | lo);
}

/* Read up to buf_len bytes from the DDC data register into buf.
   Returns number of bytes actually read, or -1 on error. */
static int ddc_read(int fd, uint8_t *buf, int want)
{
    int total = 0;
    while (want > 0) {
        int chunk = want < MAX_I2C_READ ? want : MAX_I2C_READ;
        if (i2c_read_reg(fd, REG_DATA, buf + total, chunk) < 0)
            return -1;
        total += chunk;
        want  -= chunk;
    }
    return total;
}

/* ------------------------------------------------------------------ */
/* UBX framing                                                         */
/* ------------------------------------------------------------------ */

static void ubx_checksum(const uint8_t *data, int len, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (int i = 0; i < len; i++) {
        a += data[i];
        b += a;
    }
    *ck_a = a;
    *ck_b = b;
}

static void ubx_build_poll(uint8_t cls, uint8_t id, uint8_t *frame)
{
    /* UBX frame: sync1 sync2 class id len_lo len_hi [payload] ck_a ck_b */
    frame[0] = 0xB5;
    frame[1] = 0x62;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = 0x00;  /* payload length LSB */
    frame[5] = 0x00;  /* payload length MSB */
    ubx_checksum(frame + 2, 4, &frame[6], &frame[7]);
}

/* Scan raw_buf for a complete, valid UBX frame of expected class/id.
   Returns pointer to start of payload inside raw_buf, sets *payload_len.
   Returns NULL if not found or checksum fails. */
static const uint8_t *ubx_find_frame(const uint8_t *buf, int len,
                                      uint8_t cls, uint8_t id,
                                      int *payload_len)
{
    for (int i = 0; i < len - 7; i++) {
        if (buf[i] != 0xB5 || buf[i+1] != 0x62) continue;
        if (buf[i+2] != cls || buf[i+3] != id)   continue;

        int plen = (int)(buf[i+4] | (buf[i+5] << 8));
        int frame_end = i + 6 + plen + 2;
        if (frame_end > len) continue;

        uint8_t ck_a, ck_b;
        ubx_checksum(buf + i + 2, 4 + plen, &ck_a, &ck_b);
        if (ck_a != buf[frame_end - 2] || ck_b != buf[frame_end - 1]) continue;

        *payload_len = plen;
        return buf + i + 6;
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* UBX-NAV-PVT payload parsing                                        */
/* ------------------------------------------------------------------ */

static void parse_pvt(const uint8_t *p, GpsFix *fix)
{
    /* Offsets per u-blox M10 interface description, UBX-NAV-PVT */
    uint16_t year   = (uint16_t)(p[4]  | (p[5]  << 8));
    uint8_t  month  = p[6];
    uint8_t  day    = p[7];
    uint8_t  hour   = p[8];
    uint8_t  min    = p[9];
    uint8_t  sec    = p[10];
    uint8_t  tvalid = p[11];  /* validFlags */

    uint8_t  fix_type = p[20];
    uint8_t  flags    = p[21];   /* bit0 = gnssFixOK */
    uint8_t  num_sv   = p[23];

    int32_t  lon_raw  = (int32_t)(p[24] | (p[25]<<8) | (p[26]<<16) | (p[27]<<24));
    int32_t  lat_raw  = (int32_t)(p[28] | (p[29]<<8) | (p[30]<<16) | (p[31]<<24));
    int32_t  alt_raw  = (int32_t)(p[36] | (p[37]<<8) | (p[38]<<16) | (p[39]<<24)); /* hMSL mm */

    int32_t  gspeed   = (int32_t)(p[60] | (p[61]<<8) | (p[62]<<16) | (p[63]<<24)); /* mm/s */
    int32_t  headmot  = (int32_t)(p[64] | (p[65]<<8) | (p[66]<<16) | (p[67]<<24)); /* 1e-5 deg */
    uint32_t hdop_raw = (uint32_t)(p[76] | (p[77]<<8) | (p[78]<<16) | (p[79]<<24)); /* 0.01 */

    fix->year        = year;
    fix->month       = month;
    fix->day         = day;
    fix->hour        = hour;
    fix->min         = min;
    fix->sec         = sec;
    fix->time_valid  = tvalid;
    fix->fix_type    = fix_type;
    fix->num_sv      = num_sv;
    fix->valid       = (flags & 0x01) ? 1 : 0;
    fix->lat         = lat_raw * 1e-7;
    fix->lon         = lon_raw * 1e-7;
    fix->alt_msl     = alt_raw * 1e-3;
    fix->speed_ms    = (float)(gspeed  * 1e-3);
    fix->heading_deg = (float)(headmot * 1e-5);
    fix->hdop        = (float)(hdop_raw * 0.01);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int gps_open(const char *i2c_dev)
{
    int fd = open(i2c_dev, O_RDWR);
    if (fd < 0) return -1;

    if (ioctl(fd, I2C_SLAVE, GPS_ADDR) < 0) {
        close(fd);
        return -1;
    }

    /* Confirm device responds */
    uint8_t probe;
    if (i2c_read_reg(fd, REG_BYTES_HI, &probe, 1) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

/*
 * Poll UBX-NAV-PVT and parse the response into fix.
 * Returns  0: success, fix populated
 *         -1: I2C error
 *          1: no response / incomplete frame (not an error, retry next cycle)
 */
int gps_read(int fd, GpsFix *fix)
{
    /* Flush any stale bytes from the DDC buffer before polling */
    int stale = ddc_bytes_available(fd);
    if (stale < 0) return -1;
    if (stale > 0) {
        uint8_t discard[4096];
        int to_flush = stale < (int)sizeof(discard) ? stale : (int)sizeof(discard);
        ddc_read(fd, discard, to_flush);
    }

    /* Send UBX-NAV-PVT poll */
    uint8_t poll_frame[8];
    ubx_build_poll(UBX_CLASS_NAV, UBX_ID_PVT, poll_frame);
    if (i2c_write_byte(fd, REG_DATA, poll_frame, 8) < 0) return -1;

    /* Wait up to 200 ms for response in 10 ms slices */
    uint8_t raw[256];
    int raw_len = 0;
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 10000000L }; /* 10 ms */

    for (int attempt = 0; attempt < 100; attempt++) {
        nanosleep(&ts, NULL);
        int avail = ddc_bytes_available(fd);
        if (avail < 0) return -1;
        if (avail == 0) continue;

        int want = avail < (int)(sizeof(raw) - raw_len)
                   ? avail : (int)(sizeof(raw) - raw_len);
        int got = ddc_read(fd, raw + raw_len, want);
        if (got < 0) return -1;
        raw_len += got;

        /* Minimum UBX-NAV-PVT frame: 6 header + 92 payload + 2 checksum = 100 bytes */
        if (raw_len < 100) continue;

        int plen;
        const uint8_t *payload = ubx_find_frame(raw, raw_len,
                                                 UBX_CLASS_NAV, UBX_ID_PVT, &plen);
        if (payload && plen >= UBX_PVT_LEN) {
            parse_pvt(payload, fix);
            return 0;
        }
    }
    return 1;
}

void gps_close(int fd)
{
    if (fd >= 0) close(fd);
}
