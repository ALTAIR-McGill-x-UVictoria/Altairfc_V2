#include <stdlib.h>
#include <gpiod.h>

typedef struct {
    struct gpiod_chip *chip;
    struct gpiod_line *line;
} GpioHold;

/*
 * Claim a single GPIO line as an output, driven high immediately, and hold
 * it there until closed. Returns a heap-allocated handle, or NULL on
 * failure.
 */
GpioHold *gpio_hold_open(const char *gpiochip_name, unsigned int offset)
{
    GpioHold *dev = (GpioHold *)malloc(sizeof(GpioHold));
    if (!dev) return NULL;
    dev->chip = NULL;
    dev->line = NULL;

    dev->chip = gpiod_chip_open_by_name(gpiochip_name);
    if (!dev->chip) {
        free(dev);
        return NULL;
    }

    dev->line = gpiod_chip_get_line(dev->chip, offset);
    if (!dev->line) {
        gpiod_chip_close(dev->chip);
        free(dev);
        return NULL;
    }

    if (gpiod_line_request_output(dev->line, "gpio_hold", 1) < 0) {
        gpiod_chip_close(dev->chip);
        free(dev);
        return NULL;
    }

    return dev;
}

void gpio_hold_close(GpioHold *dev)
{
    if (!dev) return;
    if (dev->line) {
        gpiod_line_set_value(dev->line, 1);
        gpiod_line_release(dev->line);
    }
    if (dev->chip) gpiod_chip_close(dev->chip);
    free(dev);
}
