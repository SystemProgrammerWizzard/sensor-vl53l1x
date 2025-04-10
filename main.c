#include "vl53l1x/vl53l1_platform.h"
#include "vl53l1x/VL53L1X_api.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <sys/stat.h>

#define I2C_DEVICE "/dev/i2c-1"
#define I2C_ADDRESS 0x29
#define FIFO_PATH "/tmp/fifo"

static int i2c_fd = -1;

int init_i2c()
{
    if ((i2c_fd = open(I2C_DEVICE, O_RDWR)) < 0)
    {
        perror("Failed to open I2C device");
        return -1;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, I2C_ADDRESS) < 0)
    {
        perror("Failed to set I2C slave address");
        return -1;
    }
    return 0;
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    uint8_t buf[2 + count];
    buf[0] = (index >> 8) & 0xFF;
    buf[1] = index & 0xFF;
    for (uint32_t i = 0; i < count; i++)
        buf[2 + i] = pdata[i];

    if (write(i2c_fd, buf, 2 + count) != (2 + count))
        return -1;
    return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    uint8_t buf[2] = {index >> 8, index & 0xFF};
    if (write(i2c_fd, buf, 2) != 2)
        return -1;
    if (read(i2c_fd, pdata, count) != (int)count)
        return -1;
    return 0;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    uint8_t buf[3] = {index >> 8, index & 0xFF, data};
    if (write(i2c_fd, buf, 3) != 3)
        return -1;
    return 0;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t buf[4] = {
        index >> 8, index & 0xFF,
        data >> 8, data & 0xFF};
    if (write(i2c_fd, buf, 4) != 4)
        return -1;
    return 0;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t buf[6] = {
        index >> 8, index & 0xFF,
        (data >> 24) & 0xFF,
        (data >> 16) & 0xFF,
        (data >> 8) & 0xFF,
        data & 0xFF};
    if (write(i2c_fd, buf, 6) != 6)
        return -1;
    return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
    uint8_t addr[2] = {index >> 8, index & 0xFF};
    if (write(i2c_fd, addr, 2) != 2)
        return -1;
    if (read(i2c_fd, data, 1) != 1)
        return -1;
    return 0;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
    uint8_t buf[2];
    if (VL53L1_ReadMulti(dev, index, buf, 2) != 0)
        return -1;
    *data = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
    uint8_t buf[4];
    if (VL53L1_ReadMulti(dev, index, buf, 4) != 0)
        return -1;
    *data = ((uint32_t)buf[0] << 24) |
            ((uint32_t)buf[1] << 16) |
            ((uint32_t)buf[2] << 8) |
            buf[3];
    return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    usleep(wait_ms * 1000);
    return 0;
}

int main(void)
{
    int fifo_fd;

    // Create the FIFO if it doesn't exist
    if (mkfifo(FIFO_PATH, 0666) == -1)
    {
        printf("FIFO already exists or error creating FIFO\n");
        exit(1);
    }

    // Initialize I2C and VL53L1X sensor
    if (init_i2c() < 0)
        return 1;

    if (VL53L1X_SensorInit(I2C_ADDRESS) != 0)
    {
        fprintf(stderr, "Sensor initialization failed\n");
        return 1;
    }

    if (VL53L1X_StartRanging(I2C_ADDRESS) != 0)
    {
        fprintf(stderr, "Sensor ranging start failed\n");
        return 1;
    }

    // Open the FIFO for writing (this blocks until reader opens it)
    fifo_fd = open(FIFO_PATH, O_WRONLY);
    if (fifo_fd < 0)
    {
        perror("Failed to open FIFO for writing");
        return 1;
    }

    // Main measurement loop
    while (1)
    {
        uint8_t ready = 0;
        while (!ready)
            VL53L1X_CheckForDataReady(I2C_ADDRESS, &ready);

        uint16_t distance = 0;
        if (VL53L1X_GetDistance(I2C_ADDRESS, &distance) == 0)
        {
            VL53L1X_ClearInterrupt(I2C_ADDRESS);

            
            if (write(fifo_fd, &distance, sizeof(distance)) < 0)
            {
                perror("Write to FIFO failed");
                break;
            }
        }

        usleep(100000); // 100ms delay between samples
    }

    close(fifo_fd);
    return 0;
}