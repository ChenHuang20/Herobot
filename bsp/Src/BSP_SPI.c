
#include "BSP_SPI.h"

SPI_HandleTypeDef hspi1;

/**
  * @brief  SPI≥ı ºªØ
  * @param  void
  * @retval void
  */
void BSP_SPI_InitConfig(void)
{
	__HAL_RCC_SPI1_CLK_ENABLE();

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	__HAL_SPI_ENABLE(&hspi1);
}

typedef enum {
    IDLE = 0,
    ICM20600,
    MS5611,
    HMC5983
} device_e;

static device_e get_device(uint8_t address)
{
    device_e device;

    if (address == 0x68) {
        device = ICM20600;

    } else if (address == 0x1e) {
        device = HMC5983;

    } else {
        device = IDLE;
    }

    return device;
}

static void cs(device_e device, uint8_t state)
{
    GPIO_TypeDef* port = 0;
    uint16_t pin = 0;
    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (device) {
        case IDLE:
            break;

        case ICM20600:
            port = GPIOA;
            pin = GPIO_PIN_4;
            break;

        case HMC5983:
            break;

        default:
            break;
    }

    HAL_GPIO_WritePin(port, pin, pin_state);
}

static uint8_t readwritebyte(uint8_t write)
{
    uint8_t read = 0;

    HAL_SPI_TransmitReceive(&hspi1, &write, &read, 1, 1000);

    return read;
}

int spi1_read(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len)
{
    device_e device;

    device = get_device(address);

    // for most devices except MS5611, bit7 is R/W bit.
    // when 0, the data is written into the device.
    // when 1, the data from the device is read.
    if (device != MS5611) {
        reg |= 1 << 7;
    }

    // for HMC5983, bit6 is M/S bit.
    // when 0, the register address will remain unchanged in multiple read/write commands.
    // when 1, the register address will be auto incremented in multiple read/write commands.
    if (device == HMC5983 && len > 1) {
        reg |= 1 << 6;
    }

    // lower CS
    cs(device, 0);

    readwritebyte(reg);

    while (len--) {
        *buf++ = readwritebyte(0xff);
    }

    // higher CS
    cs(device, 1);

    return 0;
}

int spi1_write(uint8_t address, uint8_t reg, uint8_t *buf, uint8_t len)
{
    device_e device;

    device = get_device(address);

    // for HMC5983, bit6 is M/S bit.
    // when 0, the register address will remain unchanged in multiple read/write commands.
    // when 1, the register address will be auto incremented in multiple read/write commands.
    if (device == HMC5983 && len > 1) {
        reg |= 1 << 6;
    }

    // lower CS
    cs(device, 0);

    readwritebyte(reg);

    while (len--) {
        readwritebyte(*buf++);
    }

    // higher CS
    cs(device, 1);

    return 0;
}
