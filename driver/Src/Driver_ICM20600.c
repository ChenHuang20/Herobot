
#include "Driver_ICM20600.h"
#include "Algorithm_Estimator.h"
#include "Driver_DBUS.h"
#include "BSP_SPI.h"
#include "BSP_TIM.h"

#include <math.h>

#ifndef M_PI_F
#define M_PI_F  3.14159265358979323846f
#endif

#ifndef M_ONE_G
#define M_ONE_G 9.80665f
#endif

#ifndef IMU_XY_EX
    #define IMU_XY_EX   1
#endif

#ifndef IMU_X_REV
    #define IMU_X_REV   1
#endif

#ifndef IMU_Y_REV
    #define IMU_Y_REV   1
#endif

#ifndef IMU_Z_REV
    #define IMU_Z_REV   1
#endif

params_t _params = { 
	.accel_offset[0] = 0.286253214f,
    .accel_offset[1] = 0.205908254f,
    .accel_offset[2] = -0.00103282928f,
    .accel_scale[1] = 1.0f,
    .accel_scale[0] = 1.0f,
    .accel_scale[2] = 1.0f,

    .gyro_offset[0] = -0.00396604743f,
    .gyro_offset[1] = 0.00495731691f,
    .gyro_offset[2] = 0.00031299665f,
    .gyro_scale[0] = 1.0f,
    .gyro_scale[1] = 1.0f,
    .gyro_scale[2] = 1.0f,
};
accel_t _accel = { 0 };
gyro_t _gyro = { 0 };

// device path: bus(spi) / address(0x68) / register(0x00)
static char path[3] = { 's', 0x68, 0x00 };

struct {
    const float accel_range_scale;
    const float gyro_range_scale;

    low_pass_filter_t accel_filter[3];
    low_pass_filter_t gyro_filter[3];

    int inited;

} _icm = {
    .accel_range_scale = 0.00239420166015625f, // to m/s/s :  8 g / 32768 * M_ONE_G
    .gyro_range_scale = 0.001065264436031695f, // to rad/s :  2000 deg/s / 32768 * (PI / 180 deg)

    .inited = 0
};

/***************************************低通滤波器***********************************************/
void lpf_set_cutoff_frequency(low_pass_filter_t *p, float sample_freq, float cutoff_freq)
{
    p->_cutoff_freq = cutoff_freq;

    if (p->_cutoff_freq <= 0.0f) {
        p->_a1 = 0.0f;
        p->_a2 = 0.0f;
        p->_b0 = 0.0f;

        p->_delay_element_1 = 0.0f;
        p->_delay_element_2 = 0.0f;

    } else {
        float fr = sample_freq / cutoff_freq;
        float ohm = tanf(M_PI_F / fr);
        float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

        p->_b0 = ohm * ohm / c;
        p->_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
        p->_a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;

        p->_delay_element_1 = 0.0f;
        p->_delay_element_2 = 0.0f;
    }
}

float lpf_allpy(low_pass_filter_t *p, float sample)
{
    if (p->_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }

    // do filtering
    float delay_element_0 = sample - p->_delay_element_1 * p->_a1 - p->_delay_element_2 * p->_a2;

    if (!isfinite(delay_element_0)) {
        // don't allow bad values to propagate via the filter
        delay_element_0 = sample;
    }

    float output = delay_element_0 * p->_b0 + p->_delay_element_1 * 2.0f * p->_b0 + p->_delay_element_2 * p->_b0;

    p->_delay_element_2 = p->_delay_element_1;
    p->_delay_element_1 = delay_element_0;

    // return the value. should be no need to check limits
    return output;
}

float lpf_reset(low_pass_filter_t *p, float sample)
{
    float dval = sample / (p->_b0 + 2.0f * p->_b0 + p->_b0);

    p->_delay_element_1 = dval;
    p->_delay_element_2 = dval;

    return lpf_allpy(p, sample);
}

/***************************************ICM20600初始化***********************************************/

int read(char *path, uint8_t *buf, int len)
{
    int ret = -1;

    switch (path[0]) {

        case 's':
            // spi bus
            ret = spi1_read(path[1], path[2], buf, len);
            break;

        default:
            break;
    }

    return ret;
}

int write(char *path, uint8_t *buf, int len)
{
    int ret = -1;

    switch (path[0]) {

        case 's':
            ret = spi1_write(path[1], path[2], buf, len);
            break;

        default:
            break;
    }

    return ret;
}

static int read_reg(uint8_t reg, uint8_t *buf, int len)
{
    path[2] = reg;

    return read(path, buf, len);
}

static uint8_t read_reg_single(uint8_t reg)
{
    uint8_t val = 0;

    read_reg(reg, &val, 1);

    return val;
}

static int write_reg(uint8_t reg, uint8_t val)
{
    path[2] = reg;

    return write(path, &val, 1);
}

static int write_reg_check(uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { val, ~val };

    usleep(1000);

    write_reg(reg, data[0]);

    usleep(1000);

    read_reg(reg, &data[1], 1);

    if (data[0] != data[1]) {
        return -1;
    }

    return 0;
}

//static int32_t t1 = 0, t2 = 0;
static void init()
{
//	t2 = _clock.tick_ms - t1;
    // try to start device
    int tries = 5;

    while (tries--) {

        // name: PWM_MGMT_1 (0x6b)
        //
        // DEVICE_RESET[7]: 1-reset the internal registers and restores the default setting.
        //                  the bit automatically clears to 0 once the reset is done.
        //
        // SLEEP[6]: when set to 1, the chip is set to sleep mode.
        //
        // CYCLE[5]: when set to 1, and SLEEP and STANDBY are not set to 1, the chip cycle between sleep
        //           and taking a single accelerometer sample at a rate determined by SMPLRT_DIV.
        //
        // GYRO_STANDBY[4]: when set, the gyro drive and pll circuitry are enabled, but the sense paths
        //                  are disabled. this is a low power mode tha allows quick enabling of the gyros.
        //
        // TEMP_DIS[3]: when set to 1, this bit disables the temperature sensor.
        //
        // CLKSEL[2:0]: 0 or 6: internal 20 MHz oscillator
        //              1 to 5: auto selects the best available clock source - PLL if ready, else use the internal oscillator
        //              7     : stops the clock and keeps timing generator in reset
        //
		
        write_reg(0x6b, 1 << 7);
        usleep(15000);             // stabilization time 14ms

        write_reg(0x6b, 1);
        usleep(25000);             // from sleep mode to valid data 20ms

        // check chip id
        if (read_reg_single(0x75) == 0x70) {
            break;
        }

        usleep(1000);
    }

	if (!tries) {
        return;
    }

    int state = 0;

    // disable i2c slave module and put the serial interface in spi mode only
    state += write_reg_check(0x70, 1 << 6);

    // name: SMPLRT (0x19)
    //
    // SMPLRT_DIV[7:0]
    // note: this register is only effective when FCHOICE_B register bits are 2'b00 and (0 < DLPF_CFG < 7)
    // SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) where INTERNAL_SAMPLE_RATE = 1kHz
    state += write_reg_check(0x19, 0);

    // name: CONFIG (0x1a)
    //
    // bit7: default configuration value is 1, user should set it to 0
    //
    // FIFO_MODE[6]: set to '1', when the FIFO is full, additional writes will not be written to FIFO
    //               set to '0', when the FIFO is full, additional writes will be written to the FIFO, replacing the oldest data
    //
    // EXT_SYNC_SET[5:3] : 0        | 1             | 2              | 3              | 4              | 5               | 6               | 7
    // FSYNC bit location: disabled | TEMP_OUT_L[0] | GYRO_XOUT_L[0] | GYRO_YOUT_L[0] | GYRO_ZOUT_L[0] | ACCEL_XOUT_L[0] | ACCEL_YOUT_L[0] | ACCEL_ZOUT_L[0]
    //
    // if FCHOICE_B is not zero, see REG-0x1B, else:
    // DLPF_CFG[2:0]    : 0     | 1     | 2     | 3    | 4    | 5    | 6   | 7
    // Gyro 3-dB BW(Hz) : 250   | 176   | 92    | 41   | 20   | 10   | 5   | 3281
    // Gyro Noise BW(Hz): 306.6 | 177.0 | 108.6 | 59.0 | 30.5 | 15.6 | 8.0 | 3451.0
    // Gyro Rate(kHz)   : 8     | 1     | 1     | 1    | 1    | 1    | 1   | 8
    // Temp 3-dB BW(Hz) : 4000  | 188   | 98    | 42   | 20   | 10   | 5   | 4000
    //
    // FS & DLPF  FS = 2000 deg/s, DLPF = 20Hz (low pass filter)
    // was 90 Hz, but this ruins quality and does not improve the system response
    // dlpf filter frequency 41Hz
    state += write_reg_check(0x1a, 3);

    // name: GYRO_CONFIG (0x1b)
    //
    // bit7: X Gyro self-test
    // bit6: Y Gyro self-test
    // bit5: Z Gyro self-test
    //
    // GYRO_FS_SEL[4:3]      : 0   | 1   | 2    | 3
    // Gyro full scale(+-dps): 250 | 500 | 1000 | 2000
    //
    // bit2: reserved
    //
    // FCHOICE_B[1:0]   : x1     | 10     | else see REG-0x1A
    // Gyro 3-dB BW(Hz) : 8173   | 3281   |
    // Gyro Noise BW(Hz): 8595.1 | 3451.0 |
    // Gyro Rate(kHz)   : 32     | 32     |
    // Temp 3-dB BW(Hz) : 4000   | 4000   |
    //
    // set Gyro scale 2000 deg/s
    state += write_reg_check(0x1b, 3 << 3);

    // name: ACCEL_CONFIG (0x1c)
    //
    // bit7: X Accel self-test
    // bit6: Y Accel self-test
    // bit5: Z Accel self-test
    //
    // ACCEL_FS_SEL[4:3]    : 0 | 1 | 2 | 3
    // Accel full scale(+-g): 2 | 4 | 8 | 16
    //
    // bit[2:0]: reserved
    //
    // set Accel scale +-8 g
    state += write_reg_check(0x1c, 2 << 3);

    // name: ACCEL_CONFIG2 (0x1d)
    //
    // averaging filter setting for low power accelerometer mode
    // DEC2_CFG[5:4]  : 0 | 1 | 2  | 3
    // average samples: 4 | 8 | 16 | 32
    //
    // ACCEL_FCHOICE_B[3]: when set to '1', accelerometer 3-dB BW is 1046.0Hz, noise BW is 1100.0Hz, rate is 4kHz
    //                     when set to '0', see A_DLPF_CFG shown in the table below
    //
    // A_DLPF_CFG[2:0]   : 0     | 1     | 2     | 3    | 4    | 5    | 6   | 7
    // Accel 3-dB BW(Hz) : 218.1 | 218.1 | 99.0  | 44.8 | 21.2 | 10.2 | 5.1 | 420
    // Accel Noise BW(Hz): 235.0 | 235.0 | 121.3 | 61.5 | 31.0 | 15.5 | 7.8 | 441.6
    // Accel Rate(kHz)   : 1     | 1     | 1     | 1    | 1    | 1    | 1   | 1
    //
    // dlpf filter frequency 44.8Hz
    state += write_reg_check(0x1d, 3);

    // name: INT_PIN_CFG (0x37)
    //
    // bit7: '1' - the logic level for INT/DRDY pin is active low
    //       '0' - the logic level for INT/DRDY pin is active high
    //
    // bit6: '1' - INT/DRDY pin is configured as open drain
    //       '0' - INT/DRDY pin is configured as push-pull
    //
    // bit5: '1' - INT/DRDY pin level held until interrupt status is cleared
    //       '0' - INT/DRDY pin indicates interrupt pulse's width is 50us
    //
    // bit4: '1' - interrupt status is cleared if any read operation is performed
    //       '0' - interrupt status is cleared only by reading INT_STATUS register
    //
    // bit3: '1' - the logic level for the FSYNC pin as an interrupt is active low
    //       '0' - the logic level for the FSYNC pin as an interrupt is active high
    //
    // bit2: '1' - the FSYNC pin will trigger an interrupt
    //       '0' - the FYSNC pin is disabled
    //
    // bit1: reserved
    //
    // bit0: enable INT2 interrupt pin
    //
    // INT clear on any read
    state += write_reg_check(0x37, 1 << 4);

    // name: INT_ENABLE (0x38)
    //
    // bit7: WOM_X_INT_EN
    // bit6: WOM_Y_INT_EN
    // bit5: WOM_Z_INT_EN
    // bit4: FIFO_OFLOW_EN
    // bit3: FSYNC_INT_EN
    // bit2: GDRIVE_INT_EN
    // bit1: reserved
    // bit0: DATA_RDY_INT_EN
    //
    // enable data ready interrupt
    state += write_reg_check(0x38, 1);

    // init low pass filter
    for (int i = 0; i < 3; i++) {
        lpf_set_cutoff_frequency(&_icm.accel_filter[i], 1000, 30);
        lpf_set_cutoff_frequency(&_icm.gyro_filter[i], 1000, 30);
    }

    if (!state) {
        _icm.inited = 1;
    }
}

static void measure()
{
    uint64_t t = time();

    uint8_t buffer[14] = { 0 };

    if (read_reg(0x3b, buffer, sizeof(buffer)) == 0) {

        int16_t temp_raw;
        int16_t accel_raw[3];
        int16_t gyro_raw[3];

        accel_raw[0] = (buffer[1] | (uint16_t)(buffer[0] << 8));
        accel_raw[1] = (buffer[3] | (uint16_t)(buffer[2] << 8));
        accel_raw[2] = (buffer[5] | (uint16_t)(buffer[4] << 8));
        temp_raw = (buffer[7] | (uint16_t)(buffer[6] << 8));
        gyro_raw[0] = (buffer[9] | (uint16_t)(buffer[8] << 8));
        gyro_raw[1] = (buffer[11] | (uint16_t)(buffer[10] << 8));
        gyro_raw[2] = (buffer[13] | (uint16_t)(buffer[12] << 8));

        // rotate imu from sensor frame to body frame
        #if IMU_XY_EX
            int16_t temp = accel_raw[0];
            accel_raw[0] = accel_raw[1];
            accel_raw[1] = temp;

            temp = gyro_raw[0];
            gyro_raw[0] = gyro_raw[1];
            gyro_raw[1] = temp;
        #endif

        #if IMU_X_REV
            accel_raw[0] = (accel_raw[0] == -32768) ? 32767 : -accel_raw[0];
            gyro_raw[0] = (gyro_raw[0] == -32768) ? 32767 : -gyro_raw[0];
        #endif

        #if IMU_Y_REV
            accel_raw[1] = (accel_raw[1] == -32768) ? 32767 : -accel_raw[1];
            gyro_raw[1] = (gyro_raw[1] == -32768) ? 32767 : -gyro_raw[1];
        #endif

        #if IMU_Z_REV
            accel_raw[2] = (accel_raw[2] == -32768) ? 32767 : -accel_raw[2];
            gyro_raw[2] = (gyro_raw[2] == -32768) ? 32767 : -gyro_raw[2];
        #endif

        float temperature;
        float accel_scaled[3];
        float gyro_scaled[3];

        temperature = (float)temp_raw / 361.0f + 35.0f;

        for (int i = 0; i < 3; i++) {
            accel_scaled[i] = accel_raw[i] * _icm.accel_range_scale - _params.accel_offset[i];
            gyro_scaled[i] = gyro_raw[i] * _icm.gyro_range_scale - _params.gyro_offset[i];
        }

        // publish accel
        _accel.timestamp = t;
        _accel.temperature = temperature;
        _accel.x_raw = accel_raw[0];
        _accel.y_raw = accel_raw[1];
        _accel.z_raw = accel_raw[2];
        _accel.x = lpf_allpy(&_icm.accel_filter[0], accel_scaled[0]);
        _accel.y = lpf_allpy(&_icm.accel_filter[1], accel_scaled[1]);
        _accel.z = lpf_allpy(&_icm.accel_filter[2], accel_scaled[2]);

        // publish gyro
        _gyro.timestamp = t;
        _gyro.temperature = temperature;
        _gyro.x_raw = gyro_raw[0];
        _gyro.y_raw = gyro_raw[1];
        _gyro.z_raw = gyro_raw[2];
        _gyro.x = lpf_allpy(&_icm.gyro_filter[0], gyro_scaled[0]);
        _gyro.y = lpf_allpy(&_icm.gyro_filter[1], gyro_scaled[1]);
        _gyro.z = lpf_allpy(&_icm.gyro_filter[2], gyro_scaled[2]);
    }
}

/***************************************ICM20600校准***********************************************/

static uint8_t _calibrate = 0;

static void calibrate()
{
    static int count = 0;
    static float accel_sample[3] = { 0.0f };
    static float gyro_sample[3] = { 0.0f };

    const int samples = 300;

    if (_calibrate) {
        if (!count) {
            gyro_sample[0] = _gyro.x;
            gyro_sample[1] = _gyro.y;
            gyro_sample[2] = _gyro.z;

            accel_sample[0] = _accel.x;
            accel_sample[1] = _accel.y;
            accel_sample[2] = _accel.z;

            count++;

        } else {
            
            gyro_sample[0] += _gyro.x;
            gyro_sample[1] += _gyro.y;
            gyro_sample[2] += _gyro.z;

            accel_sample[0] += _accel.x;
            accel_sample[1] += _accel.y;
            accel_sample[2] += _accel.z;

            count++;

            if (count == samples) {
                for (int i = 0; i < 3; i++) {
                    gyro_sample[i] /= (float)samples;
                    accel_sample[i] /= (float)samples;
                }

                accel_sample[2] += M_ONE_G;

                _params.gyro_offset[0] += gyro_sample[0];
                _params.gyro_offset[1] += gyro_sample[1];
                _params.gyro_offset[2] += gyro_sample[2];

                _params.accel_offset[0] += accel_sample[0];
                _params.accel_offset[1] += accel_sample[1];
                _params.accel_offset[2] += accel_sample[2];

                count = 0;

                _calibrate = 0;

//                // save parameters to flash
//                flash_save();
            }
        }
    }
}

static void commander()
{
    uint64_t t = time();
    
    static uint64_t count = 0;
    
    count++;

    // calibration
    static uint64_t calibrate_timestamp = 0;

    if (!_calibrate && _radio.rc.pitch > 0.9f && _radio.rc.yaw < -0.9f && _radio.rc.y < -0.9f && _radio.rc.x < -0.9f) {
        if (t > calibrate_timestamp + 500000) {
            _calibrate = 1;
        }

    } else {
        calibrate_timestamp = t;
    }

    calibrate();
}

void icm20600_task(void)
{

    if (!_icm.inited) {
        init();
        return;
    }
    measure();
	estimator_task();
	commander();
}
