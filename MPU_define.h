#define MPU_I2C_ADDRESS_1          0x69    // Base address of the Drotek board
#define MPU_I2C_ADDRESS_2          0x68    // Base address of the SparkFun board
#define MPU_SMPRT_DIV              0x19    // Gyro sampling rate divider
#define MPU_DEFINE                 0x1A    // Gyro and accel configuration
#define MPU_GYRO_CONFIG            0x1B    // Gyroscope configuration
#define MPU_ACCEL_CONFIG           0x1C    // Accelerometer configuration
#define MPU_FIFO_EN                0x23    // FIFO buffer control
#define MPU_INT_PIN_CFG            0x37    // Bypass enable configuration
#define MPU_INT_ENABLE             0x38    // Interrupt control
#define MPU_ACCEL_XOUT_H           0x3B    // Accel X axis High
#define MPU_ACCEL_XOUT_L           0x3C    // Accel X axis Low
#define MPU_ACCEL_YOUT_H           0x3D    // Accel Y axis High
#define MPU_ACCEL_YOUT_L           0x3E    // Accel Y axis Low
#define MPU_ACCEL_ZOUT_H           0x3F    // Accel Z axis High
#define MPU_ACCEL_ZOUT_L           0x40    // Accel Z axis Low
#define MPU_GYRO_XOUT_H            0x43    // Gyro X axis High
#define MPU_GYRO_XOUT_L            0x44    // Gyro X axis Low
#define MPU_GYRO_YOUT_H            0x45    // Gyro Y axis High
#define MPU_GYRO_YOUT_L            0x46    // Gyro Y axis Low
#define MPU_GYRO_ZOUT_H            0x47    // Gyro Z axis High
#define MPU_GYRO_ZOUT_L            0x48    // Gyro Z axis Low
#define MPU_USER_CTRL              0x6A    // User control
#define MPU_PWR_MGMT_1             0x6B    // Power management 1
#define MPU_I2C_MAGN_ADDRESS       0x0C    // Address of the magnetometer in bypass mode
#define MPU_WIA                    0x00    // Mag Who I Am
#define MPU_AKM_ID                 0x48    // Mag device ID
#define MPU_ST1                    0x02    // Magnetometer status 1
#define MPU_HXL                    0x03    // Mag X axis Low
#define MPU_HXH                    0x04    // Mag X axis High
#define MPU_HYL                    0x05    // Mag Y axis Low
#define MPU_HYH                    0x06    // Mag Y axis High
#define MPU_HZL                    0x07    // Mag Z axis Low
#define MPU_HZH                    0x08    // Mag Z axis High
#define MPU_ST2                    0x09    // Magnetometer status 2
#define MPU_CNTL                   0x0A    // Magnetometer control
