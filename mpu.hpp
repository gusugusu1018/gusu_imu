#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}
#include "MPU_define.h"

#define I2C_AUTO_INCREMENT  0x80
#define BLOCK_SIZE          6
#define GOFF_NB_ITER        500
#define I2C_OPEN_ERROR      1
#define I2C_DEV_NOT_FOUND   2
#define I2C_WRITE_ERROR     3
#define I2C_READ_ERROR      4
#define I2C_INIT_ERROR      5
#define A_GAIN              6.103515625e-05
#define G_GAIN              0.030487804878049
#define M_GAIN              0.3001221001221001

class Gusumpu
{
private:
	std::string bus;
	int fd = -1;
	unsigned char mpu_address;
	double  g_off[3] = { 0.0, 0.0, 0.0 };

public:
	Gusumpu(int busnum, unsigned char addr);
	int i2c_open();
	void i2c_close();
	int i2c_select_slave(unsigned char slave_addr);
	void i2c_set_bus(int busnum);
	int mpu_config(void );
	int mpu_read(double *a, double *g, double *m);
};

Gusumpu::Gusumpu(int busnum, unsigned char addr) {
	i2c_set_bus(busnum);
	i2c_select_slave(addr);
	int ret = mpu_config();
	if (ret > 0) {
		std::perror("mpu_config error");
		exit(EXIT_FAILURE);
	}
	usleep( 100000 );
	/* Test data reading */
	double  dummy_a[3], g[3], dummy_m[3];
	if (mpu_read(dummy_a, g, dummy_m)) {
		std::perror("mpu_read() error");
		exit(EXIT_FAILURE);
	}
	/* Calculate gyro offsets */
	double  offset[3] = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < GOFF_NB_ITER; i++) {
		if (mpu_read(dummy_a, g, dummy_m)) {
			std::perror("mpu_read() error");
			i--;
			break;
		}
		offset[0] += g[0];
		offset[1] += g[1];
		offset[2] += g[2];
	}
	g_off[0] = offset[0] / GOFF_NB_ITER;
	g_off[1] = offset[1] / GOFF_NB_ITER;
	g_off[2] = offset[2] / GOFF_NB_ITER;
}

int Gusumpu::mpu_config(void) {
	// 1 kHz sampling rate: 0b00000000
	if (i2c_smbus_write_byte_data(fd, MPU_SMPRT_DIV, 0x00 ) == -1)
		return I2C_WRITE_ERROR;
	// No ext sync, DLPF at 184Hz for the accel and 188Hz for the gyro: 0b00000001
	if (i2c_smbus_write_byte_data(fd, MPU_DEFINE, 0x01 ) == -1)
		return I2C_WRITE_ERROR;
	// Gyro range at +/-1000 °/s: 0b00010000
	if (i2c_smbus_write_byte_data(fd, MPU_GYRO_CONFIG, 0x10) == -1)
		return I2C_WRITE_ERROR;
	// Accel range at +/-2g: 0b00000000
	if (i2c_smbus_write_byte_data(fd, MPU_ACCEL_CONFIG, 0x00) == -1)
		return I2C_WRITE_ERROR;
	// Disable all FIFOs: 0b00000000
	if (i2c_smbus_write_byte_data(fd, MPU_FIFO_EN, 0x00) == -1)
		return I2C_WRITE_ERROR;
	// Bypass mode enabled: 0b00000010
	if (i2c_smbus_write_byte_data(fd, MPU_INT_PIN_CFG, 0x02) == -1)
		return I2C_WRITE_ERROR;
	// Disable all interrupts: 0b00000000
	if (i2c_smbus_write_byte_data(fd, MPU_INT_ENABLE, 0x00) == -1)
		return I2C_WRITE_ERROR;
	// No FIFO and no I2C slaves: 0b00000000
	if (i2c_smbus_write_byte_data(fd, MPU_USER_CTRL, 0x00) == -1)
		return I2C_WRITE_ERROR;
	// No power management, internal clock source: 0b00000000
	if (i2c_smbus_write_byte_data(fd, MPU_PWR_MGMT_1, 0x00) == -1)
		return I2C_WRITE_ERROR;
	/* Initialize magnetometer */
	if (ioctl(fd, I2C_SLAVE, MPU_I2C_MAGN_ADDRESS ) < 0)
		return I2C_DEV_NOT_FOUND;
	// Check for the AKM device ID
	int ret = i2c_smbus_read_byte_data(fd, MPU_WIA);
	if (ret < 0)
		return I2C_READ_ERROR;
	if (ret != MPU_AKM_ID)
		return I2C_DEV_NOT_FOUND;
	// Single measurement mode: 0b00000001
	if (i2c_smbus_write_byte_data(fd, MPU_CNTL, 0x01) == -1)
		return I2C_WRITE_ERROR;
	return 0;
}

int Gusumpu::i2c_open() {
	if (!fd) {
		std::cout << "i2c::open() : " << bus << std::endl;
		fd = open(bus.c_str(), O_RDWR);
		if (fd < 0) {
			std::perror("i2c::open()");
			fd = 0;
			return I2C_OPEN_ERROR;
		}
	}
	return 0;
}

void Gusumpu::i2c_close() {
	if (fd) {
		close(fd);
		fd = 0;
		mpu_address = 0;
	}
}

int Gusumpu::i2c_select_slave(unsigned char slave_addr) {
	if (mpu_address == slave_addr) return 0;
	if (i2c_open()) return I2C_OPEN_ERROR;
	std::cout << "i2c::select_slave(" << std::hex << int(slave_addr) << ")" << std::endl;
	if (ioctl(fd, I2C_SLAVE, slave_addr) < 0) {
		std::perror("ioctl(I2C_SLAVE)");
		return -1;
	}
	mpu_address = slave_addr;
	return 0;
}

void Gusumpu::i2c_set_bus(int busnum) {
	if (fd) i2c_close();
	bus = "/dev/i2c-" + std::to_string(busnum);
}

int Gusumpu::mpu_read( double *a, double *g, double *m ) {
	uint8_t         block[BLOCK_SIZE];
	int		ret;
	static double   last_m[3] = { 0.0, 0.0, 0.0 };
	static int      mag_state = 0;
	static uint8_t  mblock[BLOCK_SIZE] = { 0, 0, 0, 0, 0, 0 };
	a[0] = a[1] = a[2] = g[0] = g[1] = g[2] = m[0] = m[1] = m[2] = 0.0;
	if (fd == -1) return I2C_INIT_ERROR;
	if (ioctl(fd, I2C_SLAVE, mpu_address) < 0) return I2C_DEV_NOT_FOUND;
	if (i2c_smbus_read_i2c_block_data(fd, I2C_AUTO_INCREMENT | MPU_ACCEL_XOUT_H, BLOCK_SIZE, block ) != BLOCK_SIZE )
		return I2C_READ_ERROR;
	a[0] = (double)( (int16_t)( block[0] << 8 | block[1] ) * A_GAIN );
	a[1] = (double)( (int16_t)( block[2] << 8 | block[3] ) * A_GAIN );
	a[2] = (double)( (int16_t)( block[4] << 8 | block[5] ) * A_GAIN );
	/* Read gyro */
	if (i2c_smbus_read_i2c_block_data(fd, I2C_AUTO_INCREMENT | MPU_GYRO_XOUT_H, BLOCK_SIZE, block ) != BLOCK_SIZE )
		return I2C_READ_ERROR;
	g[0] = (double)( (int16_t)( block[0] << 8 | block[1] ) * G_GAIN ) - g_off[0];
	g[1] = (double)( (int16_t)( block[2] << 8 | block[3] ) * G_GAIN ) - g_off[1];
	g[2] = (double)( (int16_t)( block[4] << 8 | block[5] ) * G_GAIN ) - g_off[2];
	/* Read magnetometer */
	if (ioctl(fd, I2C_SLAVE, MPU_I2C_MAGN_ADDRESS ) < 0 )
		return I2C_DEV_NOT_FOUND;
	/* Read sequentially X, Y and Z to avoid too long delays */
	switch(mag_state) {
		case 0:
			// Check if data is ready.
			ret = i2c_smbus_read_byte_data( fd, MPU_ST1 );
			if ( ret < 0 ) return I2C_READ_ERROR;
			if ( ret & 0x01 ) mag_state = 1;
			// Duplicate last measurements
			m[0] = last_m[0];
			m[1] = last_m[1];
			m[2] = last_m[2];
			break;
		case 1:
			// Read X axis
			for (int i = 0; i < 2; i++ )  {
				ret = i2c_smbus_read_byte_data( fd, MPU_HXL + i );
				if ( ret < 0 ) return I2C_READ_ERROR;
				mblock[i] = ret;
			}
			mag_state = 2;
			// Duplicate last measurements
			m[0] = last_m[0];
			m[1] = last_m[1];
			m[2] = last_m[2];
			break;
		case 2:
			// Read Y axis
			for (int i = 2; i < 4; i++ )  {
				ret = i2c_smbus_read_byte_data( fd, MPU_HXL + i );
				if ( ret < 0 ) return I2C_READ_ERROR;
				mblock[i] = ret;
			}
			mag_state = 3;
			// Duplicate last measurements
			m[0] = last_m[0];
			m[1] = last_m[1];
			m[2] = last_m[2];
			break;
		case 3:
			// Read Z axis
			for (int i = 4; i < 6; i++ )  {
				ret = i2c_smbus_read_byte_data( fd, MPU_HXL + i );
				if ( ret < 0 ) return I2C_READ_ERROR;
				mblock[i] = ret;
			}
			m[1] = (double)( (int16_t)( mblock[1] << 8 | mblock[0] ) * M_GAIN );
			m[0] = (double)( (int16_t)( mblock[3] << 8 | mblock[2] ) * M_GAIN );
			m[2] = -(double)( (int16_t)( mblock[5] << 8 | mblock[4] ) * M_GAIN );
			last_m[0] = m[0];
			last_m[1] = m[1];
			last_m[2] = m[2];
			// Re-arm single measurement mode
			if (i2c_smbus_write_byte_data( fd, MPU_CNTL, 0x01 ) == -1 )
				return I2C_WRITE_ERROR;
			mag_state = 0;
			break;
		default:
			mag_state = 0;
	}
	return 0;
}

