#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
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

#define sampleFreq	    40.0f// sample frequency in Hz
#define betaDef		    0.1f  // 2 * proportional gain

class Gusumpu
{
private:
	std::string bus;
	int fd = -1;
	unsigned char mpu_address;
	float  g_off[3] = { 0.0f, 0.0f, 0.0f };
	float beta = betaDef;

	float invSqrt(float x);
	int i2c_open();
	void i2c_close();
	int i2c_select_slave(unsigned char slave_addr);
	void i2c_set_bus(int busnum);
	int mpu_config(void );
	int mpu_read(float *a, float *g, float *m);
	void MadgwickAHRSupdate(float *a, float *g, float *m);
	void MadgwickAHRSupdateIMU(float *a, float *g);
	void QuaternionToEulerAngles(void );

public:
	Gusumpu(int busnum, unsigned char addr);

	float raw_a[3],raw_g[3],raw_m[3];
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
	float roll = 0.0f, pitch = 0.0f, yaw=0.0f;

	void update(void ) {
		mpu_read(raw_a, raw_g, raw_m);
		MadgwickAHRSupdateIMU(raw_a, raw_g);
		QuaternionToEulerAngles();
	}
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
	float  dummy_a[3], g[3], dummy_m[3];
	if (mpu_read(dummy_a, g, dummy_m)) {
		std::perror("mpu_read() error");
		exit(EXIT_FAILURE);
	}
	/* Calculate gyro offsets */
	float  offset[3] = { 0.0f, 0.0f, 0.0f };
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

int Gusumpu::mpu_read(float *a, float *g, float *m) {
	uint8_t         block[BLOCK_SIZE];
	int		ret;
	static float   last_m[3] = { 0.0f, 0.0f, 0.0f };
	static int      mag_state = 0;
	static uint8_t  mblock[BLOCK_SIZE] = { 0, 0, 0, 0, 0, 0 };
	a[0] = a[1] = a[2] = g[0] = g[1] = g[2] = m[0] = m[1] = m[2] = 0.0f;
	if (fd == -1) return I2C_INIT_ERROR;
	if (ioctl(fd, I2C_SLAVE, mpu_address) < 0) return I2C_DEV_NOT_FOUND;
	if (i2c_smbus_read_i2c_block_data(fd, I2C_AUTO_INCREMENT | MPU_ACCEL_XOUT_H, BLOCK_SIZE, block ) != BLOCK_SIZE )
		return I2C_READ_ERROR;
	a[0] = (float)( (int16_t)( block[0] << 8 | block[1] ) * A_GAIN );
	a[1] = (float)( (int16_t)( block[2] << 8 | block[3] ) * A_GAIN );
	a[2] = (float)( (int16_t)( block[4] << 8 | block[5] ) * A_GAIN );
	/* Read gyro */
	if (i2c_smbus_read_i2c_block_data(fd, I2C_AUTO_INCREMENT | MPU_GYRO_XOUT_H, BLOCK_SIZE, block ) != BLOCK_SIZE )
		return I2C_READ_ERROR;
	g[0] = (float)( (int16_t)( block[0] << 8 | block[1] ) * G_GAIN ) - g_off[0];
	g[1] = (float)( (int16_t)( block[2] << 8 | block[3] ) * G_GAIN ) - g_off[1];
	g[2] = (float)( (int16_t)( block[4] << 8 | block[5] ) * G_GAIN ) - g_off[2];
	/* Read magnetometer */
	if (ioctl(fd, I2C_SLAVE, MPU_I2C_MAGN_ADDRESS ) < 0 )
		return I2C_DEV_NOT_FOUND;
	/* Read sequentially X, Y and Z to avoid too long dela[1]s */
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
			// Read X a[0]is
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
			// Read Y a[0]is
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
			// Read Z a[0]is
			for (int i = 4; i < 6; i++ )  {
				ret = i2c_smbus_read_byte_data( fd, MPU_HXL + i );
				if ( ret < 0 ) return I2C_READ_ERROR;
				mblock[i] = ret;
			}
			m[1] = (float)( (int16_t)( mblock[1] << 8 | mblock[0] ) * M_GAIN );
			m[0] = (float)( (int16_t)( mblock[3] << 8 | mblock[2] ) * M_GAIN );
			m[2] = -(float)( (int16_t)( mblock[5] << 8 | mblock[4] ) * M_GAIN );
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

void Gusumpu::MadgwickAHRSupdate(float *a, float *g, float *m) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((m[0] == 0.0f) && (m[1] == 0.0f) && (m[2] == 0.0f)) {
		MadgwickAHRSupdateIMU(a, g);
		return;
	}
	// Rate of change of quaternion from g[1]roscope
	qDot1 = 0.5f * (-q1 * g[0] - q2 * g[1] - q3 * g[2]);
	qDot2 = 0.5f * (q0 * g[0] + q2 * g[2] - q3 * g[1]);
	qDot3 = 0.5f * (q0 * g[1] - q1 * g[2] + q3 * g[0]);
	qDot4 = 0.5f * (q0 * g[2] + q1 * g[1] - q2 * g[0]);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((a[0] == 0.0f) && (a[1] == 0.0f) && (a[2] == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
		a[0] *= recipNorm;
		a[1] *= recipNorm;
		a[2] *= recipNorm;   
		// Normalise magnetometer measurement
		recipNorm = invSqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
		m[0] *= recipNorm;
		m[1] *= recipNorm;
		m[2] *= recipNorm;
		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * m[0];
		_2q0my = 2.0f * q0 * m[1];
		_2q0mz = 2.0f * q0 * m[2];
		_2q1mx = 2.0f * q1 * m[0];
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;
		// Reference direction of Earth's magnetic field
		hx = m[0] * q0q0 - _2q0my * q3 + _2q0mz * q2 + m[0] * q1q1 + _2q1 * m[1] * q2 + _2q1 * m[2] * q3 - m[0] * q2q2 - m[0] * q3q3;
		hy = _2q0mx * q3 + m[1] * q0q0 - _2q0mz * q1 + _2q1mx * q2 - m[1] * q1q1 + m[1] * q2q2 + _2q2 * m[2] * q3 - m[1] * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + m[2] * q0q0 + _2q1mx * q3 - m[2] * q1q1 + _2q2 * m[1] * q3 - m[2] * q2q2 + m[2] * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - a[0]) + _2q1 * (2.0f * q0q1 + _2q2q3 - a[1]) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[0]) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[1]) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[2]);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - a[0]) + _2q0 * (2.0f * q0q1 + _2q2q3 - a[1]) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a[2]) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[0]) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[1]) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[2]);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - a[0]) + _2q3 * (2.0f * q0q1 + _2q2q3 - a[1]) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a[2]) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[0]) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[1]) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[2]);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - a[0]) + _2q2 * (2.0f * q0q1 + _2q2q3 - a[1]) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m[0]) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m[1]) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m[2]);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void Gusumpu::MadgwickAHRSupdateIMU(float *a, float *g) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	// Rate of change of quaternion from g[1]roscope
	qDot1 = 0.5f * (-q1 * g[0] - q2 * g[1] - q3 * g[2]);
	qDot2 = 0.5f * (q0 * g[0] + q2 * g[2] - q3 * g[1]);
	qDot3 = 0.5f * (q0 * g[1] - q1 * g[2] + q3 * g[0]);
	qDot4 = 0.5f * (q0 * g[2] + q1 * g[1] - q2 * g[0]);
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((a[0] == 0.0f) && (a[1] == 0.0f) && (a[2] == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
		a[0] *= recipNorm;
		a[1] *= recipNorm;
		a[2] *= recipNorm;   
		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;
		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * a[0] + _4q0 * q1q1 - _2q1 * a[1];
		s1 = _4q1 * q3q3 - _2q3 * a[0] + 4.0f * q0q0 * q1 - _2q0 * a[1] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a[2];
		s2 = 4.0f * q0q0 * q2 + _2q0 * a[0] + _4q2 * q3q3 - _2q3 * a[1] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a[2];
		s3 = 4.0f * q1q1 * q3 - _2q1 * a[0] + 4.0f * q2q2 * q3 - _2q2 * a[1];
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

float Gusumpu::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void Gusumpu::QuaternionToEulerAngles(void ) {
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    pitch = asin(2.0 * (q0q2 - q1q3));
    yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}
