#include <iostream>

#include "mpu.hpp"

int main(int argc, char **argv) {
	double  a[3], g[3], m[3];
	struct  timespec ts1, ts2;
	Gusumpu mpu(1,0x68);
	while(1) {
		clock_gettime( CLOCK_MONOTONIC, &ts1 );
		mpu.mpu_read( a, g, m );
		clock_gettime( CLOCK_MONOTONIC, &ts2 );
		printf("t: %-5d ", (int)( ts2.tv_sec - ts1.tv_sec ) * 1000000 + (int)( ts2.tv_nsec - ts1.tv_nsec ) / 1000);
		printf("Ax: %-8.2f Ay: %-8.2f Az: %-8.2f ", a[0], a[1], a[2]);
		printf("Gx: %-8.2f Gy: %-8.2f Gz: %-8.2f ", g[0], g[1], g[2]);
		printf("Mx: %-8.2f My: %-8.2f Mz: %-8.2f\n", m[0], m[1], m[2]);
	}
}

