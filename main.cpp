#include <iostream>
#include <chrono>
#include <iomanip>

#include "mpu.hpp"

int main(int argc, char **argv) {
	std::chrono::system_clock::time_point now, old;
	old = std::chrono::system_clock::now();
	Gusumpu mpu(1,0x68);
	std::cout << std::fixed << std::setprecision(5) << std::showpos;
	while(1) {
		now = std::chrono::system_clock::now();
		double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now - old).count() / 1000.0);
    		std::cout <<"time: "<<time<<"[ms] ";
		mpu.update();
		std::cout<<" ax: "<<mpu.raw_a[0]<<" ay: "<<mpu.raw_a[1]<<" az: "<<mpu.raw_a[2];
		std::cout<<" gx: "<<mpu.raw_g[0]<<" gy: "<<mpu.raw_g[1]<<" gz: "<<mpu.raw_g[2];
		//std::cout<<" mx: "<<mpu.raw_m[0]<<" my: "<<mpu.raw_m[1]<<" mz: "<<mpu.raw_m[2];
		std::cout<<" q0: "<<mpu.q0<<" q1: "<<mpu.q1<<" q2: "<<mpu.q2<<" q3: "<<mpu.q3;
		std::cout<<" roll: "<<mpu.roll<<" pitch: "<<mpu.pitch<<" yaw: "<<mpu.yaw<<std::endl;
		usleep(22000);
		old = now;
	}
}
