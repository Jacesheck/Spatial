#include <iostream>
#include <chrono>
#include <string>
#include <fstream>
#include "spatial"

namespace clk = std::chrono;

int main(){
	Spatial imu;
	if(imu.connect((char*) "/dev/ttyUSB0", 2000000) == false){
		std::cout << "Could not connect\n";
		return 0;
	}

	auto old = clk::steady_clock::now();
	std::ofstream record_file("../acceleration.txt");
	for(int i=0;i<100;i++){
		while(imu.poll() == false){} // Inefficient and blocking
		auto recent = clk::steady_clock::now();
		std::cout << "Time: " << clk::duration_cast<clk::milliseconds>(recent - old).count() << "\n";
		old = recent;
		for(IMU_acc state : imu.buffer){
			std::cout << "x: " << state.x << " y: " << state.y << " z: " << state.z << "\n";
			record_file << state.x << " " << state.y << " " << state.z << "\n";
		}
		std::cout << "\n";
	}
	record_file.close();

	return 0;
}