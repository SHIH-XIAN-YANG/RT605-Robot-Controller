// TimerISR.cpp : use boost C++ lib:
#include <iostream>
#include<vector>
#include<string>
#include<cmath>
#define WIN32_LEAN_AND_MEAN // solution: https://blog.csdn.net/gzlyb/article/details/5870326
#include<Windows.h>
#include"boost\thread\thread.hpp"
#include"boost\timer\timer.hpp" // use boost timer version 2, CPU time 
#include"boost\asio.hpp"
#include"boost\asio\io_service.hpp"
#include"boost\bind.hpp"


void action01(const boost::system::error_code& /*e*/, boost::asio::deadline_timer *timer, int *dt,bool* condition, unsigned int *data) {

	if (*condition) { // waiting for the stopping condition:

		++(*data); // main action
		//----------------------------------------------------------------------
		timer->expires_at(timer->expires_at() + boost::posix_time::microsec(*dt));
		timer->async_wait(boost::bind(action01, boost::asio::placeholders::error, timer, dt, condition, data));
	}
}

void Process(bool* condition, unsigned int* data) {
	boost::asio::io_context io;
	int dt{ 1000 }; // define the sampling time or recorder time step
	boost::asio::deadline_timer timer1(io, boost::posix_time::microsec(dt));
	timer1.async_wait(boost::bind(action01, boost::asio::placeholders::error, &timer1, &dt, condition, data));
	io.run();
}

int main(void) {
	bool condition{ TRUE };
	unsigned int data{ 0 }, data_tmp;
	boost::thread thread01(Process,&condition, &data);
	data_tmp = data;
	std::string cmd;
	int i{ 0 };
	while (condition) { // main loop:
		//std::cin >> cmd;

		if (data != data_tmp) {
			std::cout <<  data << "x 2 =  " << 2 * data << std::endl;
			data_tmp = data;
			if ((++i) == 20)
				condition = FALSE;
		}
	}
	thread01.join();
    return 0;
}
