#ifndef IMU_COMMON
#define IMU_COMMON

#include <cstdio>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <csignal>
#include <unistd.h>

#include <iostream>
#include <typeinfo>

#include <sstream>
#include <vector>

#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

const float PI = 3.1415926;
const float g = 9.8;

using namespace std;
using namespace serial;

#endif // IMU_COMMON