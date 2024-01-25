#ifndef JY901_h
#define JY901_h

#include "wt931/imu_common.h"

struct ImuTime {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t sec;
    uint16_t mill_sec;
};
struct ImuAcc {
    int16_t a[3];
    int16_t T;
};
struct ImuAngularV {
    int16_t w[3];
    int16_t T;
};
struct ImuEulerAngle {
    int16_t euler_angle[3];
    int16_t T;
};
struct ImuMagneticField {
    int16_t h[3];
    int16_t T;
};

struct ImuPortStatus {
    int16_t status[4];
};

struct ImuBarometricAltitude {
    long pressure;
    long altitude;
};

struct ImuLatLon {
    long lon;
    long lat;
};

struct ImuGPS {
    int16_t gps_height;
    int16_t gps_yaw;
    int64_t gps_velocity;
};
struct ImuQuater {
    int16_t q0;
    int16_t q1;
    int16_t q2;
    int16_t q3;
};
struct ImuGpsAccuracy {
    int16_t SVNum;
    int16_t sPDOP;
    int16_t sHDOP;
    int16_t sVDOP;
};

class CJY901 {
public:
    CJY901();

    ~CJY901();

    bool CopeSerialData(const uint8_t& data);

    struct ImuTime imu_time;  // imu时间
    struct ImuAcc imu_acc;  // imu加速度
    struct ImuAngularV imu_angular_v;  // imu角速度
    struct ImuEulerAngle imu_euler_angle;  // imu Euler euler_angle
    struct ImuMagneticField imu_magnetic_field;  // 磁场
    struct ImuPortStatus imu_port_status; // 端口状态
    struct ImuBarometricAltitude imu_barometric_altitude; // 气压高度
    struct ImuLatLon imu_lat_lon; // 经纬度
    struct ImuGPS imi_gps; // GPS
    struct ImuQuater imu_quaternion; // 四元数
    struct ImuGpsAccuracy imu_gps_accuracy; // GPS定位精度
private:
    char* m_data_buffer;
    uint16_t m_data_index;
};
//extern CJY901 JY901;
#endif