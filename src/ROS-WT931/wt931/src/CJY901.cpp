#include "wt931/CJY901.h"

CJY901::CJY901() {
    m_data_buffer = new char[1024];
    m_data_index = 0;
}

CJY901::~CJY901() {
    delete[] m_data_buffer;
    m_data_buffer = nullptr;
}

bool CJY901::CopeSerialData(const uint8_t& data) {
    m_data_buffer[m_data_index++] = char(data);

    if (m_data_buffer[0] != 0x55) {
        ROS_WARN("Failed: no right start U: start data: %d", m_data_buffer[0]);
        m_data_index = 0;
        return false;
    }

    if (m_data_index < 11) return false;
    switch (m_data_buffer[1]) {
        case 0x50: {
            // 时间
            memcpy(&imu_time, &m_data_buffer[2], 8);
            break;
        }
        case 0x51: {
            // 加速度
            memcpy(&imu_acc, &m_data_buffer[2], 8);
            break;
        }
        case 0x52: {
            // 角速度
            memcpy(&imu_angular_v, &m_data_buffer[2], 8);
            break;
        }
        case 0x53: {
            // Euler euler_angle
            memcpy(&imu_euler_angle, &m_data_buffer[2], 8);
            break;
        }
        case 0x54: {
            // 磁场
            memcpy(&imu_magnetic_field, &m_data_buffer[2], 8);
            break;
        }
        case 0x55: {
            // 端口状态
            memcpy(&imu_port_status, &m_data_buffer[2], 8);
            break;
        }
        case 0x56: {
            // 气压高度
            memcpy(&imu_barometric_altitude, &m_data_buffer[2], 8);
            break;
        }
        case 0x57: {
            // 经纬度
            memcpy(&imu_lat_lon, &m_data_buffer[2], 8);
            break;
        }
        case 0x58: {
            // GPS
            memcpy(&imi_gps, &m_data_buffer[2], 8);
            break;
        }
        case 0x59: {
            // 四元数
            memcpy(&imu_quaternion, &m_data_buffer[2], 8);
            break;
        }
        case 0x5a: {
            // GPS定位精度
            memcpy(&imu_gps_accuracy, &m_data_buffer[2], 8);
            break;
        }
        default: {
            ROS_WARN("Unknown buffer start: %d", m_data_buffer[1]);
            break;
        }
    }
    m_data_index = 0;
    return true;
}