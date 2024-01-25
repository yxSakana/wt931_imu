#include <cmath>

#include "wt931/ImuController.h"

ImuController::ImuController() {
    // 参数获取

    ros::NodeHandle nh_private("~");
    m_serial_port_name = nh_private.param<string>("serial_port_name", "/dev/wt931");
    m_baud_rate = nh_private.param<int>("baud_rate", 921600);
    m_pub_topic_name = nh_private.param<string>("pub_topic_name", "/imu_data");
    ROS_INFO("serial: %s; baud rate: %d; topic: %s",
             m_serial_port_name.c_str(),
             m_baud_rate,
             m_pub_topic_name.c_str());
    // 串口初始化
    Timeout timeout = Timeout::simpleTimeout(1000);
    try {
        m_serial = std::make_unique<Serial>(m_serial_port_name, m_baud_rate, timeout);
    } catch (exception& e) {
        ROS_FATAL("Failed: SerialPort initialization failed: "
                  "\n\twhat(): %s"
                  "\n\tserial_name: %s; baud_rate: %d; m_pub_topic_name: %s",
                  e.what(),
                  m_serial_port_name.c_str(), m_baud_rate, m_pub_topic_name.c_str());
        exit(-1);
    }
    // 检查串口是否正常
    if (m_serial->isOpen()) {
        ROS_INFO("Serial port is opened!");
    } else {
        ROS_FATAL("SerialPort Opened failed");
        exit(-1);
    }
    m_protocol = std::make_unique<CJY901>();
    m_serial_buffer = new uint8_t[1024];
    for (size_t i = 0; i < 1024; i++)
        m_serial_buffer[i] = 0;
    // 发布者 接收者 初始化
    publisher = m_node_handle.advertise<sensor_msgs::Imu>(m_pub_topic_name, 1000);
    // ros系统时间
    m_time_current = ros::Time::now();
    m_imu_msg.header.frame_id = "base_link";
}

void ImuController::start() {
    uint8_t read_success_count = 0;
    bool read_is_ok = false;
    for (size_t i = 0; i < 9; i++) {
        m_imu_msg.orientation_covariance[i] = 0;
        m_imu_msg.angular_velocity_covariance[i] = 0;
        m_imu_msg.linear_acceleration_covariance[i] = 0;
    }
    while (ros::ok()) {
        if (m_serial->available() > 44) {
            m_serial->read(m_serial_buffer, 44);
            for (size_t i = 0; i < 44; i++) {
                if (m_protocol->CopeSerialData(m_serial_buffer[i])) {
                    read_success_count += 1;
                    ROS_DEBUG("buffer data(%zu): %d", i, int(m_serial_buffer[i]));
                }
                if (read_success_count >= 4) {
                    read_success_count = 0;
                    read_is_ok = true;
                }
            }
        } else {
            usleep(1800);
        }
        if (read_is_ok) {
            read_is_ok = false;
            publishImu();
        }
        ros::spinOnce();
    }
}

void ImuController::publishImu() {
    m_imu_msg.header.stamp = ros::Time::now();
    m_imu_msg.orientation.w = float(m_protocol->imu_quaternion.q0) / 32768;
    m_imu_msg.orientation.x = float(m_protocol->imu_quaternion.q1) / 32768;
    m_imu_msg.orientation.y = float(m_protocol->imu_quaternion.q2) / 32768;
    m_imu_msg.orientation.z = float(m_protocol->imu_quaternion.q3) / 32768;

    m_imu_msg.linear_acceleration.x = float(m_protocol->imu_acc.a[0]) * 16 * g / 32768;
    m_imu_msg.linear_acceleration.y = float(m_protocol->imu_acc.a[1]) * 16 * g / 32768;
    m_imu_msg.linear_acceleration.z = float(m_protocol->imu_acc.a[2]) * 16 * g / 32768;

    m_imu_msg.angular_velocity.x = float(m_protocol->imu_angular_v.w[0]) * 2000 * PI / (32768 * 180);
    m_imu_msg.angular_velocity.y = float(m_protocol->imu_angular_v.w[1]) * 2000 * PI / (32768 * 180);
    m_imu_msg.angular_velocity.z = float(m_protocol->imu_angular_v.w[2]) * 2000 * PI / (32768 * 180);

    publisher.publish(m_imu_msg);
}



