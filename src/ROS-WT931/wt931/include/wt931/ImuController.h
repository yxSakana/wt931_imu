#include "wt931/CJY901.h"

class ImuController {
public:
    ImuController();

    void start();

    /**
     * @brief 发布 IMU 数据
     */
    void publishImu();
private:
    // ros交互
    ros::NodeHandle m_node_handle;
    ros::Publisher publisher;
    sensor_msgs::Imu m_imu_msg;
    string m_pub_topic_name;  // 发布话题名称
    // ros系统时间
    ros::Time m_time_current;
    // 串口交互
    std::unique_ptr<Serial> m_serial;
    string m_serial_port_name; // 串口端口名称
    int m_baud_rate;  // 波特率
    std::unique_ptr<CJY901> m_protocol;  // 通信协议
    uint8_t* m_serial_buffer;  // 串口数据
};