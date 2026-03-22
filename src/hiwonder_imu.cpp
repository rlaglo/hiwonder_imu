#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// 시리얼 라이브러리가 설치되어 있다고 가정 (sudo apt install ros-humble-serial-driver)
// 또는 단순화를 위해 개념적 시리얼 인터페이스를 사용합니다.
#include "serial_driver/serial_driver.hpp"
#include "io_context/io_context.hpp"

using namespace std::chrono_literals;

class IMUDriverNode : public rclcpp::Node {
public:
    IMUDriverNode() : Node("imu_driver_node"), // 초기화: Node, io_context, serial_driver
                      io_context_(std::make_shared<drivers::common::IoContext>()),
                      serial_driver_(std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_)) {
        
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

        // 1. 시리얼 포트 설정 (Config 객체 생성)
        drivers::serial_driver::SerialPortConfig config(
            460800, // Baud Rate
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE
        );

        try {
            // 2. 포트 열기
            serial_driver_->init_port("/dev/imu_usb", config);
            if (!serial_driver_->port()->is_open()) {
                serial_driver_->port()->open();
            }
            RCLCPP_INFO(this->get_logger(), "Serial Port /dev/imu_usb opened with 460800 baud");
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "Error opening port: %s", ex.what());
            return;
        }

        // 3. 비동기 수신 시작 (수신 시 receive_callback 호출)
        serial_driver_->port()->async_receive(
            std::bind(&IMUDriverNode::receive_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
// 데이터가 들어올 때마다 자동으로 실행되는 콜백 함수
    void receive_callback(const std::vector<uint8_t> & buffer, const size_t size) {
        for (size_t i = 0; i < size; ++i) {
            handleSerialData(buffer[i]);
        }
    }

    void handleSerialData(uint8_t raw_byte) {
        static std::vector<uint8_t> packet;
        packet.push_back(raw_byte);

        if (packet[0] != 0x55) {
            packet.clear();
            return;
        }

        if (packet.size() < 11) return;

        // Checksum 검사
        uint8_t sum = 0;
        for (int i = 0; i < 10; ++i) sum += packet[i];
        
        if (sum != packet[10]) {
            packet.clear();
            return;
        }

        processPacket(packet);
        packet.clear();
    }

    void processPacket(const std::vector<uint8_t>& p) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now(); // 타임스탬프 추가
        msg.header.frame_id = "imu_link";
        const double scale_factor_acc = (16.0 * 9.80665) / 32768.0;
        const double scale_factor_gyro = (2000.0 * M_PI / 180.0) / 32768.0;
        const double scale_factor_rpy = M_PI / 32768.0;
        int16_t raw[4];

        for (int i = 0; i < 4; ++i) {
            raw[i] = (int16_t)(p[i * 2 + 3] << 8 | p[i * 2 + 2]);
        }

        switch (p[1]) {
            case 0x51: // Acceleration imu가 주는 값이 -32768 ~ 32767이고, imu 스펙상 센서 측정 최대치가 +-16*중력이라서 정규화하는 작업
                accel_[0] = raw[0] * scale_factor_acc;
                accel_[1] = raw[1] * scale_factor_acc;
                accel_[2] = raw[2] * scale_factor_acc;
                break;
            case 0x52: // Angular Velocity
                gyro_[0] = raw[0] * scale_factor_gyro;
                gyro_[1] = raw[1] * scale_factor_gyro;
                gyro_[2] = raw[2] * scale_factor_gyro;
                break;
            case 0x53: // Angle
                double r = raw[0] * scale_factor_rpy;
                double p_val = raw[1] * scale_factor_rpy;
                double y = raw[2] * scale_factor_rpy;
                
                tf2::Quaternion q;
                q.setRPY(r, p_val, y);
                msg.orientation = tf2::toMsg(q);
                
                // 데이터 퍼블리시 (각도 패킷이 마지막에 오므로 이때 전송)
                msg.linear_acceleration.x = accel_[0];
                msg.linear_acceleration.y = accel_[1];
                msg.linear_acceleration.z = accel_[2];
                msg.angular_velocity.x = gyro_[0];
                msg.angular_velocity.y = gyro_[1];
                msg.angular_velocity.z = gyro_[2];
                imu_pub_->publish(msg);
                break;
        }
    }

    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    double accel_[3], gyro_[3];
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUDriverNode>());
    rclcpp::shutdown();
    return 0;
}