#pragma once
#include <rviz_common/panel.hpp>
#include <QLabel>
#include <QLineEdit>
#include <QTimer>
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <a2r_robot_interfaces/msg/encoder_state.hpp>
#include <a2r_robot_interfaces/msg/hardware_status.hpp>
#include <chrono>
#include <map>
#endif
namespace a2r_rviz_panels {
class TelemetryPanel : public rviz_common::Panel {
 Q_OBJECT
public:
 explicit TelemetryPanel(QWidget *parent=nullptr);
 ~TelemetryPanel() override;
 void onInitialize() override;
 void save(rviz_common::Config config) const override;
 void load(const rviz_common::Config &config) override;
private:
 void connectRobot();
 void refresh();
 void value(const std::string &key,const QString &text);
 QLineEdit *namespace_;
 QLabel *connection_;
 QTimer *timer_;
 std::map<std::string,QLabel*> values_;
 rclcpp::Node::SharedPtr node_;
 rclcpp::Subscription<a2r_robot_interfaces::msg::EncoderState>::SharedPtr enc_sub_;
 rclcpp::Subscription<a2r_robot_interfaces::msg::HardwareStatus>::SharedPtr status_sub_;
 rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
 rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safety_sub_;
 a2r_robot_interfaces::msg::EncoderState::SharedPtr enc_;
 a2r_robot_interfaces::msg::HardwareStatus::SharedPtr status_;
 sensor_msgs::msg::Imu::SharedPtr imu_;
 sensor_msgs::msg::Joy::SharedPtr joy_;
 QString safety_;
 std::chrono::steady_clock::time_point enc_at_{},status_at_{},imu_at_{},joy_at_{};
};
}
