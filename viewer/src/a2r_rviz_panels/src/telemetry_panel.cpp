#include "a2r_rviz_panels/telemetry_panel.hpp"
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QTabWidget>
#include <QGroupBox>
#include <QFontDatabase>
#include <QScrollArea>
#include <cmath>
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
namespace a2r_rviz_panels {
using Clock=std::chrono::steady_clock;
static bool fresh(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count()<1.;}
static QString number(double v,int digits=3) {return std::isfinite(v)?QString::number(v,'f',digits):QString::fromUtf8("—");}
TelemetryPanel::TelemetryPanel(QWidget *parent):Panel(parent) {
 setMinimumWidth(330);
 auto root=new QVBoxLayout(this);
 auto select=new QHBoxLayout();namespace_=new QLineEdit("jetson1");
 namespace_->setPlaceholderText("Robot namespace");auto apply=new QPushButton("Connect");
 select->addWidget(namespace_);select->addWidget(apply);root->addLayout(select);
 connection_=new QLabel("Waiting for robot telemetry");root->addWidget(connection_);
 auto legend=new QLabel("Yellow: FRONT (+X) · Purple: REAR\nAxes: X forward · Y left · Z up");
 legend->setWordWrap(true);root->addWidget(legend);
 auto tabs=new QTabWidget();root->addWidget(tabs,1);
 auto makeTab=[&](const QString &name) {auto page=new QWidget();auto form=new QFormLayout(page);
  form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);tabs->addTab(page,name);return form;};
 auto row=[&](QFormLayout *form,const QString &label,const std::string &key) {
  auto v=new QLabel(QString::fromUtf8("—"));v->setTextInteractionFlags(Qt::TextSelectableByMouse);
  v->setAlignment(Qt::AlignRight|Qt::AlignVCenter);v->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  v->setWordWrap(true);form->addRow(label,v);values_[key]=v;};
 auto drive=makeTab("Status");
 for (auto item:std::initializer_list<std::pair<const char*,const char*>>{
  {"Motion permission","permission"},{"RB / A","buttons"},{"Applied L / R (m/s)","applied"},
  {"UART","uart"},{"IMU / encoders","sensors"},{"Gamepad / camera","devices"},
  {"Control status","safety"},{"Gate reason","reason"}}) row(drive,item.first,item.second);
 auto enc=makeTab("Encoders");
 for (auto item:std::initializer_list<std::pair<const char*,const char*>>{
  {"Stream","enc_stream"},{"Left raw ticks","l_ticks"},{"Right raw ticks","r_ticks"},
  {"Left relative ticks","l_rel"},{"Right relative ticks","r_rel"},
  {"Left distance (m)","l_distance"},{"Right distance (m)","r_distance"},
  {"Left speed (m/s)","l_speed"},{"Right speed (m/s)","r_speed"},
  {"Left scale (mm/tick)","l_scale"},{"Right scale (mm/tick)","r_scale"}}) row(enc,item.first,item.second);
 auto imu=makeTab("IMU");
 for (auto item:std::initializer_list<std::pair<const char*,const char*>>{
  {"Stream","imu_stream"},{"Front tilt (deg, up +)","front"},{"Left tilt (deg, up +)","left"},
  {"Yaw (deg)","yaw"},{"Quaternion WXYZ","quaternion"},
  {"Accel X / Y / Z (m/s²)","accel"},{"Gyro X / Y / Z (rad/s)","gyro"}}) row(imu,item.first,item.second);
 timer_=new QTimer(this);timer_->setInterval(100);
 connect(timer_,&QTimer::timeout,this,[this](){refresh();});
 connect(apply,&QPushButton::clicked,this,[this](){connectRobot();Q_EMIT configChanged();});
 connect(namespace_,&QLineEdit::returnPressed,this,[this](){connectRobot();Q_EMIT configChanged();});
}
TelemetryPanel::~TelemetryPanel(){timer_->stop();node_.reset();}
void TelemetryPanel::onInitialize(){node_=std::make_shared<rclcpp::Node>("a2r_telemetry_panel");connectRobot();timer_->start();}
void TelemetryPanel::save(rviz_common::Config c) const {Panel::save(c);c.mapSetValue("Robot Namespace",namespace_->text());}
void TelemetryPanel::load(const rviz_common::Config &c) {Panel::load(c);QString ns;if(c.mapGetString("Robot Namespace",&ns))namespace_->setText(ns);if(node_)connectRobot();}
void TelemetryPanel::connectRobot(){
 if(!node_)return;
 QString ns=namespace_->text().trimmed();while(ns.startsWith('/'))ns.remove(0,1);while(ns.endsWith('/'))ns.chop(1);
 if(ns.isEmpty()||ns.contains(' ')){connection_->setText("Enter a robot namespace, e.g. jetson1");return;}
 const auto prefix="/"+ns.toStdString();
 enc_.reset();status_.reset();imu_.reset();joy_.reset();safety_.clear();
 for(auto &v:values_)v.second->setText(QString::fromUtf8("—"));
 auto qos=rclcpp::SensorDataQoS().keep_last(1);
 enc_sub_=node_->create_subscription<a2r_robot_interfaces::msg::EncoderState>(prefix+"/encoders",qos,[this](a2r_robot_interfaces::msg::EncoderState::SharedPtr m){enc_=m;enc_at_=Clock::now();});
 status_sub_=node_->create_subscription<a2r_robot_interfaces::msg::HardwareStatus>(prefix+"/hardware/status",qos,[this](a2r_robot_interfaces::msg::HardwareStatus::SharedPtr m){status_=m;status_at_=Clock::now();});
 imu_sub_=node_->create_subscription<sensor_msgs::msg::Imu>(prefix+"/imu/data",qos,[this](sensor_msgs::msg::Imu::SharedPtr m){imu_=m;imu_at_=Clock::now();});
 joy_sub_=node_->create_subscription<sensor_msgs::msg::Joy>(prefix+"/joy",qos,[this](sensor_msgs::msg::Joy::SharedPtr m){joy_=m;joy_at_=Clock::now();});
 safety_sub_=node_->create_subscription<std_msgs::msg::String>(prefix+"/safety/status",qos,[this](std_msgs::msg::String::SharedPtr m){safety_=QString::fromStdString(m->data);});
}
void TelemetryPanel::value(const std::string &key,const QString &text){values_.at(key)->setText(text);}
void TelemetryPanel::refresh(){
 if(!node_||!rclcpp::ok())return;
 rclcpp::spin_some(node_);
 bool live=status_&&fresh(status_at_);
 connection_->setText(live?"● LIVE — "+namespace_->text():"● STALE / waiting — "+namespace_->text());
 connection_->setStyleSheet(live?"color:#39b98b; font-weight:bold":"color:#e8a34b; font-weight:bold");
 if(live){const auto &s=*status_;
  value("permission",s.drive_enabled?"LB manual / RB autonomy enabled":"MOTION DISABLED");
  value("applied",number(s.applied_left_m_s)+" / "+number(s.applied_right_m_s));
  value("uart",s.uart_ok?"OK":"FAULT");value("sensors",QString(s.imu_fresh?"LIVE":"STALE")+" / "+(s.encoders_fresh?"LIVE":"STALE"));
  value("devices",QString(s.joystick_connected?"connected":"missing")+" / "+(s.camera_fresh?"LIVE":"STALE"));
  value("reason",QString::fromStdString(s.reason));value("safety",safety_);
 }else{value("permission","STALE");value("applied",QString::fromUtf8("—"));value("reason","No fresh hardware status");}
 if(joy_&&fresh(joy_at_)&&joy_->buttons.size()>5)value("buttons",QString(joy_->buttons[4]?"LB HELD":"LB released")+" / "+(joy_->buttons[5]?"RB HELD":"RB released")+" / "+(joy_->buttons[0]?"A HELD":"A released"));else value("buttons","STALE");
 bool enc_live=enc_&&fresh(enc_at_);value("enc_stream",enc_live?"LIVE":"STALE");
 if(enc_){const auto &e=*enc_;value("l_ticks",QString::number(e.left_ticks));value("r_ticks",QString::number(e.right_ticks));
  value("l_rel",QString::number(e.left_relative_ticks));value("r_rel",QString::number(e.right_relative_ticks));
  value("l_distance",number(e.left_distance_m,4));value("r_distance",number(e.right_distance_m,4));
  value("l_speed",enc_live?number(e.left_speed_m_s):"STALE");value("r_speed",enc_live?number(e.right_speed_m_s):"STALE");
  value("l_scale",number(e.left_metres_per_tick*1000,6));value("r_scale",number(e.right_metres_per_tick*1000,6));}
 bool imu_live=imu_&&fresh(imu_at_);value("imu_stream",imu_live?"LIVE":"STALE");
 if(imu_){const auto &q=imu_->orientation;double w=q.w,x=q.x,y=q.y,z=q.z;constexpr double DEG=57.29577951308232;
  value("front",number(std::asin(std::clamp(2*(x*z-w*y),-1.,1.))*DEG,1));
  value("left",number(std::asin(std::clamp(2*(y*z+w*x),-1.,1.))*DEG,1));
  value("yaw",number(std::atan2(2*(w*z+x*y),1-2*(y*y+z*z))*DEG,1));
  value("quaternion",number(w)+" "+number(x)+" "+number(y)+" "+number(z));
  const auto &a=imu_->linear_acceleration;const auto &g=imu_->angular_velocity;
  value("accel",number(a.x)+" / "+number(a.y)+" / "+number(a.z));value("gyro",number(g.x)+" / "+number(g.y)+" / "+number(g.z));}
}
}
PLUGINLIB_EXPORT_CLASS(a2r_rviz_panels::TelemetryPanel,rviz_common::Panel)
