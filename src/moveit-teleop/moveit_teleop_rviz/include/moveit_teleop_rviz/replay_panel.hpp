// RViz panel for reviewing recorded teleop episodes.
//
// The panel holds no playback state of its own. Every widget is a view onto the
// last ReplayStatus, and every interaction is one ReplayCommand service call.
// That is what keeps the panel and the player from ever disagreeing about what
// is loaded -- a panel with its own idea of the current frame drifts out of
// sync the moment anything else seeks, and then lies to the operator.
#ifndef MOVEIT_TELEOP_RVIZ__REPLAY_PANEL_HPP_
#define MOVEIT_TELEOP_RVIZ__REPLAY_PANEL_HPP_

#include <memory>
#include <string>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QCheckBox>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "moveit_teleop_msgs/msg/replay_status.hpp"
#include "moveit_teleop_msgs/srv/replay_command.hpp"

namespace moveit_teleop_rviz
{

class ReplayPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ReplayPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void onPlayPause();
  void onStepBack();
  void onStepForward();
  void onRestart();
  void onEpisodeChosen(int index);
  void onSliderMoved(int value);
  void onSliderReleased();
  void onSpeedChanged(double speed);
  void onLoopToggled(bool loop);
  void onRefresh();

private:
  using ReplayCommand = moveit_teleop_msgs::srv::ReplayCommand;
  using ReplayStatus = moveit_teleop_msgs::msg::ReplayStatus;

  void send(uint8_t command, const std::string & episode = "",
            int32_t index = 0, float speed = 0.0f, bool loop = false);
  void statusReceived(const ReplayStatus::SharedPtr msg);
  void refreshWidgets();
  QString describePosition() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<ReplayStatus>::SharedPtr status_sub_;
  rclcpp::Client<ReplayCommand>::SharedPtr client_;

  ReplayStatus status_;
  bool have_status_{false};
  // While the operator drags, the slider is theirs; adopting incoming status
  // would yank the handle out from under them on every update.
  bool scrubbing_{false};

  QComboBox * episodes_{nullptr};
  QPushButton * play_{nullptr};
  QPushButton * back_{nullptr};
  QPushButton * forward_{nullptr};
  QPushButton * restart_{nullptr};
  QPushButton * refresh_{nullptr};
  QCheckBox * loop_{nullptr};
  QDoubleSpinBox * speed_{nullptr};
  QSlider * position_{nullptr};
  QLabel * readout_{nullptr};
  QLabel * detail_{nullptr};
  QTimer * spin_timer_{nullptr};
};

}  // namespace moveit_teleop_rviz

#endif  // MOVEIT_TELEOP_RVIZ__REPLAY_PANEL_HPP_
