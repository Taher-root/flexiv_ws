#include "moveit_teleop_rviz/replay_panel.hpp"

#include <algorithm>
#include <memory>
#include <string>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace moveit_teleop_rviz
{

namespace
{
// TeleopStatus.mode == TELEOP. Used only to describe the timeline.
constexpr uint8_t kModeTeleop = 2;

QString formatSeconds(float seconds)
{
  const int total = static_cast<int>(seconds);
  return QString("%1:%2")
    .arg(total / 60, 2, 10, QChar('0'))
    .arg(total % 60, 2, 10, QChar('0'));
}
}  // namespace

ReplayPanel::ReplayPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  episodes_ = new QComboBox;
  episodes_->setToolTip("Recorded episodes, newest first.");
  refresh_ = new QPushButton("Rescan");
  refresh_->setToolTip("Look for episodes recorded since this panel started.");

  restart_ = new QPushButton("|<");
  back_ = new QPushButton("<");
  play_ = new QPushButton("Play");
  forward_ = new QPushButton(">");
  restart_->setToolTip("Back to the first frame");
  back_->setToolTip("Previous frame");
  forward_->setToolTip("Next frame");

  speed_ = new QDoubleSpinBox;
  speed_->setRange(0.05, 8.0);
  speed_->setSingleStep(0.25);
  speed_->setValue(1.0);
  speed_->setSuffix("x");
  speed_->setToolTip("Playback speed relative to the recorded rate.");

  loop_ = new QCheckBox("Loop");

  position_ = new QSlider(Qt::Horizontal);
  position_->setRange(0, 0);
  position_->setToolTip("Scrub through the episode.");

  readout_ = new QLabel("no episode loaded");
  detail_ = new QLabel(" ");
  detail_->setWordWrap(true);
  QFont small = detail_->font();
  small.setPointSizeF(std::max(6.0, small.pointSizeF() - 1.0));
  detail_->setFont(small);

  auto * top = new QHBoxLayout;
  top->addWidget(episodes_, 1);
  top->addWidget(refresh_);

  auto * transport = new QHBoxLayout;
  transport->addWidget(restart_);
  transport->addWidget(back_);
  transport->addWidget(play_, 1);
  transport->addWidget(forward_);
  transport->addWidget(speed_);
  transport->addWidget(loop_);

  auto * layout = new QVBoxLayout;
  layout->addLayout(top);
  layout->addWidget(position_);
  layout->addWidget(readout_);
  layout->addLayout(transport);
  layout->addWidget(detail_);
  setLayout(layout);

  connect(play_, &QPushButton::clicked, this, &ReplayPanel::onPlayPause);
  connect(back_, &QPushButton::clicked, this, &ReplayPanel::onStepBack);
  connect(forward_, &QPushButton::clicked, this, &ReplayPanel::onStepForward);
  connect(restart_, &QPushButton::clicked, this, &ReplayPanel::onRestart);
  connect(refresh_, &QPushButton::clicked, this, &ReplayPanel::onRefresh);
  connect(loop_, &QCheckBox::toggled, this, &ReplayPanel::onLoopToggled);
  connect(
    episodes_, QOverload<int>::of(&QComboBox::activated),
    this, &ReplayPanel::onEpisodeChosen);
  connect(position_, &QSlider::sliderPressed, this, [this]() {scrubbing_ = true;});
  connect(position_, &QSlider::sliderMoved, this, &ReplayPanel::onSliderMoved);
  connect(position_, &QSlider::sliderReleased, this, &ReplayPanel::onSliderReleased);
  connect(
    speed_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &ReplayPanel::onSpeedChanged);
}

void ReplayPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Transient-local so the panel is correct the instant it connects, rather
  // than blank until the player next publishes.
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  status_sub_ = node_->create_subscription<ReplayStatus>(
    "/episode_player/status", qos,
    std::bind(&ReplayPanel::statusReceived, this, std::placeholders::_1));
  client_ = node_->create_client<ReplayCommand>("/episode_player/command");

  refreshWidgets();
}

void ReplayPanel::statusReceived(const ReplayStatus::SharedPtr msg)
{
  status_ = *msg;
  have_status_ = true;
  refreshWidgets();
}

void ReplayPanel::send(
  uint8_t command, const std::string & episode, int32_t index, float speed,
  bool loop)
{
  if (!client_ || !client_->service_is_ready()) {
    detail_->setText("episode_player is not running");
    return;
  }
  auto request = std::make_shared<ReplayCommand::Request>();
  request->command = command;
  request->episode = episode;
  request->index = index;
  request->speed = speed;
  request->loop = loop;
  // Fire and forget: the player answers by publishing status, which is the
  // single source the panel already renders from. Blocking here would freeze
  // the RViz UI thread.
  client_->async_send_request(request);
}

void ReplayPanel::onPlayPause()
{
  send(status_.playing ? ReplayCommand::Request::PAUSE
                       : ReplayCommand::Request::PLAY);
}

void ReplayPanel::onStepBack() {send(ReplayCommand::Request::STEP, "", -1);}
void ReplayPanel::onStepForward() {send(ReplayCommand::Request::STEP, "", 1);}
void ReplayPanel::onRestart() {send(ReplayCommand::Request::SEEK, "", 0);}
void ReplayPanel::onRefresh() {send(ReplayCommand::Request::REFRESH);}

void ReplayPanel::onLoopToggled(bool loop)
{
  send(ReplayCommand::Request::SET_LOOP, "", 0, 0.0f, loop);
}

void ReplayPanel::onEpisodeChosen(int index)
{
  if (index < 0 || index >= episodes_->count()) {
    return;
  }
  send(ReplayCommand::Request::LOAD, episodes_->itemText(index).toStdString());
}

void ReplayPanel::onSliderMoved(int value)
{
  // Seek live while dragging so the arm follows the handle. This is the whole
  // reason to have a slider rather than a progress bar.
  send(ReplayCommand::Request::SEEK, "", value);
}

void ReplayPanel::onSliderReleased()
{
  scrubbing_ = false;
  send(ReplayCommand::Request::SEEK, "", position_->value());
}

void ReplayPanel::onSpeedChanged(double speed)
{
  send(ReplayCommand::Request::SET_SPEED, "", 0, static_cast<float>(speed));
}

QString ReplayPanel::describePosition() const
{
  if (status_.count <= 0) {
    return "no episode loaded";
  }
  return QString("%1 / %2   frame %3 / %4")
    .arg(formatSeconds(status_.time))
    .arg(formatSeconds(status_.duration))
    .arg(status_.index + 1)
    .arg(status_.count);
}

void ReplayPanel::refreshWidgets()
{
  const bool loaded = have_status_ && status_.count > 0;
  play_->setText(status_.playing ? "Pause" : "Play");
  play_->setEnabled(loaded);
  back_->setEnabled(loaded);
  forward_->setEnabled(loaded);
  restart_->setEnabled(loaded);
  position_->setEnabled(loaded);

  // Rebuild the list only when it actually changed, so the drop-down does not
  // close itself under the operator every time status arrives.
  QStringList incoming;
  for (const auto & name : status_.episodes) {
    incoming << QString::fromStdString(name);
  }
  QStringList current;
  for (int i = 0; i < episodes_->count(); ++i) {
    current << episodes_->itemText(i);
  }
  if (incoming != current) {
    const QString selected = episodes_->currentText();
    QSignalBlocker block(episodes_);
    episodes_->clear();
    episodes_->addItems(incoming);
    const int restored = episodes_->findText(selected);
    if (restored >= 0) {
      episodes_->setCurrentIndex(restored);
    }
  }
  if (loaded) {
    const int at = episodes_->findText(QString::fromStdString(status_.episode));
    if (at >= 0 && at != episodes_->currentIndex()) {
      QSignalBlocker block(episodes_);
      episodes_->setCurrentIndex(at);
    }
  }

  if (!scrubbing_) {
    QSignalBlocker block(position_);
    position_->setRange(0, std::max(0, status_.count - 1));
    position_->setValue(status_.index);
  }
  if (std::abs(speed_->value() - status_.speed) > 1e-3 && status_.speed > 0.0f) {
    QSignalBlocker block(speed_);
    speed_->setValue(status_.speed);
  }
  if (loop_->isChecked() != status_.loop) {
    QSignalBlocker block(loop_);
    loop_->setChecked(status_.loop);
  }

  readout_->setText(describePosition());

  if (!status_.message.empty()) {
    detail_->setText(QString::fromStdString(status_.message));
  } else if (!have_status_) {
    detail_->setText("waiting for episode_player");
  } else if (status_.episodes.empty()) {
    detail_->setText("no episodes found");
  } else {
    // How much of this episode was actually engaged is the first thing worth
    // knowing about a recording, so it goes here rather than in a log.
    float engaged = 0.0f;
    for (size_t i = 0; i < status_.mode_values.size(); ++i) {
      if (status_.mode_values[i] != kModeTeleop) {
        continue;
      }
      const float from = status_.mode_times[i];
      const float to = (i + 1 < status_.mode_times.size())
        ? status_.mode_times[i + 1] : status_.duration;
      engaged += std::max(0.0f, to - from);
    }
    QString text = QString::fromStdString(status_.task.empty()
        ? std::string("task: (unset)") : "task: " + status_.task);
    if (status_.duration > 0.0f) {
      text += QString("   engaged %1%")
        .arg(100.0f * engaged / status_.duration, 0, 'f', 0);
    }
    detail_->setText(text);
  }
}

}  // namespace moveit_teleop_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_teleop_rviz::ReplayPanel, rviz_common::Panel)
