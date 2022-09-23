#include "selfdrive/ui/qt/offroad/settings.h"

#include <cassert>
#include <string>

#include <QDebug>

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "selfdrive/common/params.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"

TogglesPanel::TogglesPanel(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);

  QList<ParamControl*> toggles;

  toggles.append(new ParamControl("OpenpilotEnabledToggle",
                                  "Enable openpilot",
                                  "Use the openpilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature. Changing this setting takes effect when the car is powered off.",
                                  "../assets/offroad/icon_openpilot.png",
                                  this));
  toggles.append(new ParamControl("IsLdwEnabled",
                                  "Enable Lane Departure Warnings",
                                  "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31mph (50kph).",
                                  "../assets/offroad/icon_warning.png",
                                  this));
  toggles.append(new ParamControl("IsRHD",
                                  "Enable Right-Hand Drive",
                                  "Allow openpilot to obey left-hand traffic conventions and perform driver monitoring on right driver seat.",
                                  "../assets/offroad/icon_openpilot_mirrored.png",
                                  this));
  toggles.append(new ParamControl("IsMetric",
                                  "Use Metric System",
                                  "Display speed in km/h instead of mp/h.",
                                  "../assets/offroad/icon_metric.png",
                                  this));
  toggles.append(new ParamControl("UploadRaw",
                                  "Upload Raw Logs",
                                  "WiFi를 사용하는 동안 로그 및 풀해상도로 비디오 업로드",
                                  "../assets/offroad/icon_network.png",
                                  this));

  toggles.append(new ParamControl("DisableOnroadUploads",
                                  "Disable onroad uploads",
                                  "이동중 업로드 완전비활성화. 데이터 사용을 방지하기 위해 필요.",
                                  "../assets/offroad/icon_network.png",
                                  this));

  ParamControl *record_toggle = new ParamControl("RecordFront",
                                                 "Record and Upload Driver Camera",
                                                 "전면 카메라에서 운전자 데이터를 업로드하고 운전자 감시알고리즘을 개선합니다.",
                                                 "../assets/offroad/icon_monitoring.png",
                                                 this);
  toggles.append(record_toggle);
  toggles.append(new ParamControl("EndToEndToggle",
                                  "\U0001f96c Disable use of lanelines (Alpha) \U0001f96c",
                                  "차선을 무시하고 사람이 하는 것처럼 운전한다.",
                                  "../assets/offroad/icon_road.png",
                                  this));

  toggles.append(new ParamControl("EVConsumptionReset",
                                  "Reset trip/EV metrics",
                                  "Upon the next vehicle start, reset the distance travelled and EV consumption and efficiency trip and 5mi/8km metrics to 0.",
                                  "../assets/offroad/icon_calibration.png",
                                  this));
  
  toggles.append(new ParamControl("LongRangeLeadsEnabled",
                                  "Longer-range lead detection (alpha)",
                                  "Use the much longer-range lead detection ability of the car's LKA camera to detect leads up to 40\% farther than stock openpilot.  This also allows for 10\% longer range detection using radar.",
                                  "../assets/offroad/icon_plus.png",
                                  this));

  toggles.append(new ParamControl("EnableTorqueControl",
                                  "Enable \"torque\" steering control",
                                  "(재시동해야 반영됨) 목표 조향각도 보다 목표 수평가속도에 기반한 토크조향.",
                                  "../assets/offroad/icon_openpilot.png",
                                  this));

  toggles.append(new ParamControl("HandsOnWheelMonitoring",
                                  "Enable Hands on Wheel Monitoring",
                                  "운전자가 스티어링 휠에 손을 대지 않을 때 모니터링 및 경고.",
                                  "../assets/offroad/icon_hands_on_wheel.png",
                                  this));
  toggles.append(new ParamControl("TurnVisionControl",
                                  "Enable vision based turn control",
                                  "비전 경로예측을 사용하여 적합한 턴어라운드 주행속도 예측.",
                                  "../assets/offroad/icon_slow_curves_vision.png",
                                  this));
  toggles.append(new ParamControl("TurnSpeedControl",
                                  "Enable Map Data Turn Control",
                                  "지도 데이터의 곡률정보로부터 커버주행에 필요한 제한속도 확정.",
                                  "../assets/offroad/icon_slow_curves_map.png",
                                  this));
  toggles.append(new ParamControl("SpeedLimitControl",
                                  "Enable Speed Limit Control",
                                  "지도 데이터의 속도제한 정보와 차량 인터페이스 정보를 사용하여 자동으로 크루즈 속도를 도로 제한속도에 적용시킴.",
                                  "../assets/offroad/icon_speed_limit_sign.png",
                                  this));
  toggles.append(new ParamControl("EUSpeedLimitStyle",
                                  "Show EU style speed limit sign",
                                  "If enabled, show EU style circular sign. If disabled, show US/Canada style rectangular sign.",
                                  "../assets/offroad/icon_speed_limit_sign.png",
                                  this));
  toggles.append(new ParamControl("SpeedLimitPercOffset",
                                  "Enable Speed Limit Offset",
                                  "보다 자연스러운 드라이브를 위해 실제 속도 제한보다 약간 높은 속도제한을 설정.",
                                  "../assets/offroad/icon_speed_limit_percent.png",
                                  this));
  toggles.append(new ParamControl("StockSpeedAdjust",
                                  "순정 크루즈속도와 반대로 설정",
                                  "순정동작의 반대로, 가속/감속 버튼을 짧게/길게 누르면 5mph/1mph씩 변경됩니다.",
                                  "../assets/offroad/icon_stock_adjust_speed.png",
                                  this));
  toggles.append(new ParamControl("CruiseSpeedOffset",
                                  "크루즈속도 옵셑(3mph/5kph)",
                                  "적용시 순항 속도는 {5, 10, 15, 20, 25}kph가 됩니다",
                                  "../assets/offroad/icon_speed_offset.png",
                                  this));
  toggles.append(new ParamControl("DisableDisengageOnGas",
                                  "Disable disengage on gas",
                                  "액셀(가스)페달시 인게이지 해제 비활성화.",
                                  "../assets/offroad/icon_car_pedal.png",
                                  this));
  toggles.append(new ParamControl("LanePositionEnabled",
                                  "차선위치 조정",
                                  "일시적으로 화살표로 차선위치 조정이 가능함. 왼쪽 또는 오른쪽 끝차에서 주행하고 있을 때 옆차와 추돌을 피하기 위한 자동모드는 화살표 두개를 차레로 투르면 활성화 됨.",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("AutoAutoLanePosition",
                                  "36Kmh이상에서 차선위치 자동조정",
                                  "고속도로(시속36Kmh이상)나 고속도로에 진입할 때 자동으로 차선 위치를 조정합니다. 위 차선위치조정 토글이 활성화되어야 합니다. 차가 왼쪽 혹은 오른쪽 끝차선을 주행하고 있을 때, 옆차와의 간격을 자동으로 멀어지게 함.",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("AccelModeButton",
                                  "가속 프로파일",
                                  "일반, 스포츠 및 에코 가속 프로파일",
                                  "../assets/offroad/icon_rocket.png",
                                  this));
  toggles.append(new ParamControl("DynamicFollowToggle",
                                  "Dynamic follow",
                                  "속도와 트래픽에 따른 근접/중간/원거리 추종 프로파일",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("NudgelessLaneChange",
                                  "Nudgeless lane change (1s delay) ⚠️",
                                  "Perform lane change without requiring nudge from driver",
                                  "../assets/offroad/icon_hands_on_wheel.png",
                                  this));
  toggles.append(new ParamControl("GMAutoHold",
                                  "[GM] 오토홀드",
                                  "Holds brakes automatically after coming to a complete stop, even when OP is disengaged.",
                                  "../assets/offroad/icon_gm_autohold.png",
                                  this));
  toggles.append(new ParamControl("Coasting",
                                  "[GM] 타력 주행",
                                  "엔진/리젠/마찰 브레이크가 작동되지 않으며, 대신 설정된 속도 이상으로 타력주행하게 됩니다. \"Brake indicator\" 토글이 활성화된 경우 브레이크 표시등을 눌러 주행중 타력주행토글이 가능하지만, 설정된 속도보다 낮게 주행하는 경우(또는 \"Engine/regen braking\" 토글이 활성화된 경우)에만 주행중 타력 주행이 비활성화됩니다.",
                                  "../assets/offroad/icon_car_pedal.png",
                                  this));
  toggles.append(new ParamControl("CoastingBrakeOverSpeed",
                                  "[GM] Coast: brake 15% over set speed",
                                  "타력 주행 시 설정속도의 15% 초과되면 크루즈 제동시작.",
                                  "../assets/offroad/icon_speed_offset.png",
                                  this));
  toggles.append(new ParamControl("CoastingDL",
                                  "[Volt] D/L 타력주행 컨트롤",
                                  "VOLT차량에서, D/L 기어위치와 연동. D에서는 타력 주행; L에서는 설정속도 유지.",
                                  "../assets/offroad/icon_gear_shifter.png",
                                  this));
  toggles.append(new ParamControl("RegenBraking",
                                  "[GM] 엔진/리젠 브레이킹",
                                  "오파가 크루즈속도 유지를 위해 감속할 때 마찰 제동 비활성화; 추종/커브에 대해서는 그대로 제동.",
                                  "../assets/img_brake.png",
                                  this));
  toggles.append(new ParamControl("OnePedalMode",
                                  "[GM] One-pedal mode (tap me)",
                                  "\"Disable disengage on gas\"옵션과 연동됨. OP가 커브길과 앞차량 추종시 조향과 브레이크를 계속하는 동안 -조정가능한 제동옵션이 포함된- 가속페달을 사용하여 속도를 제어할 수 있습니다. 활성화하려면 One-pedal engage on gas를 참조 일반 크루즈운행으로 돌아가려면 SET, RESUME 버튼을 누르십시오. ② 페달 아이콘을 눌러 원페달모드로 전환합니다 아래 참조 one-pedal 모드가 활성화되면 차량이 거리 표시기를 따르고 페달 아이콘 색상이 1/2/3 = (⚫️)/🟢/🟠/🔴 = (regen/engine)/light/moderate/heavy braking 제동 중임을 나타냅니다. ③ 추종거리 버튼을 눌러 light/moderate braking 사이를 전환하게 합니다; 제동 사이를 전환하고, heavy braking을 유지하십시오. ④ 페달 아이콘을 탭하거나 추종 거리 버튼을 사용하여, 마찰 브레이킹 🟢/🟠/🔴 및 리젠/엔진⚫제동 사이를 전환합니다; 한번 탭하면 마찰 제동이 활성화되고, 두번 탭하면 가속페달을 밟고 있거나, 정차중에는 마찰 마찰 제동이 비활성화됩니다.",
                                  "../assets/offroad/icon_car_pedal.png",
                                  this));
  toggles.append(new ParamControl("OnePedalDLCoasting",
                                  "[Volt] One-pedal D/L coast",
                                  "VOLT차량에서 원페달모드, D에서는 엔진,마찰,회생제동 브레이크가 전혀 작동되지 않습니다. L기어에서는 회생제동이 작동됨.",
                                  "../assets/offroad/icon_gear_shifter.png",
                                  this));
  toggles.append(new ParamControl("OnePedalModeEngageOnGas",
                                  "[GM] One-pedal engage on gas (EoG)",
                                  "GM 크루즈모드에서 가속페달을 밟을 때(즉, 정지상태에서의 리쥼할 때가 아닌)는, 원페달모드로 진입됨. 정상 크루즈모드로 돌아가려면 페달을 밟거나 리셑버튼을 누름.",
                                  "../assets/offroad/icon_car_pedal.png",
                                  this));
  toggles.append(new ParamControl("OnePedalDLEngageOnGas",
                                  "[Volt] One-pedal D/L EoG",
                                  "VOLT차량에서 D/L 타력주행과 연동됨. 가스페달 중에도 인에이지 기능이, D에서는 꺼지고 L에서는 켜진다.",
                                  "../assets/offroad/icon_gear_shifter.png",
                                  this));
  toggles.append(new ParamControl("OnePedalPauseBlinkerSteering",
                                  "One-pedal no slow blinker steer",
                                  "원페달 모드에서, 20mph(72kph)미만시 자동 스티어링이 일시 중지됨",
                                  "../assets/offroad/icon_hands_on_wheel.png",
                                  this));
  toggles.append(new ParamControl("BrakeIndicator",
                                  "[GM] Brake indicator",
                                  "주행중 우측 하단에 있는 브레이크 표시등. 표시기 중앙에 있는 원이 커지고 빨간색으로 바뀌어 제동 수준을 나타냅니다. 시동 걸자마자 깜박이면서 작동 중임을 알립니다.",
                                  "../assets/offroad/icon_brake_disc.png",
                                  this));
  toggles.append(new ParamControl("CustomSounds",
                                  "Alternative sounds",
                                  "대체 사운드 사용",
                                  "../assets/offroad/icon_custom_sounds.png",
                                  this));
  toggles.append(new ParamControl("SilentEngageDisengage",
                                  "Silent engage/disengage",
                                  "engage/disengage 사운드 묵음.",
                                  "../assets/offroad/icon_mute.png",
                                  this));
  toggles.append(new ParamControl("IgnoreMissingNVME",
                                  "Ignore missing NVME",
                                  "누락된 NVME 드라이브가 32GB C3에 표시되지 않도록 방지다(적용하려면 장치재시작하십시오).",
                                  "../assets/offroad/icon_settings.png",
                                  this));
  toggles.append(new ParamControl("FPVolt",
                                  "Volt Fingerprint",
                                  "Volt 핑거 강제하기",
                                  "../assets/offroad/icon_settings.png",
                                  this));
  toggles.append(new ParamControl("LowOverheadMode",
                                  "Lower device overhead",
                                  "오래된 하드웨어를 더 오래 사용하기 위해 전력, CPU, 저장용량 줄이기 1) 중간밝기 사용(DM 아이콘 탭) 2) 온로드 로깅 비활성화(loggerd 및 proclogd).",
                                  "../assets/offroad/icon_settings.png",
                                  this));
  toggles.append(new ParamControl("ColorPath",
                                  "Colored path",
                                  "조향(스티어링)에 따른 경로색 바꾸기.",
                                  "../assets/offroad/icon_road.png",
                                  this));
  toggles.append(new ParamControl("PrintLeadInfo",
                                  "Print lead car info",
                                  "리드카 표시기 옆에 리드차량 도달시간 및 거리, 절대 및 상대속도 표시.",
                                  "../assets/offroad/icon_metric.png",
                                  this));
  toggles.append(new ParamControl("ShowDebugUI",
                                  "Show debug UI elements",
                                  "디버깅을 위해 UI 표시.",
                                  "../assets/offroad/icon_calibration.png",
                                  this));

#ifdef ENABLE_MAPS
  toggles.append(new ParamControl("NavSettingTime24h",
                                  "24시간제 표시",
                                  "Use 24h format instead of am/pm",
                                  "../assets/offroad/icon_metric.png",
                                  this));
#endif

  bool record_lock = Params().getBool("RecordFrontLock");
  record_toggle->setEnabled(!record_lock);

  for(ParamControl *toggle : toggles) {
    if(main_layout->count() != 0) {
      main_layout->addWidget(horizontal_line());
    }
    main_layout->addWidget(toggle);
  }
}

DevicePanel::DevicePanel(QWidget* parent) : QWidget(parent) {
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  Params params = Params();
  main_layout->addWidget(new LabelControl("Dongle ID", getDongleId().value_or("N/A")));
  main_layout->addWidget(horizontal_line());

  QString serial = QString::fromStdString(params.get("HardwareSerial", false));
  main_layout->addWidget(new LabelControl("Serial", serial));

  QHBoxLayout *reset_layout = new QHBoxLayout();
  reset_layout->setSpacing(30);

  // reset calibration button
  QPushButton *restart_openpilot_btn = new QPushButton("Soft restart");
  restart_openpilot_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(restart_openpilot_btn);
  QObject::connect(restart_openpilot_btn, &QPushButton::released, [=]() {
    emit closeSettings();
    QTimer::singleShot(1000, []() {
      Params().putBool("SoftRestartTriggered", true);
    });
  });

  main_layout->addWidget(horizontal_line());
  main_layout->addLayout(reset_layout);

  // reset calibration button
  QPushButton *reset_calib_btn = new QPushButton("Reset Calibration");
  reset_calib_btn->setStyleSheet("height: 120px;border-radius: 15px;background-color: #393939;");
  reset_layout->addWidget(reset_calib_btn);
  QObject::connect(reset_calib_btn, &QPushButton::released, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration and live params?", this)) {
      Params().remove("CalibrationParams");
      Params().remove("LiveParameters");
      emit closeSettings();
      QTimer::singleShot(1000, []() {
        Params().putBool("SoftRestartTriggered", true);
      });
    }
  });

  main_layout->addLayout(reset_layout);

  // offroad-only buttons

  auto dcamBtn = new ButtonControl("Driver Camera", "PREVIEW",
                                        "Preview the driver facing camera to help optimize device mounting position for best driver monitoring experience. (vehicle must be off)");
  connect(dcamBtn, &ButtonControl::clicked, [=]() { emit showDriverView(); });

  QString resetCalibDesc = "openpilot requires the device to be mounted within 4° left or right and within 5° up or down. openpilot is continuously calibrating, resetting is rarely required.";
  auto resetCalibBtn = new ButtonControl("Reset Calibration", "RESET", resetCalibDesc);
  connect(resetCalibBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration?", this)) {
      Params().remove("CalibrationParams");
    }
  });
  connect(resetCalibBtn, &ButtonControl::showDescription, [=]() {
    QString desc = resetCalibDesc;
    std::string calib_bytes = Params().get("CalibrationParams");
    if (!calib_bytes.empty()) {
      try {
        AlignedBuffer aligned_buf;
        capnp::FlatArrayMessageReader cmsg(aligned_buf.align(calib_bytes.data(), calib_bytes.size()));
        auto calib = cmsg.getRoot<cereal::Event>().getLiveCalibration();
        if (calib.getCalStatus() != 0) {
          double pitch = calib.getRpyCalib()[1] * (180 / M_PI);
          double yaw = calib.getRpyCalib()[2] * (180 / M_PI);
          desc += QString(" Your device is pointed %1° %2 and %3° %4.")
                                .arg(QString::number(std::abs(pitch), 'g', 1), pitch > 0 ? "up" : "down",
                                     QString::number(std::abs(yaw), 'g', 1), yaw > 0 ? "right" : "left");
        }
      } catch (kj::Exception) {
        qInfo() << "invalid CalibrationParams";
      }
    }
    resetCalibBtn->setDescription(desc);
  });

  ButtonControl *retrainingBtn = nullptr;
  if (!params.getBool("Passive")) {
    retrainingBtn = new ButtonControl("Review Training Guide", "REVIEW", "Review the rules, features, and limitations of openpilot");
    connect(retrainingBtn, &ButtonControl::clicked, [=]() {
      if (ConfirmationDialog::confirm("Are you sure you want to review the training guide?", this)) {
        Params().remove("CompletedTrainingVersion");
        emit reviewTrainingGuide();
      }
    });
  }

  ButtonControl *regulatoryBtn = nullptr;
  if (Hardware::TICI()) {
    regulatoryBtn = new ButtonControl("Regulatory", "VIEW", "");
    connect(regulatoryBtn, &ButtonControl::clicked, [=]() {
      const std::string txt = util::read_file(ASSET_PATH.toStdString() + "/offroad/fcc.html");
      RichTextDialog::alert(QString::fromStdString(txt), this);
    });
  }

  for (auto btn : {dcamBtn, resetCalibBtn, retrainingBtn, regulatoryBtn}) {
    if (btn) {
      main_layout->addWidget(horizontal_line());
      connect(parent, SIGNAL(offroadTransition(bool)), btn, SLOT(setEnabled(bool)));
      main_layout->addWidget(btn);
    }
  }

  // power buttons
  QHBoxLayout *power_layout = new QHBoxLayout();
  power_layout->setSpacing(30);

  QPushButton *reboot_btn = new QPushButton("Reboot");
  reboot_btn->setObjectName("reboot_btn");
  power_layout->addWidget(reboot_btn);
  QObject::connect(reboot_btn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reboot?", this)) {
      Hardware::reboot();
    }
  });

  QPushButton *poweroff_btn = new QPushButton("Power Off");
  poweroff_btn->setObjectName("poweroff_btn");
  power_layout->addWidget(poweroff_btn);
  QObject::connect(poweroff_btn, &QPushButton::clicked, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to power off?", this)) {
      Hardware::poweroff();
    }
  });

  setStyleSheet(R"(
    QPushButton {
      height: 120px;
      border-radius: 15px;
    }
    #reboot_btn { background-color: #393939; }
    #reboot_btn:pressed { background-color: #4a4a4a; }
    #poweroff_btn { background-color: #E22C2C; }
    #poweroff_btn:pressed { background-color: #FF2424; }
  )");
  main_layout->addLayout(power_layout);
}

SoftwarePanel::SoftwarePanel(QWidget* parent) : QWidget(parent) {
  gitBranchLbl = new LabelControl("Git Branch");
  gitCommitLbl = new LabelControl("Git Commit");
  osVersionLbl = new LabelControl("OS Version");
  versionLbl = new LabelControl("Version", "", QString::fromStdString(params.get("ReleaseNotes")).trimmed());
  lastUpdateLbl = new LabelControl("Last Update Check", "", "The last time openpilot successfully checked for an update. The updater only runs while the car is off.");
  updateBtn = new ButtonControl("Check for Update", "");
  connect(updateBtn, &ButtonControl::clicked, [=]() {
    if (params.getBool("IsOffroad")) {
      fs_watch->addPath(QString::fromStdString(params.getParamPath("LastUpdateTime")));
      fs_watch->addPath(QString::fromStdString(params.getParamPath("UpdateFailedCount")));
      updateBtn->setText("CHECKING");
      updateBtn->setEnabled(false);
    }
    std::system("pkill -1 -f selfdrive.updated");
  });

  QVBoxLayout *main_layout = new QVBoxLayout(this);
  QWidget *widgets[] = {versionLbl, lastUpdateLbl, updateBtn, gitBranchLbl, gitCommitLbl, osVersionLbl};
  for (int i = 0; i < std::size(widgets); ++i) {
    main_layout->addWidget(widgets[i]);
    main_layout->addWidget(horizontal_line());
  }

  auto uninstallBtn = new ButtonControl("Uninstall " + getBrand(), "UNINSTALL");
  connect(uninstallBtn, &ButtonControl::clicked, [=]() {
    if (ConfirmationDialog::confirm("Are you sure you want to uninstall?", this)) {
      Params().putBool("DoUninstall", true);
    }
  });
  connect(parent, SIGNAL(offroadTransition(bool)), uninstallBtn, SLOT(setEnabled(bool)));
  main_layout->addWidget(uninstallBtn);

  fs_watch = new QFileSystemWatcher(this);
  QObject::connect(fs_watch, &QFileSystemWatcher::fileChanged, [=](const QString path) {
    int update_failed_count = params.get<int>("UpdateFailedCount").value_or(0);
    if (path.contains("UpdateFailedCount") && update_failed_count > 0) {
      lastUpdateLbl->setText("failed to fetch update");
      updateBtn->setText("CHECK");
      updateBtn->setEnabled(true);
    } else if (path.contains("LastUpdateTime")) {
      updateLabels();
    }
  });
}

void SoftwarePanel::showEvent(QShowEvent *event) {
  updateLabels();
}

void SoftwarePanel::updateLabels() {
  QString lastUpdate = "";
  auto tm = params.get("LastUpdateTime");
  if (!tm.empty()) {
    lastUpdate = timeAgo(QDateTime::fromString(QString::fromStdString(tm + "Z"), Qt::ISODate));
  }

  versionLbl->setText(getBrandVersion());
  lastUpdateLbl->setText(lastUpdate);
  updateBtn->setText("CHECK");
  updateBtn->setEnabled(true);
  gitBranchLbl->setText(QString::fromStdString(params.get("GitBranch")));
  gitCommitLbl->setText(QString::fromStdString(params.get("GitCommit")).left(10));
  osVersionLbl->setText(QString::fromStdString(Hardware::get_os_version()).trimmed());
}

QWidget * network_panel(QWidget * parent) {
#ifdef QCOM
  QWidget *w = new QWidget(parent);
  QVBoxLayout *layout = new QVBoxLayout(w);
  layout->setSpacing(30);

  // wifi + tethering buttons
  auto wifiBtn = new ButtonControl("WiFi Settings", "OPEN");
  QObject::connect(wifiBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_wifi(); });
  layout->addWidget(wifiBtn);
  layout->addWidget(horizontal_line());

  auto tetheringBtn = new ButtonControl("Tethering Settings", "OPEN");
  QObject::connect(tetheringBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_tethering(); });
  layout->addWidget(tetheringBtn);
  layout->addWidget(horizontal_line());

  // SSH key management
  layout->addWidget(new SshToggle());
  layout->addWidget(horizontal_line());
  layout->addWidget(new SshControl());

  layout->addStretch(1);
#else
  Networking *w = new Networking(parent);
#endif
  return w;
}

void SettingsWindow::showEvent(QShowEvent *event) {
  panel_widget->setCurrentIndex(0);
  nav_btns->buttons()[0]->setChecked(true);
}

SettingsWindow::SettingsWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 30px;
    background-color: #292929;
  )");

  // close button
  QPushButton *close_btn = new QPushButton("×");
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 140px;
      padding-bottom: 20px;
      font-weight: bold;
      border 1px grey solid;
      border-radius: 100px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(200, 200);
  sidebar_layout->addSpacing(45);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, &QPushButton::clicked, this, &SettingsWindow::closeSettings);

  // setup panels
  DevicePanel *device = new DevicePanel(this);
  QObject::connect(device, &DevicePanel::reviewTrainingGuide, this, &SettingsWindow::reviewTrainingGuide);
  QObject::connect(device, &DevicePanel::showDriverView, this, &SettingsWindow::showDriverView);
  QObject::connect(device, &DevicePanel::closeSettings, this, &SettingsWindow::closeSettings);

  QList<QPair<QString, QWidget *>> panels = {
    {"Device", device},
    {"Network", network_panel(this)},
    {"Toggles", new TogglesPanel(this)},
    {"Software", new SoftwarePanel(this)},
  };

#ifdef ENABLE_MAPS
  auto map_panel = new MapPanel(this);
  panels.push_back({"Navigation", map_panel});
  QObject::connect(map_panel, &MapPanel::closeSettings, this, &SettingsWindow::closeSettings);
#endif

  const int padding = panels.size() > 3 ? 25 : 35;

  nav_btns = new QButtonGroup();
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 65px;
        font-weight: 500;
        padding-top: %1px;
        padding-bottom: %1px;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )").arg(padding));

    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignRight);

    const int lr_margin = name != "Network" ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    SettingsWindow {
      background-color: black;
    }
  )");
}

void SettingsWindow::hideEvent(QHideEvent *event) {
#ifdef QCOM
  HardwareEon::close_activities();
#endif
}
