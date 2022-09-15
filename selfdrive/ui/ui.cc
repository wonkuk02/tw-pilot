#include "selfdrive/ui/ui.h"

#include <unistd.h>
#include <string>
#include <vector>

#include <cassert>
#include <cmath>
#include <cstdio>

#include <QDateTime>

#include "selfdrive/common/params.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/common/visionimg.h"
#include "selfdrive/common/watchdog.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/paint.h"
#include "selfdrive/ui/qt/qt_window.h"
#include "selfdrive/ui/dashcam.h"

#define BACKLIGHT_DT 0.05
#define BACKLIGHT_TS 10.00
#define BACKLIGHT_OFFROAD 75


static const float fade_duration = 0.3; // [s] time it takes for the brake indicator to fade in/out
static const float fade_time_step = 1. / fade_duration; // will step in the transparent or opaque direction

static const float dynamic_follow_fade_duration = 0.5;
static const float dynamic_follow_fade_step = 1. / dynamic_follow_fade_duration;

static const float voacc_lead_min_laneline_prob = 0.6; // should match MIN_LANE_PROB in selfdrive/controls/radard.py

// Given interpolate between engaged/warning/critical bg color on [0-1]
// If a < 0, interpolate that too based on bg color alpha, else pass through.
NVGcolor interp_alert_color(float p, int a){
  char c1, c2;
  if (p <= 0.){
    return (a < 0 ? nvgRGBA(bg_colors[STATUS_ENGAGED].red(), 
                            bg_colors[STATUS_ENGAGED].green(), 
                            bg_colors[STATUS_ENGAGED].blue(), 
                            bg_colors[STATUS_ENGAGED].alpha()) 
                  : nvgRGBA(bg_colors[STATUS_ENGAGED].red(), 
                            bg_colors[STATUS_ENGAGED].green(), 
                            bg_colors[STATUS_ENGAGED].blue(), a));
  }
  else if (p <= 0.5){
    c1 = STATUS_ENGAGED; // lower color index
    c2 = STATUS_WARNING; // higher color index
  }
  else if (p < 1.){
    p -= 0.5;
    c1 = STATUS_WARNING;
    c2 = STATUS_ALERT;
  }
  else{
    return (a < 0 ? nvgRGBA(bg_colors[STATUS_ALERT].red(), 
                            bg_colors[STATUS_ALERT].green(), 
                            bg_colors[STATUS_ALERT].blue(), 
                            bg_colors[STATUS_ALERT].alpha()) 
                  : nvgRGBA(bg_colors[STATUS_ALERT].red(), 
                            bg_colors[STATUS_ALERT].green(), 
                            bg_colors[STATUS_ALERT].blue(), a));
  }
  
  p *= 2.; // scale to 1
  
  int r, g, b;
  float complement = (1.f - p);
  r = bg_colors[c1].red() * complement + bg_colors[c2].red() * p;
  g = bg_colors[c1].green() * complement + bg_colors[c2].green() * p;
  b = bg_colors[c1].blue() * complement + bg_colors[c2].blue() * p;
  if (a < 0){
    a = bg_colors[c1].alpha() * complement + bg_colors[c2].alpha() * p;
  }
  
  NVGcolor out = nvgRGBA(r, g, b, a);
  
  return out;
}

// Projects a point in car to space to the corresponding point in full frame
// image space.
static bool calib_frame_to_full_frame(const UIState *s, float in_x, float in_y, float in_z, vertex_data *out) {
  const float margin = 500.0f;
  const vec3 pt = (vec3){{in_x, in_y, in_z}};
  const vec3 Ep = matvecmul3(s->scene.view_from_calib, pt);
  const vec3 KEp = matvecmul3(s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix, Ep);

  // Project.
  float x = KEp.v[0] / KEp.v[2];
  float y = KEp.v[1] / KEp.v[2];

  nvgTransformPoint(&out->x, &out->y, s->car_space_transform, x, y);
  return out->x >= -margin && out->x <= s->fb_w + margin && out->y >= -margin && out->y <= s->fb_h + margin;
}

static void ui_init_vision(UIState *s) {
  // Invisible until we receive a calibration message.
  s->scene.world_objects_visible = false;

  for (int i = 0; i < s->vipc_client->num_buffers; i++) {
    s->texture[i].reset(new EGLImageTexture(&s->vipc_client->buffers[i]));

    glBindTexture(GL_TEXTURE_2D, s->texture[i]->frame_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // BGR
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_GREEN);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
  }
  assert(glGetError() == GL_NO_ERROR);
}

static int get_path_length_idx(const cereal::ModelDataV2::XYZTData::Reader &line, const float path_height) {
  const auto line_x = line.getX();
  int max_idx = 0;
  for (int i = 0; i < TRAJECTORY_SIZE && line_x[i] < path_height; ++i) {
    max_idx = i;
  }
  return max_idx;
}

static void update_leads(UIState *s, const cereal::ModelDataV2::Reader &model) {
  auto leads = model.getLeadsV3();
  auto model_position = model.getPosition();
  bool lead_drawn = false;
  for (int i = 0; i < 2; ++i) {
    if (leads[i].getProb() > 0.5) {
      lead_drawn = true;
      float z = model_position.getZ()[get_path_length_idx(model_position, leads[i].getX()[0])];
      calib_frame_to_full_frame(s, leads[i].getX()[0], leads[i].getY()[0], z + 1.22, &s->scene.lead_vertices[i]);
    }
  }
  if (!lead_drawn){
    // draw VOACC leads
    for (int i = 0; i < 2; ++i) {
      if (s->scene.lead_data[i].getStatus() && s->scene.lead_data[i].getDRel() >= 120.) {
        int path_ind = get_path_length_idx(model_position, s->scene.lead_data[i].getDRel());
        float z = model_position.getZ()[path_ind];
        if (path_ind == TRAJECTORY_SIZE-1){
          for (int j = 0; j < 2; ++j){
            auto prob = model.getLaneLineProbs()[1+j];
            if (prob > voacc_lead_min_laneline_prob){
              int path_ind2 = get_path_length_idx(model.getLaneLines()[1+j], s->scene.lead_data[i].getDRel());
              if (path_ind2 < path_ind){
                path_ind = path_ind2;
                z = model.getLaneLines()[1+j].getZ()[path_ind];
              }
            }
          }
        }
        calib_frame_to_full_frame(s, s->scene.lead_data[i].getDRel(), -(s->scene.lead_data[i].getYRel()), z + 1.22, &s->scene.lead_vertices[i]);
      }
    }
  }
}

static void update_line_data(const UIState *s, const cereal::ModelDataV2::XYZTData::Reader &line,
                             float y_off, float z_off, line_vertices_data *pvd, int max_idx, bool allow_invert=true) {
  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
  std::vector<vertex_data> left_points, right_points;
  for (int i = 0; i <= max_idx; i++) {
    vertex_data left, right;
    bool l = calib_frame_to_full_frame(s, line_x[i], line_y[i] - y_off, line_z[i] + z_off, &left);
    bool r = calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_off, line_z[i] + z_off, &right);
    if (l && r) {
      // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
      if (!allow_invert && left_points.size() && left.y > left_points.back().y) {
        continue;
      }
      left_points.push_back(left);
      right_points.push_back(right);
    }
  }

  pvd->cnt = 2 * left_points.size();
  assert(left_points.size() == right_points.size());
  assert(pvd->cnt <= std::size(pvd->v));

  for (int left_idx = 0; left_idx < left_points.size(); left_idx++){
    int right_idx = 2 * left_points.size() - left_idx - 1;
    pvd->v[left_idx] = left_points[left_idx];
    pvd->v[right_idx] = right_points[left_idx];
  }
}

static void update_model(UIState *s, const cereal::ModelDataV2::Reader &model) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;
  auto model_position = model.getPosition();
  float max_distance = std::clamp(model_position.getX()[TRAJECTORY_SIZE - 1],
                                  MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

  // update lane lines
  const auto lane_lines = model.getLaneLines();
  const auto lane_line_probs = model.getLaneLineProbs();
  int max_idx = get_path_length_idx(lane_lines[0], max_distance);
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    scene.lane_line_probs[i] = lane_line_probs[i];
    update_line_data(s, lane_lines[i], 0.025 * scene.lane_line_probs[i], 0, &scene.lane_line_vertices[i], max_idx);
  }

  // update road edges
  const auto road_edges = model.getRoadEdges();
  const auto road_edge_stds = model.getRoadEdgeStds();
  for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
    scene.road_edge_stds[i] = road_edge_stds[i];
    update_line_data(s, road_edges[i], 0.025, 0, &scene.road_edge_vertices[i], max_idx);
  }
  
  scene.lateral_plan = sm["lateralPlan"].getLateralPlan();

  // update path
  auto lead_one = model.getLeadsV3()[0];
  if (lead_one.getProb() > 0.5) {
    const float lead_d = lead_one.getX()[0] * 2.;
    max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
  }
  max_idx = get_path_length_idx(model_position, max_distance);
  update_line_data(s, model_position, scene.end_to_end ? 0.8 : 0.5, 1.22, &scene.track_vertices, max_idx, false);
}

static void update_sockets(UIState *s) {
  s->sm->update(0);
}

static void update_state(UIState *s) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;
  float t = seconds_since_boot();


  scene.map_open = (s->fb_h != 0 && (float)s->fb_w / (float)s->fb_h < 1.5);
  
  if (t - scene.paramsCheckLast > scene.paramsCheckFreq){
    scene.paramsCheckLast = t;
    scene.disableDisengageOnGasEnabled = Params().getBool("DisableDisengageOnGas");
    scene.speed_limit_control_enabled = Params().getBool("SpeedLimitControl");
    scene.screen_dim_mode = std::stoi(Params().get("ScreenDimMode"));
    scene.lane_pos_enabled = Params().getBool("LanePositionEnabled");
    scene.lead_info_print_enabled = Params().getBool("PrintLeadInfo");
    scene.speed_limit_eu_style = int(Params().getBool("EUSpeedLimitStyle"));
    scene.show_debug_ui = Params().getBool("ShowDebugUI");
    scene.brake_indicator_enabled = Params().getBool("BrakeIndicator");
    if (scene.auto_lane_pos_active){
      scene.lane_pos = std::stoi(Params().get("LanePosition"));
    }
    if (scene.disableDisengageOnGasEnabled){
      scene.onePedalModeActive = Params().getBool("OnePedalMode");
      scene.onePedalEngageOnGasEnabled = Params().getBool("OnePedalModeEngageOnGas");
      scene.visionBrakingEnabled = Params().getBool("TurnVisionControl");
      scene.mapBrakingEnabled = Params().getBool("TurnSpeedControl");
    }
    if (scene.accel_mode_button_enabled){
      scene.accel_mode = std::stoi(Params().get("AccelMode"));
    }
    if (scene.dynamic_follow_mode_button_enabled){
      scene.dynamic_follow_active = std::stoi(Params().get("DynamicFollow"));
    }
    if (scene.ev_eff_total_dist < 10.){
      float oldDist = std::stof(Params().get("EVConsumptionTripDistance"));
      if (oldDist > scene.ev_eff_total_dist){
        scene.ev_eff_total_dist = oldDist;
        s->scene.ev_recip_eff_wa[1] = std::stof(Params().get("EVConsumption5Mi"));
        s->scene.ev_eff_total_dist = oldDist;
        s->scene.ev_eff_total_kWh = std::stof(Params().get("EVConsumptionTripkWh"));
      }
    }
  }

  if (scene.started){

    if (scene.ev_eff_total_dist > 10. && sm.frame % scene.ev_eff_params_write_freq == 0) {
      {
        char val_str[18];
        sprintf(val_str, "%.3f", scene.ev_recip_eff_wa[1]);
        Params().put("EVConsumption5Mi", val_str, strlen(val_str));
      }
      {
        char val_str[18];
        sprintf(val_str, "%.3f", scene.ev_eff_total_kWh);
        Params().put("EVConsumptionTripkWh", val_str, strlen(val_str));
      }
      {
        char val_str[18];
        sprintf(val_str, "%.3f", scene.ev_eff_total_dist);
        Params().put("EVConsumptionTripDistance", val_str, strlen(val_str));
      }
    }
    
    if (scene.lane_pos != 0 && !s->scene.auto_lane_pos_active && scene.lane_pos_dist_since_set > scene.lane_pos_timeout_dist){
      scene.lane_pos = 0;
      scene.lane_pos_timeout_dist = scene.lane_pos_dist_short;
      Params().put("LanePosition", "0", 1);
    }
  
    // fade screen brightness
    // update screen dim
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    const int radius = 96;
    const int center_x = maxspeed_rect.centerX();
    const int center_y = offset_button_y(s, s->fb_h - footer_h / 2, radius);
    scene.screen_dim_touch_rect = {center_x - (1+scene.screen_dim_mode_max-scene.screen_dim_mode) * radius, center_y - (1+scene.screen_dim_mode_max-scene.screen_dim_mode) * radius, (2*(1+scene.screen_dim_mode_max-scene.screen_dim_mode)) * radius, (2*(1+scene.screen_dim_mode_max-scene.screen_dim_mode)) * radius};
    
    if (s->status == STATUS_WARNING){
      scene.screen_dim_mode_cur = scene.screen_dim_mode + 1;
      if (scene.screen_dim_mode_cur > scene.screen_dim_mode_max){
        scene.screen_dim_mode_cur = scene.screen_dim_mode_max;
      }
    }
    else if (s->status == STATUS_ALERT){
      scene.screen_dim_mode_cur = scene.screen_dim_mode_max;
      scene.screen_dim_fade = scene.screen_dim_modes_v[scene.screen_dim_mode_cur];
    }
    else{
      scene.screen_dim_mode_cur = scene.screen_dim_mode;
    }
    
    if (scene.screen_dim_mode_cur != scene.screen_dim_mode_last){
      scene.screen_dim_fade_step = scene.screen_dim_modes_v[scene.screen_dim_mode_cur] - scene.screen_dim_modes_v[scene.screen_dim_mode_last];
      scene.screen_dim_fade_step /= (scene.screen_dim_fade_step > 0 ? scene.screen_dim_fade_dur_up : scene.screen_dim_fade_dur_down);
    }
    
    if (scene.screen_dim_fade > scene.screen_dim_modes_v[scene.screen_dim_mode_cur]){
      scene.screen_dim_fade += scene.screen_dim_fade_step * (t - scene.screen_dim_fade_last_t);
      if (scene.screen_dim_fade < scene.screen_dim_modes_v[scene.screen_dim_mode_cur])
        scene.screen_dim_fade = scene.screen_dim_modes_v[scene.screen_dim_mode_cur];
    }
    else if (scene.screen_dim_fade < scene.screen_dim_modes_v[scene.screen_dim_mode_cur]){
      scene.screen_dim_fade += scene.screen_dim_fade_step * (t - scene.screen_dim_fade_last_t);
      if (scene.screen_dim_fade > scene.screen_dim_modes_v[scene.screen_dim_mode_cur])
        scene.screen_dim_fade = scene.screen_dim_modes_v[scene.screen_dim_mode_cur];
    }
  }
  else{
    scene.screen_dim_mode_cur = scene.screen_dim_mode_max;
    scene.screen_dim_fade = scene.screen_dim_modes_v[scene.screen_dim_mode_cur];
    scene.screen_dim_touch_rect = {1,1,1,1};
  }
  scene.screen_dim_mode_last = scene.screen_dim_mode_cur;
  scene.screen_dim_fade_last_t = t;

  // update engageability and DM icons at 2Hz
  if (sm.frame % (UI_FREQ / 2) == 0) {
    scene.engageable = sm["controlsState"].getControlsState().getEngageable();
    scene.dm_active = sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode();
  }
  if (scene.started && sm.updated("controlsState")) {
    scene.controls_state = sm["controlsState"].getControlsState();
    scene.car_state = sm["carState"].getCarState();
    if (scene.is_using_torque_control){// if lateral torque controller in use, angle error is stored in its unused error_rate.
      scene.lateralCorrection = scene.controls_state.getLateralControlState().getTorqueState().getOutput();
      scene.angleSteersErr = scene.controls_state.getLateralControlState().getTorqueState().getErrorRate();
    }
    else{
      scene.lateralCorrection = scene.controls_state.getLateralControlState().getPidState().getOutput();
      scene.angleSteersErr = scene.controls_state.getLateralControlState().getPidState().getAngleError();
    }
    scene.angleSteersDes = scene.angleSteersErr + scene.car_state.getSteeringAngleDeg();
  }
  if (sm.updated("carState")){
    scene.car_state = sm["carState"].getCarState();
  
    scene.brake_percent = scene.car_state.getFrictionBrakePercent();
    
    scene.steerOverride= scene.car_state.getSteeringPressed();
    scene.angleSteers = scene.car_state.getSteeringAngleDeg();
    scene.engineRPM = static_cast<int>((scene.car_state.getEngineRPM() / (10.0)) + 0.5) * 10;

    // EV efficiency
    float cur_dist = std::abs(scene.car_state.getVEgo() * (t - scene.ev_eff_last_time));
    
    scene.ev_eff_total_dist += cur_dist;
    float cur_kW = -scene.car_state.getHvbWattage();
    float cur_kWh = cur_kW * (t - scene.ev_eff_last_time) * 2.8e-4; // [kJ converted to kWh]
    scene.ev_eff_total_kWh += cur_kWh;

    if (cur_dist > 1e-4){
      if (scene.ev_eff_stopped_kWh != 0.){
        cur_kWh += scene.ev_eff_stopped_kWh;
        scene.ev_eff_stopped_kWh = 0.;
      }
      float cur_recip_eff = cur_kWh * (scene.is_metric ? 1000. : 1609.) / cur_dist;
      for (int i = 0; i < 2; ++i){
        float tmp_cur_dist = (cur_dist > scene.ev_eff_distances[i] ? scene.ev_eff_distances[i] : cur_dist);
        scene.ev_recip_eff_wa[i] = tmp_cur_dist * scene.ev_eff_distances_recip[i] * cur_recip_eff 
                            + (1. - tmp_cur_dist * scene.ev_eff_distances_recip[i]) * scene.ev_recip_eff_wa[i];
      }
    }
    else{
      scene.ev_eff_stopped_kWh += cur_kWh;
    }
    if (scene.ev_eff_total_kWh != 0.){
        scene.ev_eff_total = scene.ev_eff_total_dist / (scene.is_metric ? 1000. : 1609.) / scene.ev_eff_total_kWh;
        if (std::abs(scene.ev_eff_total) > scene.ev_recip_eff_wa_max){
          scene.ev_eff_total = (scene.ev_eff_total > 0. ? scene.ev_recip_eff_wa_max : -scene.ev_recip_eff_wa_max);
        }
    }
    scene.ev_eff_last_time = t;

    // lane position
    if (scene.lane_pos != 0){
      scene.lane_pos_dist_since_set += scene.car_state.getVEgo() * (t - scene.lane_pos_dist_last_t);
      if (!s->scene.auto_lane_pos_active && abs(scene.car_state.getSteeringAngleDeg()) > scene.lane_pos_max_steer_deg){
        scene.lane_pos = 0;
        Params().put("LanePosition", "0", 1);
      }
    }
    scene.lane_pos_dist_last_t = t;
  }
  if (sm.updated("liveParameters")){
    scene.road_roll = sm["liveParameters"].getLiveParameters().getRoll();
  }
  if (sm.updated("radarState")) {
    auto radar_state = sm["radarState"].getRadarState();
    scene.lead_data[0] = radar_state.getLeadOne();
    scene.lead_data[1] = radar_state.getLeadTwo();
    scene.lead_v_rel = scene.lead_data[0].getVRel();
    scene.lead_d_rel = scene.lead_data[0].getDRel();
    scene.lead_v = scene.lead_data[0].getVLead();
    scene.lead_status = scene.lead_data[0].getStatus();
    if (!scene.lead_status){
      scene.lead_x_vals.clear();
      scene.lead_y_vals.clear();
    }
  }
  if (sm.updated("modelV2") && s->vg) {
    auto model = sm["modelV2"].getModelV2();
    update_model(s, model);
    update_leads(s, model);
  }
  if (sm.updated("liveCalibration")) {
    scene.world_objects_visible = true;
    auto rpy_list = sm["liveCalibration"].getLiveCalibration().getRpyCalib();
    Eigen::Vector3d rpy;
    rpy << rpy_list[0], rpy_list[1], rpy_list[2];
    Eigen::Matrix3d device_from_calib = euler2rot(rpy);
    Eigen::Matrix3d view_from_device;
    view_from_device << 0,1,0,
                        0,0,1,
                        1,0,0;
    Eigen::Matrix3d view_from_calib = view_from_device * device_from_calib;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        scene.view_from_calib.v[i*3 + j] = view_from_calib(i,j);
      }
    }
  }
  if (sm.updated("pandaState")) {
    auto pandaState = sm["pandaState"].getPandaState();
    scene.fanspeed_rpm = pandaState.getFanSpeedRpm();
    scene.pandaType = pandaState.getPandaType();
    scene.ignition = pandaState.getIgnitionLine() || pandaState.getIgnitionCan();
  } else if ((s->sm->frame - s->sm->rcv_frame("pandaState")) > 5*UI_FREQ) {
    scene.pandaType = cereal::PandaState::PandaType::UNKNOWN;
  }
  if (sm.updated("carParams")) {
    scene.longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
    scene.is_using_torque_control = (sm["carParams"].getCarParams().getLateralTuning().which() == cereal::CarParams::LateralTuning::TORQUE);
  }
  if (sm.updated("liveMapData")) {
    scene.current_road_name = sm["liveMapData"].getLiveMapData().getCurrentRoadName();
  }
  if (sm.updated("sensorEvents")) {
    for (auto sensor : sm["sensorEvents"].getSensorEvents()) {
      if (!scene.started && sensor.which() == cereal::SensorEventData::ACCELERATION) {
        auto accel = sensor.getAcceleration().getV();
        if (accel.totalSize().wordCount) { // TODO: sometimes empty lists are received. Figure out why
          scene.accel_sensor = accel[2];
        }
      } else if (!scene.started && sensor.which() == cereal::SensorEventData::GYRO_UNCALIBRATED) {
        auto gyro = sensor.getGyroUncalibrated().getV();
        if (gyro.totalSize().wordCount) {
          scene.gyro_sensor = gyro[1];
        }
      }
    }
  }
  if (sm.updated("roadCameraState")) {
    auto camera_state = sm["roadCameraState"].getRoadCameraState();

    float max_lines = Hardware::EON() ? 5408 : 1904;
    float max_gain = Hardware::EON() ? 1.0: 10.0;
    float max_ev = max_lines * max_gain;

    if (Hardware::TICI()) {
      max_ev /= 6;
    }

    float ev = camera_state.getGain() * float(camera_state.getIntegLines());

    scene.light_sensor = std::clamp<float>(1.0 - (ev / max_ev), 0.0, 1.0);
  }
  scene.started = sm["deviceState"].getDeviceState().getStarted() && scene.ignition;
  if (sm.updated("deviceState")) {
    scene.deviceState = sm["deviceState"].getDeviceState();
    scene.network_type_string = ui_network_type[(int)scene.deviceState.getNetworkType()];
    scene.network_strength = (int)scene.deviceState.getNetworkStrength();
  }
  if (sm.updated("liveLocationKalman")) {
    scene.gpsOK = sm["liveLocationKalman"].getLiveLocationKalman().getGpsOK();
    scene.device_roll = sm["liveLocationKalman"].getLiveLocationKalman().getCalibratedOrientationNED().getValue()[0];
  }
  if (sm.updated("lateralPlan")) {
    scene.lateral_plan = sm["lateralPlan"].getLateralPlan();
    auto data = sm["lateralPlan"].getLateralPlan();

    scene.lateralPlan.laneWidth = data.getLaneWidth();
    scene.lateralPlan.dProb = data.getDProb();
    scene.lateralPlan.lProb = data.getLProb();
    scene.lateralPlan.rProb = data.getRProb();
    scene.lateralPlan.lanelessModeStatus = data.getLanelessMode();
  }
  if (sm.updated("gpsLocationExternal")) {
    auto data = sm["gpsLocationExternal"].getGpsLocationExternal();
    scene.bearingAccuracy = data.getBearingAccuracyDeg();
    scene.bearingDeg = data.getBearingDeg();
  }
  if (sm.updated("longitudinalPlan")) {
    scene.longitudinal_plan = sm["longitudinalPlan"].getLongitudinalPlan();
    
    scene.desiredFollowDistance = scene.longitudinal_plan.getDesiredFollowDistance();
    scene.followDistanceCost = scene.longitudinal_plan.getLeadDistCost();
    scene.followAccelCost = scene.longitudinal_plan.getLeadAccelCost();
    scene.stoppingDistance = scene.longitudinal_plan.getStoppingDistance();
    scene.dynamic_follow_level = scene.longitudinal_plan.getDynamicFollowLevel();
  }
  
  if (scene.brake_percent > 50){
    if (scene.brake_indicator_alpha < 1.){
      scene.brake_indicator_alpha += fade_time_step * (t - scene.brake_indicator_last_t);
      if (scene.brake_indicator_alpha > 1.)
        scene.brake_indicator_alpha = 1.;
    }
  }
  else if (scene.brake_indicator_alpha > 0.){
    scene.brake_indicator_alpha -= fade_time_step * (t - scene.brake_indicator_last_t);
    if (scene.brake_indicator_alpha < 0.)
      scene.brake_indicator_alpha = 0.;
  }
  scene.brake_indicator_last_t = t;

  if (t - scene.sessionInitTime > 3.){
    if ((scene.car_state.getOnePedalModeActive() || scene.car_state.getCoastOnePedalModeActive())
      || (s->status == UIStatus::STATUS_DISENGAGED && scene.controls_state.getVCruise() <= 3 && (scene.onePedalModeActive || scene.disableDisengageOnGasEnabled))){
      scene.one_pedal_fade += fade_time_step * (t - scene.one_pedal_fade_last_t);
      if (scene.one_pedal_fade > 1.)
        scene.one_pedal_fade = 1.;
    }
    else if (scene.one_pedal_fade > -1.){
      scene.one_pedal_fade -= fade_time_step * (t - scene.one_pedal_fade_last_t);
      if (scene.one_pedal_fade < -1.)
        scene.one_pedal_fade = -1.;
    }
  }
  scene.one_pedal_fade_last_t = t;
  
  // dynamic follow
  if (scene.dynamic_follow_level != scene.dynamic_follow_level_ui){
    if (scene.dynamic_follow_level > scene.dynamic_follow_level_ui){
      scene.dynamic_follow_level_ui += dynamic_follow_fade_step * (t - scene.dynamic_follow_last_t);
      if (scene.dynamic_follow_level_ui > scene.dynamic_follow_level){
        scene.dynamic_follow_level_ui = scene.dynamic_follow_level;
      }
    }
    else{ // if (scene.dynamic_follow_level < scene.dynamic_follow_level_ui){
      scene.dynamic_follow_level_ui -= dynamic_follow_fade_step * (t - scene.dynamic_follow_last_t);
      if (scene.dynamic_follow_level_ui < scene.dynamic_follow_level){
        scene.dynamic_follow_level_ui = scene.dynamic_follow_level;
      }
    }
  }
  scene.dynamic_follow_last_t = t;
  
  scene.lastTime = t;
}

static void update_params(UIState *s) {
  const uint64_t frame = s->sm->frame;
  UIScene &scene = s->scene;
  if (frame % (5*UI_FREQ) == 0) {
    scene.is_metric = Params().getBool("IsMetric");
  }
}

static void update_vision(UIState *s) {
  if (!s->vipc_client->connected && s->scene.started) {
    if (s->vipc_client->connect(false)) {
      ui_init_vision(s);
    }
  }

  if (s->vipc_client->connected) {
    VisionBuf * buf = s->vipc_client->recv();
    if (buf != nullptr) {
      s->last_frame = buf;
    } else if (!Hardware::PC()) {
      LOGE("visionIPC receive timeout");
    }
  } else if (s->scene.started) {
    util::sleep_for(1000. / UI_FREQ);
  }
  if(s->awake)
  {
    int touch_x = -1, touch_y = -1;
    touch_poll(&(s->touch), &touch_x, &touch_y, 0);
    dashcam(s, touch_x, touch_y);
  }
}

static void update_status(UIState *s) {
  if (s->scene.started && s->sm->updated("controlsState")) {
    auto controls_state = (*s->sm)["controlsState"].getControlsState();
    auto alert_status = controls_state.getAlertStatus();
    if (alert_status == cereal::ControlsState::AlertStatus::USER_PROMPT) {
      s->status = STATUS_WARNING;
    } else if (alert_status == cereal::ControlsState::AlertStatus::CRITICAL) {
      s->status = STATUS_ALERT;
    } else {
      s->status = controls_state.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
    }
  }

  // Handle onroad/offroad transition
  static bool started_prev = false;
  if (s->scene.started != started_prev) {
    if (s->scene.started) {
      s->status = STATUS_DISENGAGED;
      s->scene.started_frame = s->sm->frame;

      if (Params().getBool("LowOverheadMode") && s->scene.screen_dim_mode_cur == s->scene.screen_dim_mode_max){
        s->scene.screen_dim_mode_cur -= 1;
        Params().put("ScreenDimMode", std::to_string(s->scene.screen_dim_mode_cur).c_str(), 1);
      }
      s->scene.end_to_end = Params().getBool("EndToEndToggle");
      s->scene.color_path = Params().getBool("ColorPath");
      if (!s->scene.end_to_end){
        s->scene.laneless_btn_touch_rect = {1,1,1,1};
      }
      s->scene.laneless_mode = std::stoi(Params().get("LanelessMode"));
      s->scene.brake_percent = std::stoi(Params().get("FrictionBrakePercent"));

      s->scene.accel_mode_button_enabled = Params().getBool("AccelModeButton");
      if (!s->scene.accel_mode_button_enabled){
        s->scene.accel_mode_touch_rect = {1,1,1,1};
      }
      s->scene.dynamic_follow_mode_button_enabled = Params().getBool("DynamicFollowToggle");
      if (!s->scene.dynamic_follow_mode_button_enabled){
        s->scene.dynamic_follow_mode_touch_rect = {1,1,1,1};
      }

      if (Params().getBool("EVConsumptionReset")) {
        char val_str[18];
        sprintf(val_str, "0.0");
        Params().put("EVConsumption5Mi", val_str, strlen(val_str));
        Params().put("EVConsumptionTripkWh", val_str, strlen(val_str));
        Params().put("EVConsumptionTripDistance", val_str, strlen(val_str));
        Params().putBool("EVConsumptionReset", false);
        s->scene.ev_recip_eff_wa[1] = 0.0;
        s->scene.ev_eff_total_dist = 0.0;
        s->scene.ev_eff_total_kWh = 0.0;
        s->scene.ev_eff_total = 0.0;
      }
      s->scene.ev_recip_eff_wa[0] = 0.0;

      s->scene.sessionInitTime = seconds_since_boot();
      s->scene.percentGrade = 0;
      for (int i = 0; i < 5; ++i){
        s->scene.percentGradeAltitudes[i] = 0.;
        s->scene.percentGradePositions[i] = 0.;
        s->scene.percentGrades[i] = 0.;
        s->scene.percentGradeIterRolled = false;
        s->scene.percentGradeRollingIter = 0;
      }
      s->scene.ev_eff_total_kWh = 0.;
      s->scene.ev_eff_total_dist = 0.;

      s->scene.measure_config_num = std::stoi(Params().get("MeasureConfigNum"));
      s->scene.measure_cur_num_slots = s->scene.measure_config_list[s->scene.measure_config_num];
      s->scene.measure_num_rows = s->scene.measure_cur_num_slots;
      if (s->scene.measure_num_rows > s->scene.measure_max_rows){
        s->scene.measure_num_rows /= 2;
      }
      s->scene.measure_row_offset = s->scene.measure_max_rows - s->scene.measure_num_rows;
      for (int i = 0; i < s->scene.measure_max_num_slots; ++i){
        char slotName[16];
        snprintf(slotName, sizeof(slotName), "MeasureSlot%.2d", i);
        s->scene.measure_slots[i] = std::stoi(Params().get(slotName));
      }

      s->wide_camera = Hardware::TICI() ? Params().getBool("EnableWideCamera") : false;

      // Update intrinsics matrix after possible wide camera toggle change
      if (s->vg) {
        ui_resize(s, s->fb_w, s->fb_h);
      }

      // Choose vision ipc client
      if (s->wide_camera) {
        s->vipc_client = s->vipc_client_wide;
      } else {
        s->vipc_client = s->vipc_client_rear;
      }

      s->scene.speed_limit_control_enabled = Params().getBool("SpeedLimitControl");
      s->scene.speed_limit_perc_offset = Params().getBool("SpeedLimitPercOffset");
      s->scene.show_debug_ui = Params().getBool("ShowDebugUI");
    } else {
      s->vipc_client->connected = false;
    }
  }
  started_prev = s->scene.started;
}

static void update_extras(UIState *s)
{
   UIScene &scene = s->scene;
   SubMaster &sm = *(s->sm);

   if(sm.updated("carControl"))
    scene.car_control = sm["carControl"].getCarControl();
}


QUIState::QUIState(QObject *parent) : QObject(parent) {
  ui_state.sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({
    "modelV2", "controlsState", "liveCalibration", "deviceState", "roadCameraState", "liveMapData",
    "pandaState", "carParams", "driverMonitoringState", "sensorEvents", "carState", "radarState", "liveLocationKalman", "ubloxGnss", "gpsLocationExternal", 
    "longitudinalPlan", "lateralPlan", "carControl", "liveParameters", "roadLimitSpeed",
  });

  ui_state.fb_w = vwp_w;
  ui_state.fb_h = vwp_h;
  ui_state.scene.started = false;
  ui_state.last_frame = nullptr;
  ui_state.wide_camera = Hardware::TICI() ? Params().getBool("EnableWideCamera") : false;

  ui_state.vipc_client_rear = new VisionIpcClient("camerad", VISION_STREAM_RGB_BACK, true);
  ui_state.vipc_client_wide = new VisionIpcClient("camerad", VISION_STREAM_RGB_WIDE, true);

  ui_state.vipc_client = ui_state.vipc_client_rear;

  // update timer
  timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &QUIState::update);
  timer->start(0);

  touch_init(&(ui_state.touch));
}

void QUIState::update() {
  update_params(&ui_state);
  update_sockets(&ui_state);
  update_state(&ui_state);
  update_status(&ui_state);
  update_vision(&ui_state);
  update_extras(&ui_state);

  if (ui_state.scene.started != started_prev || ui_state.sm->frame == 1) {
    started_prev = ui_state.scene.started;
    emit offroadTransition(!ui_state.scene.started);

    // Change timeout to 0 when onroad, this will call update continuously.
    // This puts visionIPC in charge of update frequency, reducing video latency
    timer->start(ui_state.scene.started ? 0 : 1000 / UI_FREQ);
  }

  watchdog_kick();
  emit uiUpdate(ui_state);
}

Device::Device(QObject *parent) : brightness_filter(BACKLIGHT_OFFROAD, BACKLIGHT_TS, BACKLIGHT_DT), QObject(parent) {
}

void Device::update(const UIState &s) {
  updateBrightness(s);
  updateWakefulness(s);

  // TODO: remove from UIState and use signals
  QUIState::ui_state.awake = awake;
}

void Device::setAwake(bool on, bool reset) {
  if (on != awake) {
    awake = on;
    Hardware::set_display_power(awake);
    LOGD("setting display power %d", awake);
    emit displayPowerChanged(awake);
  }

  if (reset) {
    awake_timeout = 30 * UI_FREQ;
  }
}

void Device::updateBrightness(const UIState &s) {
  // Scale to 0% to 100%
  float clipped_brightness = 100.0 * s.scene.light_sensor;

  // CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
  if (clipped_brightness <= 8) {
    clipped_brightness = (clipped_brightness / 903.3);
  } else {
    clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
  }

  // Scale back to 10% to 100%
  clipped_brightness = std::clamp(100.0f * clipped_brightness, 10.0f, 100.0f);

  if (!s.scene.started) {
    clipped_brightness = BACKLIGHT_OFFROAD;
  }

  int brightness = brightness_filter.update(clipped_brightness);
  if (!awake) {
    brightness = 0;
  }
  else if (s.scene.started && QUIState::ui_state.scene.screen_dim_fade < 1.0){
    brightness = std::clamp(int(float(brightness) * QUIState::ui_state.scene.screen_dim_fade),1,100);
  }

  if (brightness != last_brightness) {
    std::thread{Hardware::set_brightness, brightness}.detach();
  }
  last_brightness = brightness;
}

void Device::updateWakefulness(const UIState &s) {
  awake_timeout = std::max(awake_timeout - 1, 0);

  bool should_wake = s.scene.started || s.scene.ignition;
  if (!should_wake) {
    // tap detection while display is off
    bool accel_trigger = abs(s.scene.accel_sensor - accel_prev) > 0.2;
    bool gyro_trigger = abs(s.scene.gyro_sensor - gyro_prev) > 0.15;
    should_wake = accel_trigger && gyro_trigger;
    gyro_prev = s.scene.gyro_sensor;
    accel_prev = (accel_prev * (accel_samples - 1) + s.scene.accel_sensor) / accel_samples;
  }

  setAwake(awake_timeout, should_wake);
}

int offset_button_y(UIState *s, int center_y, int radius){
  if ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::SMALL){
    center_y = 2 * center_y / 3 + radius / 2;
  }
  else if ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::MID){
    center_y = (center_y + radius) / 2;
  }
  return center_y;
}

int offset_right_side_button_x(UIState *s, int center_x, int radius, bool doShift){
  if ((doShift || (*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::SMALL)
  && s->scene.measure_cur_num_slots > 0 && !s->scene.map_open){
    int off = s->scene.measure_slots_rect.right() - center_x;
    center_x = s->scene.measure_slots_rect.x - off - bdr_s;
  }
  return center_x;
}
