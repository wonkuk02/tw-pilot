#include "selfdrive/ui/paint.h"

#include <time.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <algorithm>
#include <cassert>
#include <string>
#include <cmath>
#include <deque>

#include <QDateTime>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#define NANOVG_GLES3_IMPLEMENTATION
#include <nanovg_gl.h>
#include <nanovg_gl_utils.h>

#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"

#include "selfdrive/ui/ui.h"

static void ui_draw_text(const UIState *s, float x, float y, const char *string, float size, NVGcolor color, const char *font_name) {
  nvgFontFace(s->vg, font_name);
  nvgFontSize(s->vg, size);
  nvgFillColor(s->vg, color);
  nvgText(s->vg, x, y, string, NULL);
}

static void ui_draw_circle(UIState *s, float x, float y, float size, NVGcolor color) {
  nvgBeginPath(s->vg);
  nvgCircle(s->vg, x, y, size);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
}

static void ui_draw_speed_sign(UIState *s, float x, float y, int size, float speed, const char *subtext, 
                               float subtext_size, const char *font_name, bool is_map_sourced, bool is_active) {
  std::string const speedlimit_str = std::to_string((int)std::nearbyint(speed));
  float one_pedal_fade = MAX(0.5,-s->scene.one_pedal_fade);
  if (s->scene.speed_limit_eu_style){ // eu style
    NVGcolor ring_color = is_active ? COLOR_RED_ALPHA(int(one_pedal_fade * 255.)) : COLOR_RED_ALPHA(int(.2 * 255.));
    NVGcolor inner_color = is_active ? COLOR_WHITE_ALPHA(int(one_pedal_fade * 255.)) : COLOR_WHITE_ALPHA(int(.5 * 255.));
    NVGcolor text_color = is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.3 * 255.));

    ui_draw_circle(s, x, y, float(size), ring_color);
    ui_draw_circle(s, x, y, float(size) * 0.8, inner_color);

    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

    ui_draw_text(s, x, y, speedlimit_str.c_str(), 120, text_color, font_name);
    ui_draw_text(s, x, y + 55, subtext, subtext_size, text_color, font_name);

    if (is_map_sourced) {
      const int img_size = 35;
      const int img_y = int(y - 55);
      ui_draw_image(s, {int(x - (img_size / 2)), img_y - (img_size / 2), img_size, img_size}, "map_source_icon", 
                    is_active ? 1. : .3);
    }
    s->scene.speed_limit_sign_touch_rect = Rect{int(x) - speed_sgn_touch_pad, 
                                            int(y) - speed_sgn_touch_pad,
                                            2 * (speed_sgn_touch_pad + size), 
                                            2 * (speed_sgn_touch_pad + size)};
  }
  else{ // us/canada style
    const int border_width = 6;
    const int sign_width = 164;
    const int sign_height = 216;
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};

    // White outer square
    Rect sign_rect_outer = {maxspeed_rect.x + 10, int(y-size), sign_width, sign_height};
    ui_fill_rect(s->vg, sign_rect_outer, is_active ? COLOR_WHITE_ALPHA(int(one_pedal_fade * 255.)) : COLOR_WHITE_ALPHA(int(.5 * 255.)), 24);

    // Smaller black border
    Rect sign_rect = {int(sign_rect_outer.x + 1.5 * border_width), int(sign_rect_outer.y + 1.5 * border_width), int(sign_width - 3 * border_width), int(sign_height - 3 * border_width)};
    ui_draw_rect(s->vg, sign_rect, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), border_width, 16);

    

    // Speed limit value

    if (subtext_size > 0. && is_active && s->scene.one_pedal_fade <= 0.){
      // "SPEED"
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 7, "SPEED", 19 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-semibold");

      // "LIMIT"
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 45, "LIMIT", 19 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-semibold");
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 76, speedlimit_str.c_str(), 37 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-bold");
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 150, subtext, 20 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 200.)), "sans-bold");
    }
    else{
      // "SPEED"
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 15, "SPEED", 19 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-semibold");

      // "LIMIT"
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 54, "LIMIT", 19 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-semibold");
      ui_draw_text(s, sign_rect.centerX(), sign_rect.y + 83, speedlimit_str.c_str(), 48 * 2.5, is_active ? COLOR_BLACK_ALPHA(int(one_pedal_fade * 255.)) : COLOR_BLACK_ALPHA(int(.5 * 255.)), "sans-bold");
    }


    s->scene.speed_limit_sign_touch_rect = sign_rect_outer;
  }
}

const float OneOverSqrt3 = 1.0 / sqrt(3.0);
static void ui_draw_turn_speed_sign(UIState *s, float x, float y, int width, float speed, int curv_sign, 
                                    const char *subtext, const char *font_name, bool is_active) {
  const float stroke_w = 15.0;
  NVGcolor border_color = is_active ? COLOR_RED : COLOR_BLACK_ALPHA(.2f * 255);
  NVGcolor inner_color = is_active ? COLOR_WHITE : COLOR_WHITE_ALPHA(.35f * 255);
  NVGcolor text_color = is_active ? COLOR_BLACK : COLOR_BLACK_ALPHA(.3f * 255);

  const float cS = stroke_w * 0.5 + 4.5;  // half width of the stroke on the corners of the triangle
  const float R = width * 0.5 - stroke_w * 0.5;
  const float A = 0.73205;
  const float h2 = 2.0 * R / (1.0 + A);
  const float h1 = A * h2;
  const float L = 4.0 * R * OneOverSqrt3;

  // Draw the internal triangle, compensate for stroke width. Needed to improve rendering when in inactive 
  // state due to stroke transparency being different from inner transparency.
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x, y - R + cS);
  nvgLineTo(s->vg, x - L * 0.5 + cS, y + h1 + h2 - R - stroke_w * 0.5);
  nvgLineTo(s->vg, x + L * 0.5 - cS, y + h1 + h2 - R - stroke_w * 0.5);
  nvgClosePath(s->vg);

  nvgFillColor(s->vg, inner_color);
  nvgFill(s->vg);
  
  // Draw the stroke
  nvgLineJoin(s->vg, NVG_ROUND);
  nvgStrokeWidth(s->vg, stroke_w);
  nvgStrokeColor(s->vg, border_color);

  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x, y - R);
  nvgLineTo(s->vg, x - L * 0.5, y + h1 + h2 - R);
  nvgLineTo(s->vg, x + L * 0.5, y + h1 + h2 - R);
  nvgClosePath(s->vg);

  nvgStroke(s->vg);

  // Draw the turn sign
  if (curv_sign != 0) {
    const int img_size = 35;
    const int img_y = int(y - R + stroke_w + 30);
    ui_draw_image(s, {int(x - (img_size / 2)), img_y, img_size, img_size}, 
                  curv_sign > 0 ? "turn_left_icon" : "turn_right_icon", is_active ? 1. : .3);
  }

  // Draw the texts.
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  const std::string speedlimit_str = std::to_string((int)std::nearbyint(speed));
  ui_draw_text(s, x, y + 25, speedlimit_str.c_str(), 90., text_color, font_name);
  ui_draw_text(s, x, y + 65, subtext, 30., text_color, font_name);
}

static void draw_chevron(UIState *s, float x, float y, float sz, NVGcolor fillColor, NVGcolor glowColor) {
  // glow
  float g_xo = sz * 0.2;
  float g_yo = sz * 0.1;
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
  nvgLineTo(s->vg, x, y-g_xo);
  nvgLineTo(s->vg, x-(sz*1.35)-g_xo, y+sz+g_yo);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, glowColor);
  nvgFill(s->vg);

  // chevron
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.25), y+sz);
  nvgLineTo(s->vg, x, y);
  nvgLineTo(s->vg, x-(sz*1.25), y+sz);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, fillColor);
  nvgFill(s->vg);
}

static void ui_draw_circle_image(const UIState *s, int center_x, int center_y, int radius, const char *image, NVGcolor color, float img_alpha) {
  nvgBeginPath(s->vg);
  nvgCircle(s->vg, center_x, center_y, radius);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
  const int img_size = radius * 1.5;
  ui_draw_image(s, {center_x - (img_size / 2), center_y - (img_size / 2), img_size, img_size}, image, img_alpha);
}

static void ui_draw_circle_image(const UIState *s, int center_x, int center_y, int radius, const char *image, bool active) {
  float bg_alpha = active ? 0.3f : 0.1f;
  float img_alpha = active ? 1.0f : 0.15f;
  ui_draw_circle_image(s, center_x, center_y, radius, image, nvgRGBA(0, 0, 0, (255 * bg_alpha)), img_alpha);
}


static void draw_lead(UIState *s, float d_rel, float v_rel, const vertex_data &vd, bool draw_info, bool is_voacc) {
  // Draw lead car indicator
  auto [x, y] = vd;

  float fillAlpha = 0;
  float speedBuff = 10.;
  float leadBuff = 40.;
  if (d_rel < leadBuff) {
    fillAlpha = 255*(1.0-(d_rel/leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255*(-1*(v_rel/speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel * 0.33333f + 30), 15.0f, 30.0f) * 2.35;
  x = std::clamp(x, 0.f, s->fb_w - sz * 0.5f);
  y = std::fmin(s->fb_h - sz * .6, y);
  draw_chevron(s, x, y, sz, nvgRGBA(201, 34, 49, fillAlpha), is_voacc ? COLOR_BLUE : COLOR_YELLOW);

  if (s->scene.lead_info_print_enabled && !s->scene.map_open && draw_info){
    // print lead info around chevron
    // Print relative distances to the left of the chevron
    int const x_offset = 100;
    int const y_offset = 48;
    int const y_max = s->fb_h - 4*bdr_s;
    s->scene.lead_x_vals.push_back(x);
    s->scene.lead_y_vals.push_back(y);
    while (s->scene.lead_x_vals.size() > s->scene.lead_xy_num_vals){
      s->scene.lead_x_vals.pop_front();
      s->scene.lead_y_vals.pop_front();
    }
    s->scene.lead_x_vals.shrink_to_fit();
    s->scene.lead_y_vals.shrink_to_fit();
    int lead_x = 0, lead_y = 0;
    for (int const & v : s->scene.lead_x_vals){
      lead_x += v;
    }
    lead_x /= float(s->scene.lead_x_vals.size());
    for (int const & v : s->scene.lead_y_vals){
      lead_y += v > y_max ? y_max : v;
    }
    lead_y /= float(s->scene.lead_y_vals.size());
    nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 180));
    nvgFontFace(s->vg, "sans-semibold");
    nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_MIDDLE);
    nvgBeginPath(s->vg);
    nvgFontSize(s->vg, 120);
    char val[16], unit[8];

    // first time distance
    if (s->scene.car_state.getVEgo() > 0.5){
      snprintf(unit, sizeof(unit), "s"); 
      float follow_t = d_rel / s->scene.car_state.getVEgo();
      snprintf(val, sizeof(val), "%.1f%s", follow_t, unit);
    }
    else{
      snprintf(val, sizeof(val), "-");
    }
    nvgText(s->vg,lead_x-x_offset,lead_y-y_offset,val,NULL);

    // then length distance
    snprintf(unit, sizeof(unit), "m"); 
    if (s->scene.lead_d_rel < 10.){
      snprintf(val, sizeof(val), "%.1f%s", s->scene.lead_d_rel, unit);
    }
    else{
      snprintf(val, sizeof(val), "%.0f%s", s->scene.lead_d_rel, unit);
    }
    nvgText(s->vg,lead_x-x_offset,lead_y+y_offset,val,NULL);

    // now abs and relative speed to the right

    nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);
    // first abs speed

    snprintf(unit, sizeof(unit), "kph"); 
    float v = (s->scene.lead_v * 3.6);
    if (v < 100.){
      snprintf(val, sizeof(val), "%.1f", v);
    }
    else{
      snprintf(val, sizeof(val), "%.0f", v);
    }

    nvgText(s->vg,lead_x+x_offset,lead_y-(y_offset*1.3),val,NULL);

    // then relative speed

    snprintf(val, sizeof(val), "%s%.1f", s->scene.lead_v_rel >= 0. ? "+" : "", (s->scene.lead_v_rel * 3.6));
    nvgText(s->vg,lead_x+x_offset,lead_y+(y_offset*1.4),val,NULL);

    nvgFontSize(s->vg, 70);
    nvgText(s->vg,lead_x+x_offset+20,lead_y,unit,NULL);
  }
}

static void ui_draw_line(UIState *s, const line_vertices_data &vd, NVGcolor *color, NVGpaint *paint) {
  if (vd.cnt == 0) return;

  const vertex_data *v = &vd.v[0];
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, v[0].x, v[0].y);
  for (int i = 1; i < vd.cnt; i++) {
    nvgLineTo(s->vg, v[i].x, v[i].y);
  }
  nvgClosePath(s->vg);
  if (color) {
    nvgFillColor(s->vg, *color);
  } else if (paint) {
    nvgFillPaint(s->vg, *paint);
  }
  nvgFill(s->vg);
}

static void draw_vision_frame(UIState *s) {
  glBindVertexArray(s->frame_vao);
  mat4 *out_mat = &s->rear_frame_mat;
  glActiveTexture(GL_TEXTURE0);

  if (s->last_frame) {
    glBindTexture(GL_TEXTURE_2D, s->texture[s->last_frame->idx]->frame_tex);
    if (!Hardware::EON()) {
      // this is handled in ion on QCOM
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, s->last_frame->width, s->last_frame->height,
                   0, GL_RGB, GL_UNSIGNED_BYTE, s->last_frame->addr);
    }
  }

  glUseProgram(s->gl_shader->prog);
  glUniform1i(s->gl_shader->getUniformLocation("uTexture"), 0);
  glUniformMatrix4fv(s->gl_shader->getUniformLocation("uTransform"), 1, GL_TRUE, out_mat->v);

  assert(glGetError() == GL_NO_ERROR);
  glEnableVertexAttribArray(0);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, (const void *)0);
  glDisableVertexAttribArray(0);
  glBindVertexArray(0);
}

// sunnyhaibin's colored lane line
static void ui_draw_vision_lane_lines(UIState *s) {
  const UIScene &scene = s->scene;
  NVGpaint track_bg;
  int steerOverride = scene.car_state.getSteeringPressed();
  //if (!scene.end_to_end) {
  if (!scene.lateralPlan.lanelessModeStatus) {
    // paint lanelines
    for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
      NVGcolor color = interp_alert_color(1.f - scene.lane_line_probs[i], 255);
      ui_draw_line(s, scene.lane_line_vertices[i], &color, nullptr);
    }
    // paint road edges
    for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
      NVGcolor color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0));
      ui_draw_line(s, scene.road_edge_vertices[i], &color, nullptr);
    }
  }
  if (scene.controls_state.getEnabled()) {
    if (steerOverride) {
      track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
        COLOR_BLACK_ALPHA(80), COLOR_BLACK_ALPHA(20));
    } 
    else if (!scene.lateralPlan.lanelessModeStatus) {
      if (scene.car_state.getLkMode()){
        if (scene.color_path){
          track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
            interp_alert_color(fabs(scene.lateralCorrection), 140), 
            interp_alert_color(fabs(scene.lateralCorrection), 0));
        }
        else{
          track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
            interp_alert_color(0., 140), 
            interp_alert_color(0., 0));
        }
      }
      else{
        track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE_ALPHA(130), COLOR_WHITE_ALPHA(0));
      }
    } 
    else { // differentiate laneless mode color (Grace blue)
      if (scene.car_state.getLkMode()){
        if (scene.color_path){
          int g, r = 255. * fabs(scene.lateralCorrection);
          r = CLIP(r, 0, 255);
          g = 100 + r;
          g = CLIP(g, 0, 255);
          track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                      nvgRGBA(r, g, 255, 150), 
                                      nvgRGBA(r, g, 255, 0));
        }
        else{
          track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                    COLOR_GRACE_BLUE_ALPHA(150), 
                                    COLOR_GRACE_BLUE_ALPHA(0));
        }
      }
      else{
        track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE_ALPHA(130), COLOR_WHITE_ALPHA(0));
      }
    }
  } else {
    // Draw white vision track
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                                          COLOR_WHITE_ALPHA(130), COLOR_WHITE_ALPHA(0));
  }
  // paint path
  ui_draw_line(s, scene.track_vertices, nullptr, &track_bg);

  // print lane and shoulder widths and probabilities
  if (s->scene.show_debug_ui && !s->scene.map_open){
    auto l_probs = s->scene.lateral_plan.getLaneProbs();
    auto road_edge_probs = s->scene.lateral_plan.getRoadEdgeProbs();
    if (l_probs.size() == 4 && road_edge_probs.size() == 2){
      const int width_font_size = 25;
      char cstr[16];
      int y = s->fb_h - 18;
      nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

      // current lane
      snprintf(cstr, sizeof(cstr), "(%.1f) %.1f (%.1f)", l_probs[1], (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getLaneWidth(), l_probs[2]);
      ui_draw_text(s, s->fb_w / 2, y, cstr, width_font_size * 2.5, COLOR_WHITE, "sans-semibold");
      // left adjacent lane
      if (1||s->scene.lateral_plan.getLaneWidthMeanLeftAdjacent() > 0.){
        snprintf(cstr, sizeof(cstr), "(%.1f) %.1f", l_probs[0], (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getLaneWidthMeanLeftAdjacent());
        ui_draw_text(s, s->fb_w / 5, y, cstr, width_font_size * 2.5, COLOR_WHITE, "sans-semibold");
      }
      // right adjacent lane
      if (1||s->scene.lateral_plan.getLaneWidthMeanRightAdjacent() > 0.){
        snprintf(cstr, sizeof(cstr), "%.1f (%.1f)", (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getLaneWidthMeanRightAdjacent(), l_probs[3]);
        ui_draw_text(s, 4 * s->fb_w / 5, y, cstr, width_font_size * 2.5, COLOR_WHITE, "sans-semibold");
      }
      // left shoulder
      nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_BASELINE);
      if (1||s->scene.lateral_plan.getShoulderMeanWidthLeft() > 0.){
        snprintf(cstr, sizeof(cstr), "(%.1f) %.1f", road_edge_probs[0], (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getShoulderMeanWidthLeft());
        ui_draw_text(s, 50, y, cstr, width_font_size * 2.5, COLOR_RED, "sans-bold");
      }    
      // right shoulder
      nvgTextAlign(s->vg, NVG_ALIGN_RIGHT | NVG_ALIGN_BASELINE);
      if (1||s->scene.lateral_plan.getShoulderMeanWidthRight() > 0.){
        snprintf(cstr, sizeof(cstr), "%.1f (%.1f)", (!s->scene.is_metric ? 3.28084 : 1.0) * s->scene.lateral_plan.getShoulderMeanWidthRight(), road_edge_probs[1]);
        ui_draw_text(s, s->fb_w - 50, y, cstr, width_font_size * 2.5, COLOR_RED, "sans-bold");
      }
    }
  }
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);

  // Draw lane edges and vision/mpc tracks
  ui_draw_vision_lane_lines(s);

  // Draw lead indicators if openpilot is handling longitudinal
  if (s->scene.longitudinal_control) {
    auto lead_one = (*s->sm)["modelV2"].getModelV2().getLeadsV3()[0];
    auto lead_two = (*s->sm)["modelV2"].getModelV2().getLeadsV3()[1];
    bool lead_drawn = false;
    if (lead_one.getProb() > .5) {
      lead_drawn = true;
      draw_lead(s, lead_one.getX()[0], lead_one.getV()[0], s->scene.lead_vertices[0], true, false);
    }
    if (lead_two.getProb() > .5 && (std::abs(lead_one.getX()[0] - lead_two.getX()[0]) > 3.0)) {
      lead_drawn = true;
      draw_lead(s, lead_two.getX()[0], lead_two.getV()[0], s->scene.lead_vertices[1], lead_one.getProb() <= .5, false);
    }
    for (int i = 0; i < 2 && !lead_drawn; ++i){
      if (s->scene.lead_data[i].getStatus()){
        lead_drawn = true;
        draw_lead(s, s->scene.lead_data[i].getDRel(), s->scene.lead_data[i].getVRel(), s->scene.lead_vertices[i], true, true);
      }
    }
  }
  nvgResetScissor(s->vg);
}

static void ui_draw_vision_maxspeed(UIState *s) {
  const int SET_SPEED_NA = 255;
  float maxspeed = (*s->sm)["controlsState"].getControlsState().getVCruise();
  const Rect rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  if (s->scene.one_pedal_fade > 0.){
    NVGcolor nvg_color;
    if(s->status == UIStatus::STATUS_DISENGAGED){
          const QColor &color = bg_colors[UIStatus::STATUS_DISENGAGED];
          nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), int(s->scene.one_pedal_fade * float(color.alpha())));
        }
    else if (s->scene.car_state.getOnePedalModeActive()){
      const QColor &color = bg_colors[s->scene.car_state.getOnePedalBrakeMode() + 1];
      nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), int(s->scene.one_pedal_fade * float(color.alpha())));
    }
    else {
      nvg_color = nvgRGBA(0, 0, 0, int(s->scene.one_pedal_fade * 100.));
    }
    const Rect pedal_rect = {rect.centerX() - brake_size, rect.centerY() - brake_size, brake_size * 2, brake_size * 2};
    ui_fill_rect(s->vg, pedal_rect, nvg_color, brake_size);
    ui_draw_image(s, {rect.centerX() - brake_size, rect.centerY() - brake_size, brake_size * 2, brake_size * 2}, "one_pedal_mode", s->scene.one_pedal_fade);
    s->scene.one_pedal_touch_rect = pedal_rect;
    s->scene.maxspeed_touch_rect = {1,1,1,1};
    
    // draw extra circle to indiate one-pedal engage on gas is enabled
    if (s->scene.onePedalEngageOnGasEnabled){
      nvgBeginPath(s->vg);
      const int r = int(float(brake_size) * 1.15);
      nvgRoundedRect(s->vg, rect.centerX() - r, rect.centerY() - r, 2 * r, 2 * r, r);
      nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(int(s->scene.one_pedal_fade * 255.)));
      nvgFillColor(s->vg, nvgRGBA(0,0,0,0));
      nvgFill(s->vg);
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
    }
  }
  else{
    s->scene.one_pedal_touch_rect = {1,1,1,1};
    s->scene.maxspeed_touch_rect = rect;
    const bool is_cruise_set = maxspeed != 0 && maxspeed != SET_SPEED_NA;
    if (is_cruise_set && !s->scene.is_metric) { maxspeed *= 0.6225; }

    ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(int(-s->scene.one_pedal_fade * 100.)), 30.);
    ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(int(-s->scene.one_pedal_fade * 100.)), 6, 20.);

    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
    NVGcolor max_color;
    if (is_cruise_set){
      max_color = nvgRGBA(0x80, 0xd8, 0xa6, int(-s->scene.one_pedal_fade * 255.));
    }
    else{
      max_color = nvgRGBA(0xa6, 0xa6, 0xa6, int(-s->scene.one_pedal_fade * 255.));
    }
    ui_draw_text(s, rect.centerX(), 118, "MAX", 26 * 2.5, max_color, is_cruise_set ? "sans-semibold" : "sans-regular");
    if (is_cruise_set) {
      std::string maxspeed_str = std::to_string((int)std::nearbyint(maxspeed));
      float font_size = 48 * 2.5;
      if (s->scene.car_state.getCoastingActive()){
        maxspeed_str += "+";
        font_size *= 0.9;
      }
      ui_draw_text(s, rect.centerX(), 212, maxspeed_str.c_str(), font_size, COLOR_WHITE_ALPHA(is_cruise_set ? int(-s->scene.one_pedal_fade * 200.) : int(-s->scene.one_pedal_fade * 100.)), "sans-bold");
    } else {
      ui_draw_text(s, rect.centerX(), 212, "N/A", 42 * 2.5, COLOR_WHITE_ALPHA(int(-s->scene.one_pedal_fade * 100.)), "sans-semibold");
    }
  }
}

static void ui_draw_vision_speedlimit(UIState *s) {
  const float speedLimit = s->scene.longitudinal_plan.getSpeedLimit();
  const float speedLimitOffset = s->scene.longitudinal_plan.getSpeedLimitOffset();

  if (speedLimit > 0.0 && s->scene.engageable) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    Rect speed_sign_rect = Rect{maxspeed_rect.centerX() - speed_sgn_r, 
      maxspeed_rect.bottom() + bdr_s, 
      2 * speed_sgn_r, 
      2 * speed_sgn_r};
    const float speed = speedLimit * (s->scene.is_metric ? 3.6 : 2.2369362921);
    const float speed_offset = speedLimitOffset * (s->scene.is_metric ? 3.6 : 2.2369362921);

    auto speedLimitControlState = s->scene.longitudinal_plan.getSpeedLimitControlState();
    const bool force_active = s->scene.speed_limit_control_enabled && 
                              seconds_since_boot() < s->scene.last_speed_limit_sign_tap + 2.0;
    const bool inactive = !force_active && (!s->scene.speed_limit_control_enabled || 
                          speedLimitControlState == cereal::LongitudinalPlan::SpeedLimitControlState::INACTIVE);
    const bool temp_inactive = !force_active && (s->scene.speed_limit_control_enabled && 
                               speedLimitControlState == cereal::LongitudinalPlan::SpeedLimitControlState::TEMP_INACTIVE);

    const int distToSpeedLimit = int(s->scene.longitudinal_plan.getDistToSpeedLimit() * 
                                     (s->scene.is_metric ? 1.0 : 3.28084) / 10) * 10;
    const bool is_map_sourced = s->scene.longitudinal_plan.getIsMapSpeedLimit();
    const std::string distance_str = std::to_string(distToSpeedLimit) + (s->scene.is_metric ? "m" : "f");
    const std::string offset_str = speed_offset > 0.0 ? "+" + std::to_string((int)std::nearbyint(speed_offset)) : "";
    const std::string inactive_str = temp_inactive ? "TEMP" : "";
    const std::string substring = inactive || temp_inactive ? inactive_str : 
                                                              distToSpeedLimit > 0 ? distance_str : offset_str;
    const float substring_size = inactive || temp_inactive || distToSpeedLimit > 0 ? 30.0 : 50.0;

    ui_draw_speed_sign(s, speed_sign_rect.centerX(), speed_sign_rect.centerY(), speed_sgn_r, speed, substring.c_str(), 
                       substring_size, "sans-bold", is_map_sourced, !inactive && !temp_inactive);

  }
}


NVGcolor color_from_thermal_status(int thermalStatus){
  switch (thermalStatus){
    case 0: return nvgRGBA(0, 255, 0, 200);
    case 1: return nvgRGBA(255, 128, 0, 200);
    default: return nvgRGBA(255, 0, 0, 200);
  }
}

static void ui_draw_measures(UIState *s){
  if (s->scene.measure_cur_num_slots){
    SubMaster &sm = *(s->sm);
    UIScene &scene = s->scene;
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    int center_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    const int brake_y = s->fb_h - footer_h / 2;
    const int y_min = maxspeed_rect.bottom() + bdr_s / 2;
    const int y_max = brake_y - brake_size - bdr_s / 2;
    const int y_rng = y_max - y_min;
    int slot_y_rng;
    if (scene.measure_num_rows > 4 || scene.map_open){
      slot_y_rng = y_rng / scene.measure_max_rows;
    }
    else{
      slot_y_rng = y_rng / (scene.measure_num_rows < 3 ? 3 : scene.measure_num_rows);
    }
    const int slot_y_rng_orig = y_rng / scene.measure_max_rows; // two columns
    const float slot_aspect_ratio_ratio = float(slot_y_rng) / float(slot_y_rng_orig);
    const int y_mid = (y_max + y_min) / 2;
    const int slots_y_rng = slot_y_rng * (scene.measure_num_rows <= scene.measure_max_rows ? scene.measure_num_rows : scene.measure_max_rows);
    const int slots_y_min = y_mid - (slots_y_rng / 2);
  
    NVGcolor default_name_color = nvgRGBA(255, 255, 255, 200);
    NVGcolor default_unit_color = nvgRGBA(255, 255, 255, 200);
    NVGcolor default_val_color = nvgRGBA(255, 255, 255, 200);
    int default_val_font_size = 78. * slot_aspect_ratio_ratio;
    int default_name_font_size = 32. * (slot_y_rng_orig > 1. ? 0.9 * slot_aspect_ratio_ratio : 1.);
    int default_unit_font_size = 38. * slot_aspect_ratio_ratio;
  
    // determine bounding rectangle
    int slots_r, slots_w, slots_x;

    const int slots_r_orig = brake_size + 6 + (s->scene.measure_cur_num_slots <= 5 ? 6 : 0);
    slots_r = brake_size * slot_aspect_ratio_ratio + 6 + (scene.measure_cur_num_slots <= scene.measure_max_rows ? 6 : 0);
    center_x -= slots_r - slots_r_orig;
    slots_w = (scene.measure_cur_num_slots <= scene.measure_max_rows ? 2 : 4) * slots_r;
    slots_x = (scene.measure_cur_num_slots <= scene.measure_max_rows ? center_x - slots_r : center_x - 3 * slots_r);

    scene.measure_slots_rect = {slots_x, slots_y_min, slots_w, slots_y_rng};
    // draw bounding rectangle
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, scene.measure_slots_rect.x, scene.measure_slots_rect.y, scene.measure_slots_rect.w, scene.measure_slots_rect.h, 20);
    if (QUIState::ui_state.scene.lastTime - QUIState::ui_state.scene.measures_last_tap_t > QUIState::ui_state.scene.measures_touch_timeout){
      nvgStrokeColor(s->vg, nvgRGBA(200,200,200,200));
    }
    else{
      nvgStrokeColor(s->vg, COLOR_GRACE_BLUE_ALPHA(200));
    }
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    nvgFillColor(s->vg, nvgRGBA(0,0,0,100));
    nvgFill(s->vg);

    char const * deg = Hardware::EON() ? "°" : "°";
    
    // now start from the top and draw the current set of metrics
    for (int ii = 0; ii < scene.measure_cur_num_slots; ++ii){
      try{
        int i = ii;
        if (scene.measure_cur_num_slots > scene.measure_max_rows && i >= scene.measure_num_rows){
          i += scene.measure_row_offset;
        }

        char name[16], val[16], unit[8];
        snprintf(name, sizeof(name), "");
        snprintf(val, sizeof(val), "");
        snprintf(unit, sizeof(unit), "");
        NVGcolor val_color, label_color, unit_color;
        int val_font_size, label_font_size, unit_font_size;
        int g, b;
        float p;
        
        val_color = default_val_color;
        label_color = default_name_color;
        unit_color = default_unit_color;
        val_font_size = default_val_font_size;
        label_font_size = default_name_font_size;
        unit_font_size = default_unit_font_size;

        // switch to get metric strings/colors 
        switch (scene.measure_slots[i]){

          
          case UIMeasure::CPU_TEMPC: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getCpuTempC()[0]);
            snprintf(unit, sizeof(unit), "%sC", deg);
            snprintf(name, sizeof(name), "CPU온도");}
            break;
          
          case UIMeasure::MEMORY_TEMPC: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getMemoryTempC());
            snprintf(unit, sizeof(unit), "%sC", deg);
            snprintf(name, sizeof(name), "MEM TEMP");}
            break;
          
          case UIMeasure::AMBIENT_TEMPC: 
            {
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%.0f", scene.deviceState.getAmbientTempC());
            snprintf(unit, sizeof(unit), "%sC", deg);
            snprintf(name, sizeof(name), "이온온도");}
            break;
          
          case UIMeasure::CPU_PERCENT: 
            {
            auto cpus = scene.deviceState.getCpuUsagePercent();
            float cpu = 0.;
            int num_cpu = 0;
            for (auto c : cpus){
              cpu += c;
              num_cpu++;
            }
            if (num_cpu > 1){
              cpu /= num_cpu;
            }
            val_color = color_from_thermal_status(int(scene.deviceState.getThermalStatus()));
            snprintf(val, sizeof(val), "%d%%", int(cpu));
            snprintf(name, sizeof(name), "CPU사용량");}
            break;
            
          
          case UIMeasure::MEMORY_USAGE_PERCENT: 
            {
            int mem_perc = scene.deviceState.getMemoryUsagePercent();
            g = 255; 
            b = 255;
            p = 0.011764706 * (mem_perc); // red by 85%
            g -= int(0.5 * p * 255.);
            b -= int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            snprintf(val, sizeof(val), "%d%%", mem_perc);
            snprintf(name, sizeof(name), "메모리사용량");}
            break;
          
          case UIMeasure::FREESPACE_STORAGE: 
            {
            int free_perc = scene.deviceState.getFreeSpacePercent();
            g = 0;
            b = 0;
            p = 0.05 * free_perc; // white at or above 20% freespace
            g += int((0.5+p) * 255.);
            b += int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            snprintf(val, sizeof(val), "%d%%", free_perc);
            snprintf(name, sizeof(name), "저장공간");}
            break;

          case UIMeasure::GPS_ACCURACY:
            {
            if (sm.updated("ubloxGnss")) {
              auto data = sm["ubloxGnss"].getUbloxGnss();
              if (data.which() == cereal::UbloxGnss::MEASUREMENT_REPORT) {
                scene.satelliteCount = data.getMeasurementReport().getNumMeas();
              }
              auto data2 = sm["gpsLocationExternal"].getGpsLocationExternal();
              scene.gpsAccuracyUblox = data2.getAccuracy();
            }
            snprintf(name, sizeof(name), "GPS정확도");
            if (scene.gpsAccuracyUblox != 0.00) {
              //show red/orange if gps accuracy is low
              if(scene.gpsAccuracyUblox > 0.85) {
                val_color = nvgRGBA(255, 188, 3, 200);
              }
              if(scene.gpsAccuracyUblox > 1.3) {
                val_color = nvgRGBA(255, 0, 0, 200);
              }
              // gps accuracy is always in meters
              if(scene.gpsAccuracyUblox > 99 || scene.gpsAccuracyUblox == 0) {
                snprintf(val, sizeof(val), "None");
              }else if(scene.gpsAccuracyUblox > 9.99) {
                snprintf(val, sizeof(val), "%.1f", scene.gpsAccuracyUblox);
              }
              else {
                snprintf(val, sizeof(val), "%.2f", scene.gpsAccuracyUblox);
              }
              snprintf(unit, sizeof(unit), "%d", scene.satelliteCount);
            }}
            break;

          case UIMeasure::ALTITUDE:
            {
            if (sm.updated("gpsLocationExternal")) {
              auto data2 = sm["gpsLocationExternal"].getGpsLocationExternal();
              scene.altitudeUblox = data2.getAltitude();
              scene.gpsAccuracyUblox = data2.getAccuracy();
            }
            snprintf(name, sizeof(name), "고도");
            if (scene.gpsAccuracyUblox != 0.00) {
              float tmp_val;
              tmp_val = scene.altitudeUblox;
              snprintf(val, sizeof(val), "%.0f", scene.altitudeUblox);
              snprintf(unit, sizeof(unit), "m");

              if (log10(tmp_val) >= 4){
                val_font_size -= 10;
              }
            }}
            break;


          case UIMeasure::ACCELERATION:
            {
            snprintf(name, sizeof(name), "ACCEL");
            snprintf(val, sizeof(val), "%.1f", scene.car_state.getAEgo());
            snprintf(unit, sizeof(unit), "m/s²");
            break;}

          case UIMeasure::LEAD_DISTANCE_LENGTH:
            {
              snprintf(name, sizeof(name), "앞차거리");
              if (scene.lead_status) {
                g = 0;
                b = 0;
                p = 0.0333 * scene.lead_d_rel;
                g += int((0.5+p) * 255.);
                b += int(p * 255.);
                g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
                b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
                val_color = nvgRGBA(255, g, b, 200);
                snprintf(val, sizeof(val), "%.0f", scene.lead_d_rel);
              } else {
                snprintf(val, sizeof(val), "-");
              }
              snprintf(unit, sizeof(unit), "미터");
            }
            break;
        
          case UIMeasure::LEAD_DESIRED_DISTANCE_LENGTH:
            {
              snprintf(name, sizeof(name), "추돌회피거리");
              auto follow_d = scene.desiredFollowDistance * scene.car_state.getVEgo() + scene.stoppingDistance;
              if (scene.lead_status) {
                g = 0;
                b = 0;
                p = 0.0333 * scene.lead_d_rel;
                g += int((0.5+p) * 255.);
                b += int(p * 255.);
                g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
                b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
                val_color = nvgRGBA(255, g, b, 200);
                snprintf(val, sizeof(val), "%d:%d", (int)scene.lead_d_rel, (int)follow_d);
              } else {
                snprintf(val, sizeof(val), "-");
              }
              snprintf(unit, sizeof(unit), "미터");
            }
            break;
          
          case UIMeasure::LEAD_DESIRED_DISTANCE_TIME:
            {
            snprintf(name, sizeof(name), "추돌회피시간");
            if (scene.lead_status && scene.car_state.getVEgo() > 0.5) {
              float follow_t = scene.lead_d_rel / scene.car_state.getVEgo();
              float des_follow_t = scene.desiredFollowDistance + scene.stoppingDistance / scene.car_state.getVEgo();
              g = 0;
              b = 0;
              p = 0.6667 * follow_t;
              g += int((0.5+p) * 255.);
              b += int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
              snprintf(val, sizeof(val), "%.1f:%.1f", follow_t, des_follow_t);
            } else {
              snprintf(val, sizeof(val), "-");
            }
            snprintf(unit, sizeof(unit), "초");}
            break;
          
          case UIMeasure::LEAD_VELOCITY_RELATIVE:
            {
            snprintf(name, sizeof(name), "앞차속도");
            if (scene.lead_status) {
              g = 255; 
              b = 255;
              p = -0.2 * (scene.lead_v_rel);
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
              // lead car relative speed is always in meters
              snprintf(val, sizeof(val), "%.1f", (scene.lead_v_rel * 3.6));
            } else {
              snprintf(val, sizeof(val), "-");
            }
            snprintf(unit, sizeof(unit), "km/h");;
            }
            break;

          case UIMeasure::STEERING_ANGLE: 
            {
            snprintf(name, sizeof(name), "핸들각");
            float angleSteers = scene.angleSteers > 0. ? scene.angleSteers : -scene.angleSteers;
            g = 255;
            b = 255;
            p = 0.0333 * angleSteers;
            g -= int(0.5 * p * 255.);
            b -= int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            // steering is in degrees
            if (scene.angleSteers < 10.){
              snprintf(val, sizeof(val), "%.1f%s", scene.angleSteers, deg);
            }
            else{
              snprintf(val, sizeof(val), "%.0f%s", scene.angleSteers, deg);
            }
            }
            break;

          case UIMeasure::DESIRED_STEERING_ANGLE: 
            {
            snprintf(name, sizeof(name), "필요조향각");
            float angleSteers = scene.angleSteers > 0. ? scene.angleSteers : -scene.angleSteers;
            g = 255;
            b = 255;
            p = 0.0333 * angleSteers;
            g -= int(0.5 * p * 255.);
            b -= int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            if (scene.controls_state.getEnabled()) {
              // steering is in degrees
              if (scene.angleSteers < 10. && scene.angleSteersDes < 10.){
                snprintf(val, sizeof(val), "%.1f%s:%.1f%s", scene.angleSteers, deg, scene.angleSteersDes, deg);
              }
              else{
                snprintf(val, sizeof(val), "%.0f%s:%.0f%s", scene.angleSteers, deg, scene.angleSteersDes, deg);
              }
              val_font_size += 12;
            }else{
              if (scene.angleSteers < 10.){
                snprintf(val, sizeof(val), "%.1f%s", scene.angleSteers, deg);
              }
              else{
                snprintf(val, sizeof(val), "%.0f%s", scene.angleSteers, deg);
              }
            }
            }
            break;

          case UIMeasure::STEERING_ANGLE_ERROR: 
            {
            snprintf(name, sizeof(name), "조향각차이");
            float angleSteers = scene.angleSteersErr > 0. ? scene.angleSteersErr : -scene.angleSteersErr;
            if (scene.controls_state.getEnabled()) {
              g = 255;
              b = 255;
              p = 0.2 * angleSteers;
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
              // steering is in degrees
              if (angleSteers < 10.){
                snprintf(val, sizeof(val), "%.1f%s", scene.angleSteersErr, deg);
              }
              else{
                snprintf(val, sizeof(val), "%.0f%s", scene.angleSteersErr, deg);
              }
              val_font_size += 12;
            }else{
              snprintf(val, sizeof(val), "-");
            }
            }
            break;

          case UIMeasure::ENGINE_RPM: 
            {
              snprintf(name, sizeof(name), "엔진 RPM");
              if(scene.engineRPM == 0) {
                snprintf(val, sizeof(val), "OFF");
              }
              else {
                snprintf(val, sizeof(val), "%d", scene.engineRPM);
              }
            }
            break;
            
           
          case UIMeasure::COOLANT_TEMPC: 
            {
              snprintf(name, sizeof(name), "냉각수온");
              snprintf(unit, sizeof(unit), "%sC", deg);
              int temp = scene.car_state.getEngineCoolantTemp();
              snprintf(val, sizeof(val), "%d", temp);
              if(scene.engineRPM > 0) {
                if (temp < 87){
                  val_color = nvgRGBA(84, 207, 249, 200); // cyan if too cool
                }
                else if (temp > 120){
                  val_color = nvgRGBA(255, 0, 0, 200); // red if too hot
                }
                else if (temp > 105){
                  val_color = nvgRGBA(255, 169, 63, 200); // orange if close to too hot
                }
              }
            }
            break;

          case UIMeasure::PERCENT_GRADE:
            {
            auto data2 = sm["gpsLocationExternal"].getGpsLocationExternal();
            float altitudeUblox = data2.getAltitude();
            float gpsAccuracyUblox = data2.getAccuracy();
            if (scene.car_state.getVEgo() > 0.0){
              scene.percentGradeCurDist += scene.car_state.getVEgo() * (scene.lastTime - scene.percentGradeLastTime);
              if (scene.percentGradeCurDist > scene.percentGradeLenStep){ // record position/elevation at even length intervals
                float prevDist = scene.percentGradePositions[scene.percentGradeRollingIter];
                scene.percentGradeRollingIter++;
                if (scene.percentGradeRollingIter >= scene.percentGradeNumSamples){
                  if (!scene.percentGradeIterRolled){
                    scene.percentGradeIterRolled = true;
                    // Calculate initial mean percent grade
                    float u = 0.;
                    for (int i = 0; i < scene.percentGradeNumSamples; ++i){
                      float rise = scene.percentGradeAltitudes[i] - scene.percentGradeAltitudes[(i+1)%scene.percentGradeNumSamples];
                      float run = scene.percentGradePositions[i] - scene.percentGradePositions[(i+1)%scene.percentGradeNumSamples];
                      if (run != 0.){
                        scene.percentGrades[i] = rise/run * 100.;
                        u += scene.percentGrades[i];
                      }
                    }
                    u /= float(scene.percentGradeNumSamples);
                    scene.percentGrade = u;
                  }
                  scene.percentGradeRollingIter = 0;
                }
                scene.percentGradeAltitudes[scene.percentGradeRollingIter] = altitudeUblox;
                scene.percentGradePositions[scene.percentGradeRollingIter] = prevDist + scene.percentGradeCurDist;
                if (scene.percentGradeIterRolled){
                  float rise = scene.percentGradeAltitudes[scene.percentGradeRollingIter] - scene.percentGradeAltitudes[(scene.percentGradeRollingIter+1)%scene.percentGradeNumSamples];
                  float run = scene.percentGradePositions[scene.percentGradeRollingIter] - scene.percentGradePositions[(scene.percentGradeRollingIter+1)%scene.percentGradeNumSamples];
                  if (run != 0.){
                    // update rolling average
                    float newGrade = rise/run * 100.;
                    scene.percentGrade -= scene.percentGrades[scene.percentGradeRollingIter] / float(scene.percentGradeNumSamples);
                    scene.percentGrade += newGrade / float(scene.percentGradeNumSamples);
                    scene.percentGrades[scene.percentGradeRollingIter] = newGrade;
                  }
                }
                scene.percentGradeCurDist = 0.;
              }
            }
            scene.percentGradeLastTime = scene.lastTime;

            snprintf(name, sizeof(name), "GRADE (GPS)");
            if (scene.percentGradeIterRolled && scene.percentGradePositions[scene.percentGradeRollingIter] >= scene.percentGradeMinDist && gpsAccuracyUblox != 0.00){
              g = 255;
              b = 255;
              p = 0.125 * (scene.percentGrade > 0 ? scene.percentGrade : -scene.percentGrade); // red by 8% grade
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
              snprintf(val, sizeof(val), "%.1f%%", scene.percentGrade);
            }
            else{
              snprintf(val, sizeof(val), "-");
            }}
            break;
          
          case UIMeasure::PERCENT_GRADE_DEVICE:
            {
            scene.percentGradeDevice = tan(scene.car_state.getPitch()) * 100.;
            snprintf(name, sizeof(name), "GRADE");
            g = 255;
            b = 255;
            p = 0.125 * (scene.percentGradeDevice > 0 ? scene.percentGradeDevice : -scene.percentGradeDevice); // red by 8% grade
            g -= int(0.5 * p * 255.);
            b -= int(p * 255.);
            g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
            b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
            val_color = nvgRGBA(255, g, b, 200);
            snprintf(val, sizeof(val), "%.1f%%", scene.percentGradeDevice);
            }
            break;
                    
          case UIMeasure::ROLL_DEVICE:
            {
            float degroll = nvgRadToDeg(scene.device_roll);
            snprintf(name, sizeof(name), "DEVICE ROLL");
            val_color = nvgRGBA(255, 255, 255, 200);
            snprintf(val, sizeof(val), "%.1f°", degroll);
            }
            break;

          case UIMeasure::ROLL:
            {
            float degroll = nvgRadToDeg(scene.road_roll);
            snprintf(name, sizeof(name), "ROAD ROLL");
            val_color = nvgRGBA(255, 255, 255, 200);
            snprintf(val, sizeof(val), "%.1f°", degroll);
            }
            break;

          case UIMeasure::FOLLOW_LEVEL: 
            {
              std::string gap;
              snprintf(name, sizeof(name), "GAP");
              if (scene.dynamic_follow_active){
                snprintf(val, sizeof(val), "%.1f", scene.dynamic_follow_level);
              }else
              {
                switch (int(scene.car_state.getReaddistancelines())){
                  case 1:
                  gap =  "I";
                  break;
                
                  case 2:
                  gap =  "I I";
                  break;
                
                  case 3:
                  gap =  "I I I";
                  break;
                
                  default:
                  gap =  "";
                  break;
                }
                snprintf(val, sizeof(val), "%s", gap.c_str());
              }
            }
            break;
            
          case UIMeasure::HVB_VOLTAGE: 
            {
              snprintf(name, sizeof(name), "HVB VOLT");
              snprintf(unit, sizeof(unit), "V");
              float temp = scene.car_state.getHvbVoltage();
              snprintf(val, sizeof(val), "%.0f", temp);
              g = 255;
              b = 255;
              p = temp - 360.;
              p = p > 0 ? p : -p;
              p *= 0.01666667; // red by the time voltage deviates from nominal voltage (360) by 60V deviation from nominal
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
            }
            break;
          
          case UIMeasure::HVB_CURRENT: 
            {
              snprintf(name, sizeof(name), "HVB CUR");
              snprintf(unit, sizeof(unit), "A");
              float temp = -scene.car_state.getHvbCurrent();
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              g = 255;
              b = 255;
              p = scene.car_state.getHvbVoltage() - 360.;
              p = p > 0 ? p : -p;
              p *= 0.01666667; // red by the time voltage deviates from nominal voltage (360) by 60V deviation from nominal
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
            }
            break;
          
          case UIMeasure::HVB_WATTAGE: 
            {
              snprintf(name, sizeof(name), "HVB POW");
              snprintf(unit, sizeof(unit), "kW");
              float temp = -scene.car_state.getHvbWattage();
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              g = 255;
              b = 255;
              p = scene.car_state.getHvbVoltage() - 360.;
              p = p > 0 ? p : -p;
              p *= 0.01666667; // red by the time voltage deviates from nominal voltage (360) by 60V deviation from nominal
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
            }
            break;
          
          case UIMeasure::HVB_WATTVOLT: 
            {
              snprintf(name, sizeof(name), "HVB kW");
              float temp = -scene.car_state.getHvbWattage();
              if (abs(temp) >= 100.){
                snprintf(val, sizeof(val), "%.0f", temp);
              }
              else{
                snprintf(val, sizeof(val), "%.1f", temp);
              }
              temp = scene.car_state.getHvbVoltage();
              snprintf(unit, sizeof(unit), "%.0fV", temp);
              g = 255;
              b = 255;
              p = temp - 360.;
              p = p > 0 ? p : -p;
              p *= 0.01666667; // red by the time voltage deviates from nominal voltage (360) by 60V deviation from nominal
              g -= int(0.5 * p * 255.);
              b -= int(p * 255.);
              g = (g >= 0 ? (g <= 255 ? g : 255) : 0);
              b = (b >= 0 ? (b <= 255 ? b : 255) : 0);
              val_color = nvgRGBA(255, g, b, 200);
            }
            break;
            
          case UIMeasure::LANE_WIDTH: 
            {
              snprintf(name, sizeof(name), "차폭 W");
              snprintf(unit, sizeof(unit), "m");
              snprintf(val, sizeof(val), "%f.1", scene.lateralPlan.laneWidth);

            }
            break;
            
          case UIMeasure::DEVICE_BATTERY: 
            {
              snprintf(name, sizeof(name), "이온전류");
              snprintf(unit, sizeof(unit), "%.1f A", float(scene.deviceState.getBatteryCurrent()) * 1e-6);
              snprintf(val, sizeof(val), "%d", scene.deviceState.getBatteryPercent());
            }
            break;

          case UIMeasure::VISION_VF: 
            {
              snprintf(name, sizeof(name), "V: VF");
              snprintf(val, sizeof(val), "%.2f", (float)scene.longitudinal_plan.getVisionVf());
            }
            break;

          default: {// invalid number
            snprintf(name, sizeof(name), "INVALID");
            snprintf(val, sizeof(val), "42");}
            break;
        }

        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
        // now print the metric
        // first value
        
        int vallen = strlen(val);
        if (vallen > 4){
          val_font_size -= (vallen - 4) * 5;
        }
        int unitlen = strlen(unit);
        if (unitlen > 5){
          unit_font_size -= (unitlen - 5) * 5;
        }
        int slot_x = scene.measure_slots_rect.x + (scene.measure_cur_num_slots <= scene.measure_max_rows ? 0 : (i < scene.measure_max_rows ? slots_r * 2 : 0));
        int x = slot_x + slots_r - unit_font_size / 2;
        if (i >= scene.measure_max_rows){
          x = slot_x + slots_r + unit_font_size / 2;
        }
        int slot_y = scene.measure_slots_rect.y + (i % scene.measure_num_rows) * slot_y_rng;
        int slot_y_mid = slot_y + slot_y_rng / 2;
        int y = slot_y_mid + slot_y_rng / 2 - 8 - label_font_size;
        if (strlen(name) == 0){
          y += label_font_size / 2;
        }
        if (strlen(unit) == 0){
          x = slot_x + slots_r;
        }
        nvgFontFace(s->vg, "sans-semibold");
        nvgFontSize(s->vg, val_font_size);
        nvgFillColor(s->vg, val_color);
        nvgText(s->vg, x, y, val, NULL);
      
        // now label
        y = slot_y_mid + slot_y_rng / 2 - 9;
        nvgFontFace(s->vg, "sans-regular");
        nvgFontSize(s->vg, label_font_size);
        nvgFillColor(s->vg, label_color);
        nvgText(s->vg, x, y, name, NULL);
      
        // now unit
        if (strlen(unit) > 0){
          nvgSave(s->vg);
          int rx = slot_x + slots_r * 2;
          if (i >= 5){
            rx = slot_x;
            nvgTranslate(s->vg, rx + 13, slot_y_mid);
            nvgRotate(s->vg, 1.5708); //-90deg in radians
          }
          else{
            nvgTranslate(s->vg, rx - 13, slot_y_mid);
            nvgRotate(s->vg, -1.5708); //-90deg in radians
          }
          nvgFontFace(s->vg, "sans-regular");
          nvgFontSize(s->vg, unit_font_size);
          nvgFillColor(s->vg, unit_color);
          nvgText(s->vg, 0, 0, unit, NULL);
          nvgRestore(s->vg);
        }
      // update touch rect
      scene.measure_slot_touch_rects[i] = {slot_x, slot_y, slots_r * 2, slot_y_rng};
      }
      catch(...){}    
    }
  }
}


static void ui_draw_vision_turnspeed(UIState *s) {
  const float mapTurnSpeed = s->scene.longitudinal_plan.getTurnSpeed();
  auto visionTurnControllerState = s->scene.longitudinal_plan.getVisionTurnControllerState();
  const bool vision_active = visionTurnControllerState > cereal::LongitudinalPlan::VisionTurnControllerState::DISABLED;
  const float visionTurnSpeed = vision_active ? s->scene.longitudinal_plan.getVisionTurnSpeed() : 0.;
  float turnSpeed;
  if (mapTurnSpeed > 0. && visionTurnSpeed > 0.){
    turnSpeed = mapTurnSpeed < visionTurnSpeed ? mapTurnSpeed : visionTurnSpeed;
  }
  else if (mapTurnSpeed > 0.){
    turnSpeed = mapTurnSpeed;
  }
  else if (visionTurnSpeed > 0.){
    turnSpeed = visionTurnSpeed;
  }
  else{
    turnSpeed = 0.;
  }
  const float vEgo = s->scene.car_state.getVEgo();  
  auto source = s->scene.longitudinal_plan.getLongitudinalPlanSource();
  const bool manual_long = (s->scene.car_state.getOnePedalModeActive() || s->scene.car_state.getCoastOnePedalModeActive());
  const bool show = (turnSpeed > 0.0 && ((turnSpeed < vEgo+2.24 && !manual_long) || s->scene.show_debug_ui));

  if (show) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    Rect speed_sign_rect = Rect{maxspeed_rect.centerX() - speed_sgn_r, 
      maxspeed_rect.bottom() + 2 * (bdr_s + speed_sgn_r), 
      2 * speed_sgn_r, 
      maxspeed_rect.h};
    const float speed = turnSpeed * (s->scene.is_metric ? 3.6 : 2.2369362921);

    if (vision_active){
      // vision turn controller, so need sign of curvature to know curve direction
      int curveSign = 0;
      if (visionTurnControllerState == cereal::LongitudinalPlan::VisionTurnControllerState::ENTERING){
        curveSign = s->scene.longitudinal_plan.getVisionMaxPredictedCurvature() > 0. ? -1 : 1;
      }
      else {
        auto curvatures = s->scene.lateral_plan.getCurvatures();
        for (auto curvature : curvatures){
          curveSign = curvature > 0. ? -1 : 1;
          break;
        }
      }

      const bool is_active = source == cereal::LongitudinalPlan::LongitudinalPlanSource::TURN;

      const int distToTurn = visionTurnControllerState > cereal::LongitudinalPlan::VisionTurnControllerState::ENTERING ? -1 : int(s->scene.longitudinal_plan.getVisionMaxPredictedLateralAccelerationDistance() * 
                                (s->scene.is_metric ? 1.0 : 3.28084) / 10) * 10;
      const std::string distance_str = visionTurnControllerState > cereal::LongitudinalPlan::VisionTurnControllerState::ENTERING ? "TURN" : "VIS";

      ui_draw_turn_speed_sign(s, speed_sign_rect.centerX(), speed_sign_rect.centerY(), 
                              speed_sign_rect.w, speed, 
                              curveSign, distToTurn > 0 ? distance_str.c_str() : "", "sans-bold", is_active);
    }
    else{
      auto turnSpeedControlState = s->scene.longitudinal_plan.getTurnSpeedControlState();
      const bool is_active = turnSpeedControlState > cereal::LongitudinalPlan::SpeedLimitControlState::TEMP_INACTIVE;

      

      const int curveSign = s->scene.longitudinal_plan.getTurnSign();
      const int distToTurn = int(s->scene.longitudinal_plan.getDistToTurn() * 
                                (s->scene.is_metric ? 1.0 : 3.28084) / 10) * 10;
      const std::string distance_str = std::to_string(distToTurn) + (s->scene.is_metric ? "m" : "f");

      ui_draw_turn_speed_sign(s, speed_sign_rect.centerX(), speed_sign_rect.centerY(), 
                              speed_sign_rect.w, speed, 
                              curveSign, distToTurn > 0 ? distance_str.c_str() : "", "sans-bold", is_active);
    }



  }
}

static void ui_draw_vision_speed(UIState *s) {
  const float speed = std::max(0.0, (*s->sm)["carState"].getCarState().getVEgo() * (s->scene.is_metric ? 3.6 : 2.2369363));
  const std::string speed_str = std::to_string((int)std::nearbyint(speed));
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, s->fb_w / 2, 210, speed_str.c_str(), 96 * 2.5, COLOR_WHITE, "sans-bold");
  ui_draw_text(s, s->fb_w / 2, 290, s->scene.is_metric ? "km/h" : "mph", 36 * 2.5, COLOR_WHITE_ALPHA(200), "sans-regular");
  s->scene.speed_rect = {s->fb_w / 2 - 50, 150, 150, 300};
}

static void ui_draw_speed_limit(UIState *s)
{
    const UIScene *scene = &s->scene;
    SubMaster &sm = *(s->sm);
    // cereal::CarControl::SccSmoother::Reader scc_smoother = scene->car_control.getSccSmoother();
    auto roadLimitSpeed = sm["roadLimitSpeed"].getRoadLimitSpeed();

    int activeNDA = roadLimitSpeed.getActive();

    int camLimitSpeed = roadLimitSpeed.getCamLimitSpeed();
    int camLimitSpeedLeftDist = roadLimitSpeed.getCamLimitSpeedLeftDist();

    int sectionLimitSpeed = roadLimitSpeed.getSectionLimitSpeed();
    int sectionLeftDist = roadLimitSpeed.getSectionLeftDist();

    int limit_speed = 0;
    int left_dist = 0;

    if(camLimitSpeed >= 0 && camLimitSpeedLeftDist > 0) {
      limit_speed = camLimitSpeed;
      left_dist = camLimitSpeedLeftDist;
    }
    else if(sectionLimitSpeed >= 0 && sectionLeftDist > 0) {
      limit_speed = sectionLimitSpeed;
      left_dist = sectionLeftDist;
    }

    if(activeNDA > 0)
    {
        int w = 120;
        int h = 54;
        int x = (s->fb_w + (bdr_s*2))/3 - w/2 - bdr_s*4;
        int y = bdr_s - 20;

        const char* img = activeNDA == 1 ? "img_nda" : "img_hda";
        ui_draw_image(s, {x, y, w, h}, img, 1.f);
    }
    if(limit_speed > 10 && left_dist > 0)
    {
       int w = 180;
        int h = 180;
        int x = (s->viz_rect.x + bdr_s + 400);
        int y = 70;
        char str[32];

        nvgBeginPath(s->vg);
        nvgRoundedRect(s->vg, x, y, w, h, 185);
        nvgStrokeColor(s->vg, nvgRGBA(255, 0, 0, 200));
        nvgStrokeWidth(s->vg, 15);
        nvgStroke(s->vg);

        NVGcolor fillColor = nvgRGBA(0, 0, 0, 50);
        nvgFillColor(s->vg, fillColor);
        nvgFill(s->vg);

        nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 250));

        nvgFontSize(s->vg, 110);
        nvgFontFace(s->vg, "sans-bold");
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

        snprintf(str, sizeof(str), "%d", limit_speed);
        nvgText(s->vg, x+w/2, y+h/2, str, NULL);

        nvgBeginPath(s->vg);
        nvgRect(s->vg, x+w/2-100, y+h-30, 190, 80);
        NVGcolor squareColor = nvgRGBA(255, 0, 0, 200);
        nvgFillColor(s->vg, squareColor);
        nvgFill(s->vg);
        nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 250));

        nvgFontSize(s->vg, 86);
        nvgFontFace(s->vg, "sans-bold");
        nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

        if(left_dist >= 1000)
          snprintf(str, sizeof(str), "%.1fkm", left_dist / 1000.f);
        else if(left_dist > 0)
          snprintf(str, sizeof(str), "%dm", left_dist);

        nvgText(s->vg, x+w/2, y+h, str, NULL);
    }
}


static void ui_draw_vision_event(UIState *s) {
  s->scene.wheel_touch_rect = {1,1,1,1};
  if (s->scene.engageable) {
    // draw steering wheel
    const float rot_angle = -s->scene.angleSteers * 0.01745329252;
    const int radius = 88;
    const int center_x = s->fb_w - radius - bdr_s * 2;
    const int center_y = radius  + (bdr_s * 1.5);
    const QColor &color = bg_colors[(s->scene.car_state.getLkMode() ? s->status : UIStatus::STATUS_DISENGAGED)];
    NVGcolor nvg_color = nvgRGBA(color.red(), color.green(), color.blue(), color.alpha());
  
    // draw circle behind wheel
    s->scene.wheel_touch_rect = {center_x - radius, center_y - radius, 2 * radius, 2 * radius};
    ui_fill_rect(s->vg, s->scene.wheel_touch_rect, nvg_color, radius);

    // now rotate and draw the wheel
    nvgSave(s->vg);
    nvgTranslate(s->vg, center_x, center_y);
    if (s->scene.wheel_rotates){
      nvgRotate(s->vg, rot_angle);
    }
    ui_draw_image(s, {-radius, -radius, 2*radius, 2*radius}, "wheel", 1.0f);
    nvgRestore(s->vg);
    
    // draw extra circle to indiate paused low-speed one-pedal blinker steering is enabled
    if (s->scene.visionBrakingEnabled and !(s->scene.mapBrakingEnabled)){
      nvgBeginPath(s->vg);
      const int r = int(float(radius) * 1.15);
      nvgRoundedRect(s->vg, center_x - r, center_y - r, 2 * r, 2 * r, r);
      nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(255));
      nvgFillColor(s->vg, nvgRGBA(0,0,0,0));
      nvgFill(s->vg);
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
    }
    else if (s->scene.visionBrakingEnabled and s->scene.mapBrakingEnabled){
      nvgBeginPath(s->vg);
      const int r = int(float(radius) * 1.15);
      nvgRoundedRect(s->vg, center_x - r, center_y - r, 2 * r, 2 * r, r);
      nvgStrokeColor(s->vg, s->scene.network_strength > 0 ? COLOR_GREEN_ALPHA(255) : COLOR_RED_ALPHA(255));
      nvgFillColor(s->vg, nvgRGBA(0,0,0,0));
      nvgFill(s->vg);
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
    }
    
    // draw hands on wheel pictogram under wheel pictogram.
    auto handsOnWheelState = (*s->sm)["driverMonitoringState"].getDriverMonitoringState().getHandsOnWheelState();
    if (handsOnWheelState >= cereal::DriverMonitoringState::HandsOnWheelState::WARNING) {
      NVGcolor color = COLOR_RED;
      if (handsOnWheelState == cereal::DriverMonitoringState::HandsOnWheelState::WARNING) {
        color = COLOR_YELLOW;
      } 
      const int wheel_y = center_y + bdr_s + 2 * radius;
      ui_draw_circle_image(s, center_x, wheel_y, radius, "hands_on_wheel", color, 1.0f);
    }
  }
  // draw cell/wifi indicator if map-braking or speed limit control (which require data connection) enabled
  if (s->scene.mapBrakingEnabled || s->scene.speed_limit_control_enabled){
    const int r = 12;
    int x = bdr_s * 2;
    int y = bdr_s - 22;
    for (int i = 0; i < 5; ++i){
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, x, y, 2*r, 2*r, r);
      nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(200));
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(i < s->scene.network_strength ? 200 : 70));
      nvgFill(s->vg);
      nvgStrokeWidth(s->vg, 0);
      nvgStroke(s->vg);
      x += 2*r + 6;
    }
    if (s->scene.network_strength > 0){
      x += 5;
      y -= 9;
      nvgBeginPath(s->vg);
      nvgTextAlign(s->vg, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
      nvgFontFace(s->vg, "sans-semibold");
      nvgFontSize(s->vg, 40);
      nvgFillColor(s->vg, COLOR_WHITE_ALPHA(200));
      nvgText(s->vg, x, y, s->scene.network_type_string.c_str(), NULL);
    }
  }

  // current road name
  if (s->scene.network_strength > 0 && !s->scene.map_open){//} && s->scene.current_road_name != ""){
    nvgBeginPath(s->vg);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
    nvgFontFace(s->vg, "sans-regular");
    nvgFontSize(s->vg, 75);
    nvgFillColor(s->vg, COLOR_WHITE_ALPHA(255));
    nvgText(s->vg, s->fb_w / 2, bdr_s - 31, s->scene.current_road_name.c_str(), NULL);
  }
}

static void ui_draw_vision_face(UIState *s) {
  const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
  const int radius = 96;
  const int center_x = maxspeed_rect.centerX();
  int center_y = s->fb_h - footer_h / 2;
  center_y = offset_button_y(s, center_y, radius);
  ui_draw_circle_image(s, center_x, center_y, radius, "driver_face", s->scene.dm_active);
}

static void ui_draw_vision_brake(UIState *s) {
  if (s->scene.brake_indicator_enabled){
    // scene.brake_percent in [0,50] is engine/regen
    // scene.brake_percent in [51,100] is friction
    int brake_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    int brake_y = s->fb_h - footer_h / 2;
    brake_x = offset_right_side_button_x(s, brake_x, brake_size);
    brake_y = offset_button_y(s, brake_y, brake_size);
    const int brake_r1 = 1;
    const int brake_r2 = brake_size / 3 + 2;
    const float brake_r_range = brake_r2 - brake_r1;
    const int circ_offset = 1;
    float bg_alpha = 0.1 + 0.3 * s->scene.brake_indicator_alpha;
    float img_alpha = 0.15 + 0.85 * s->scene.brake_indicator_alpha;
    if (s->scene.brake_percent > 0 && s->scene.brake_percent <= 50){
      // engine/regen braking indicator only
      int bp = s->scene.brake_percent * 2;
      float p = bp;
      const int brake_r = brake_r1 + int(brake_r_range * p * 0.01);
      bg_alpha = (0.1 + (p * 0.004));
      if (bg_alpha > 0.3){
        bg_alpha = 0.3;
      }
      ui_draw_circle_image(s, brake_x, brake_y, brake_size, "brake_disk", nvgRGBA(0, 0, 0, int(bg_alpha * 255.)), img_alpha);
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, brake_x - brake_r + circ_offset, brake_y - brake_r + circ_offset, 2 * brake_r, 2 * brake_r, brake_r);
      nvgStrokeWidth(s->vg, 9);
      NVGcolor nvg_color = nvgRGBA(131,232,42, 200);
      nvgFillColor(s->vg, nvg_color);
      nvgStrokeColor(s->vg, nvg_color);
      nvgFill(s->vg);
      nvgStroke(s->vg);
    }
    else if (s->scene.brake_percent > 50){
      int bp = (s->scene.brake_percent - 50) * 2;
      bg_alpha = 0.3 + 0.1 * s->scene.brake_indicator_alpha;
      NVGcolor color = nvgRGBA(0, 0, 0, (255 * bg_alpha));
      if (bp > 0 && bp <= 100){
        int r = 0;
        if (bp >= 50){
          float p = 0.01 * float(bp - 50);
          bg_alpha += 0.3 * p;
          r = 200. * p;
        }
        color = nvgRGBA(r, 0, 0, (255 * bg_alpha));
      }
      ui_draw_circle_image(s, brake_x, brake_y, brake_size, "brake_disk", color, img_alpha);
      if (bp <= 100){
        float p = bp;
        
        // friction braking indicator starts at outside of regen indicator and grows from there 
        // do this by increasing radius while decreasing stroke width.
        nvgBeginPath(s->vg);
        const int start_r = brake_r2 + 3;
        const int end_r = brake_size;
        const int brake_r = start_r + float(end_r - start_r) * p * 0.01;
        const int stroke_width = brake_r - brake_r2;
        const int path_r = stroke_width / 2 + brake_r2;
        nvgRoundedRect(s->vg, brake_x - path_r + circ_offset, brake_y - path_r + circ_offset, 2 * path_r, 2 * path_r, path_r);
        nvgStrokeWidth(s->vg, stroke_width);
        int r = 255, g = 255, b = 255, a = 200;
        p *= 0.01;
        g -= int(p * 255.);
        g = (g > 0 ? g : 0);
        b -= int((.4 + p) * 255.);
        b = (b > 0 ? b : 0); // goes from white to orange to red as p goes from 0 to 100
        nvgFillColor(s->vg, nvgRGBA(0,0,0,0));
        nvgStrokeColor(s->vg, nvgRGBA(r,g,b,a));
        nvgFill(s->vg);
        nvgStroke(s->vg);
        
        // another brake image (this way the regen is on top of the background, while the brake disc itself occludes the other indicator)
        ui_draw_circle_image(s, brake_x, brake_y, brake_size, "brake_disk", nvgRGBA(0,0,0,0), img_alpha);
        
        // engine/regen braking indicator
        nvgBeginPath(s->vg);
        nvgRoundedRect(s->vg, brake_x - brake_r2 + circ_offset, brake_y - brake_r2 + circ_offset, 2 * brake_r2, 2 * brake_r2, brake_r2);
        nvgStrokeWidth(s->vg, 9);
        NVGcolor nvg_color = nvgRGBA(131,232,42, 200);
        nvgFillColor(s->vg, nvg_color);
        nvgStrokeColor(s->vg, nvg_color);
        nvgFill(s->vg);
        nvgStroke(s->vg);
      }
    }
    else{
      ui_draw_circle_image(s, brake_x, brake_y, brake_size, "brake_disk", nvgRGBA(0, 0, 0, bg_alpha), img_alpha);
    }
    // s->scene.brake_touch_rect = {1,1,1,1};
  }
}

static void draw_lane_pos_buttons(UIState *s) {
  if (s->vipc_client->connected && s->scene.lane_pos_enabled) {
    const int radius = ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::NONE && !(s->scene.map_open) ? 190 : 100);
    const int right_x = (s->scene.measure_cur_num_slots > 0 
                          ? s->scene.measure_slots_rect.x - 4 * radius / 3
                          : 4 * s->fb_w / 5);
    int left_x = s->fb_w / 5 + 100;
    if (left_x > right_x - 2 * radius - 40){
      left_x = right_x - 2 * radius - 40;
    }
    const int y = offset_button_y(s, s->fb_h / 2, radius);

    // left button
    s->scene.lane_pos_left_touch_rect = {left_x - radius, y - radius, 2 * radius, 2 * radius};
    int radius_inner = 0;
    if (s->scene.lane_pos == 1){
      radius_inner = s->scene.auto_lane_pos_active ? radius : int(float(s->scene.lane_pos_timeout_dist - s->scene.lane_pos_dist_since_set) / float(s->scene.lane_pos_timeout_dist) * float(radius));
      if (radius_inner < 1){
        radius_inner = 1;
      }
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, left_x - radius_inner, y - radius_inner, 2 * radius_inner, 2 * radius_inner, radius_inner);
      nvgFillColor(s->vg, s->scene.auto_lane_pos_active ? COLOR_GRACE_BLUE_ALPHA(100) : COLOR_WHITE_ALPHA(200));
      nvgFill(s->vg);
      ui_draw_circle_image(s, left_x, y, radius, "lane_pos_left", COLOR_BLACK_ALPHA(80), 1.0);
    }
    else{
      ui_draw_circle_image(s, left_x, y, radius, "lane_pos_left", COLOR_BLACK_ALPHA(80), 0.4);
    }
    
    if (s->scene.lane_pos == 1 || s->scene.auto_lane_pos_active){
      // outline of button when active
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, left_x - radius, y - radius, 2 * radius, 2 * radius, radius);
      if (s->scene.auto_lane_pos_active){
        nvgStrokeWidth(s->vg, 30);
      }
      nvgStrokeColor(s->vg, s->scene.auto_lane_pos_active ? COLOR_GRACE_BLUE_ALPHA(100) : COLOR_WHITE_ALPHA(200));
      nvgStroke(s->vg);
    }
    
    // right button
    s->scene.lane_pos_right_touch_rect = {right_x - radius, y - radius, 2 * radius, 2 * radius};
    radius_inner = 0;
    if (s->scene.lane_pos == -1){
      radius_inner = s->scene.auto_lane_pos_active ? radius : int(float(s->scene.lane_pos_timeout_dist - s->scene.lane_pos_dist_since_set) / float(s->scene.lane_pos_timeout_dist) * float(radius));
      if (radius_inner < 1){
        radius_inner = 1;
      }
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, right_x - radius_inner, y - radius_inner, 2 * radius_inner, 2 * radius_inner, radius_inner);
      nvgFillColor(s->vg, s->scene.auto_lane_pos_active ? COLOR_GRACE_BLUE_ALPHA(100) : COLOR_WHITE_ALPHA(200));
      nvgFill(s->vg);
      ui_draw_circle_image(s, right_x, y, radius, "lane_pos_right", COLOR_BLACK_ALPHA(80), 1.0);
    }
    else{
      ui_draw_circle_image(s, right_x, y, radius, "lane_pos_right", COLOR_BLACK_ALPHA(80), 0.4);
    }
    if (s->scene.lane_pos == -1 || s->scene.auto_lane_pos_active){
      // outline of button when active
      nvgBeginPath(s->vg);
      nvgRoundedRect(s->vg, right_x - radius, y - radius, 2 * radius, 2 * radius, radius);
      if (s->scene.auto_lane_pos_active){
        nvgStrokeWidth(s->vg, 30);
      }
      nvgStrokeColor(s->vg, s->scene.auto_lane_pos_active ? COLOR_GRACE_BLUE_ALPHA(100) : COLOR_WHITE_ALPHA(200));
      nvgStroke(s->vg);
    }


  }
}

static void draw_accel_mode_button(UIState *s) {
  if (s->vipc_client->connected && s->scene.accel_mode_button_enabled) {
    const int radius = 72;
    int center_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    if (s->scene.brake_percent >= 0){
      center_x -= brake_size + (s->scene.map_open ? 1.2 : 3) * bdr_s + radius;
    }
    int center_y = s->fb_h - footer_h / 2 - radius / 2;
    center_y = offset_button_y(s, center_y, radius);
    center_x = offset_right_side_button_x(s, center_x, radius);
    int btn_w = radius * 2;
    int btn_h = radius * 2;
    int btn_x1 = center_x - 0.5 * radius;
    int btn_y = center_y - 0.5 * radius;
    int btn_xc1 = btn_x1 + radius;
    int btn_yc = btn_y + radius;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, radius);
    // nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    nvgStrokeColor(s->vg, nvgRGBA(0,0,0,80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    nvgFontSize(s->vg, 52);

    if (s->scene.accel_mode == 0) { // normal
      nvgStrokeColor(s->vg, nvgRGBA(200,200,200,200));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(0,0,0,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Stock",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"accel",NULL);
    } else if (s->scene.accel_mode == 1) { // sport
      nvgStrokeColor(s->vg, nvgRGBA(142,0,11,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(142,0,11,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Sport",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"accel",NULL);
    } else if (s->scene.accel_mode == 2) { // eco
      nvgStrokeColor(s->vg, nvgRGBA(74,132,23,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(74,132,23,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Eco",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"accel",NULL);
    }
    
    
    s->scene.accel_mode_touch_rect = Rect{center_x - laneless_btn_touch_pad, 
                                                center_y - laneless_btn_touch_pad,
                                                radius + 2 * laneless_btn_touch_pad, 
                                                radius + 2 * laneless_btn_touch_pad}; 
  }
}

static void draw_dynamic_follow_mode_button(UIState *s) {
  if (s->vipc_client->connected && s->scene.dynamic_follow_mode_button_enabled) {
    const int radius = 72;
    int center_x = s->fb_w - face_wheel_radius - bdr_s * 2;
    if (s->scene.brake_percent >= 0){
      center_x -= brake_size + (s->scene.map_open ? 1.2 : 3) * bdr_s + radius;
    }
    if (s->scene.accel_mode_button_enabled){
      center_x -= (s->scene.map_open ? 1.2 : 2) * bdr_s + 2 * radius;
    }
    int center_y = s->fb_h - footer_h / 2 - radius / 2;
    center_y = offset_button_y(s, center_y, radius);
    center_x = offset_right_side_button_x(s, center_x, radius);
    int btn_w = radius * 2;
    int btn_h = radius * 2;
    int btn_x1 = center_x - 0.5 * radius;
    int btn_y = center_y - 0.5 * radius;
    int btn_xc1 = btn_x1 + radius;
    int btn_yc = btn_y + radius;
    float df_level = s->scene.dynamic_follow_level_ui >= 0. ? s->scene.dynamic_follow_level_ui : 0.;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, radius);
    // nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    const bool df_active = s->scene.dynamic_follow_active && !(s->scene.car_state.getOnePedalModeActive() || s->scene.car_state.getCoastOnePedalModeActive());
    if (df_active){
      int r, b, g;
      int bg_r, bg_b, bg_g;
      for (int i = 1; i < 3; ++i){
        if (df_level <= i){
          float c = float(i) - df_level;
          r = float(s->scene.dynamic_follow_r[i-1]) * c + float(s->scene.dynamic_follow_r[i]) * (1. - c);
          b = float(s->scene.dynamic_follow_b[i-1]) * c + float(s->scene.dynamic_follow_b[i]) * (1. - c);
          g = float(s->scene.dynamic_follow_g[i-1]) * c + float(s->scene.dynamic_follow_g[i]) * (1. - c);
          bg_r = float(s->scene.dynamic_follow_bg_r[i-1]) * c + float(s->scene.dynamic_follow_bg_r[i]) * (1. - c);
          bg_b = float(s->scene.dynamic_follow_bg_b[i-1]) * c + float(s->scene.dynamic_follow_bg_b[i]) * (1. - c);
          bg_g = float(s->scene.dynamic_follow_bg_g[i-1]) * c + float(s->scene.dynamic_follow_bg_g[i]) * (1. - c);
          break;
        }
      }
      nvgStrokeColor(s->vg, nvgRGBA(r,b,g,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgFillColor(s->vg, nvgRGBA(bg_r,bg_b,bg_g,80));
      nvgFill(s->vg);
    } else{
      nvgStrokeColor(s->vg, nvgRGBA(0,0,0,80));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgStrokeColor(s->vg, nvgRGBA(200,200,200,80));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      nvgFillColor(s->vg, nvgRGBA(0,0,0,80));
      nvgFill(s->vg);
    }
    
    // draw the three follow level strings. adjust alpha and y position to create a rolling effect
    const float dscale = 0.5;
    for (int i = 0; i < 3; ++i){
      char val[16];
      snprintf(val, sizeof(val), "%s", s->scene.dynamic_follow_strs[i].c_str());
      float alpha_f = abs(float(i) - df_level);
      alpha_f = (alpha_f > 1. ? 1. : alpha_f) * 1.5707963268;
      nvgFillColor(s->vg, nvgRGBA(255, 255, 255, int(cos(alpha_f) * (df_active ? 200. : 80.))));
                                
      nvgFontSize(s->vg, 40 + int(cos(alpha_f * 1.5707963268) * 16.));
      
      int text_y = btn_yc;
      if (df_level <= i){
        text_y -= float(radius) * sin(alpha_f) * dscale;
      }
      else{
        text_y += float(radius) * sin(alpha_f) * dscale;
      }
      nvgText(s->vg, btn_xc1, text_y, val, NULL);
    }
    
    s->scene.dynamic_follow_mode_touch_rect = Rect{center_x - laneless_btn_touch_pad, 
                                                center_y - laneless_btn_touch_pad,
                                                radius + 2 * laneless_btn_touch_pad, 
                                                radius + 2 * laneless_btn_touch_pad}; 
  }
}

static void draw_laneless_button(UIState *s) {
  if (s->vipc_client->connected) {
    const Rect maxspeed_rect = {bdr_s * 2, int(bdr_s * 1.5), 184, 202};
    const int vision_face_radius = 96;
    const int radius = 72;
    const int center_x = maxspeed_rect.centerX() + vision_face_radius + bdr_s * (s->scene.map_open ? 0 : 1) + radius;
    int center_y = s->fb_h - footer_h / 2 - radius / 2;
    center_y = offset_button_y(s, center_y, radius);
    int btn_w = radius * 2;
    int btn_h = radius * 2;
    int btn_x1 = center_x - 0.5 * radius;
    int btn_y = center_y - 0.5 * radius;
    int btn_xc1 = btn_x1 + radius;
    int btn_yc = btn_y + radius;
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, radius);
    // nvgRoundedRect(s->vg, btn_x1, btn_y, btn_w, btn_h, 100);
    nvgStrokeColor(s->vg, nvgRGBA(0,0,0,80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
    nvgFontSize(s->vg, 54);

    if (s->scene.laneless_mode == 0) {
      nvgStrokeColor(s->vg, nvgRGBA(0,125,0,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(0,125,0,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Lane",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"only",NULL);
    } else if (s->scene.laneless_mode == 1) {
      nvgStrokeColor(s->vg, nvgRGBA(0,100,255,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(0,100,255,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Lane",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"less",NULL);
    } else if (s->scene.laneless_mode == 2) {
      nvgStrokeColor(s->vg, nvgRGBA(125,0,125,255));
      nvgStrokeWidth(s->vg, 6);
      nvgStroke(s->vg);
      NVGcolor fillColor = nvgRGBA(125,0,125,80);
      nvgFillColor(s->vg, fillColor);
      nvgFill(s->vg);
      nvgFillColor(s->vg, nvgRGBA(255,255,255,200));
      nvgText(s->vg,btn_xc1,btn_yc-20,"Auto",NULL);
      nvgText(s->vg,btn_xc1,btn_yc+20,"Lane",NULL);
    }
    
    s->scene.laneless_btn_touch_rect = Rect{center_x - laneless_btn_touch_pad, 
                                                center_y - laneless_btn_touch_pad,
                                                radius + 2 * laneless_btn_touch_pad, 
                                                radius + 2 * laneless_btn_touch_pad}; 
  }
}

static void ui_draw_vision_header(UIState *s) {
  NVGpaint gradient = nvgLinearGradient(s->vg, 0, header_h - (header_h * 0.4), 0, header_h,
                                        nvgRGBAf(0, 0, 0, 0.45), nvgRGBAf(0, 0, 0, 0));
  ui_fill_rect(s->vg, {0, 0, s->fb_w , header_h}, gradient);
  ui_draw_vision_maxspeed(s);
  ui_draw_vision_speedlimit(s);
  ui_draw_vision_speed(s);
  ui_draw_vision_turnspeed(s);
  ui_draw_vision_event(s);
  ui_draw_speed_limit(s);
}

static void ui_draw_vision(UIState *s) {
  const UIScene *scene = &s->scene;
  // Draw augmented elements
  if (scene->world_objects_visible) {
    ui_draw_world(s);
  }
  // Set Speed, Current Speed, Status/Events
  ui_draw_vision_header(s);
  if ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::NONE
  || (*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::SMALL) {
    ui_draw_vision_face(s);
    ui_draw_vision_brake(s);
    if (!s->scene.map_open || (*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::NONE){
      ui_draw_measures(s);
    }
  }
  else if ((*s->sm)["controlsState"].getControlsState().getAlertSize() == cereal::ControlsState::AlertSize::MID) {
    ui_draw_vision_face(s);
    ui_draw_vision_brake(s);
  }
  if (s->scene.lane_pos_enabled){
    draw_lane_pos_buttons(s);
  }
  if (s->scene.end_to_end) {
    draw_laneless_button(s);
  }
  if (s->scene.accel_mode_button_enabled){
    draw_accel_mode_button(s);
  }
  if (s->scene.dynamic_follow_mode_button_enabled){
    draw_dynamic_follow_mode_button(s);
  }
}

void ui_draw(UIState *s, int w, int h) {
  const bool draw_vision = s->scene.started && s->vipc_client->connected;

  glViewport(0, 0, s->fb_w, s->fb_h);
  if (draw_vision) {
    draw_vision_frame(s);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // NVG drawing functions - should be no GL inside NVG frame
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  if (draw_vision) {
    ui_draw_vision(s);
  }
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

void ui_draw_image(const UIState *s, const Rect &r, const char *name, float alpha) {
  nvgBeginPath(s->vg);
  NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
  nvgRect(s->vg, r.x, r.y, r.w, r.h);
  nvgFillPaint(s->vg, imgPaint);
  nvgFill(s->vg);
}

void ui_draw_rect(NVGcontext *vg, const Rect &r, NVGcolor color, int width, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  nvgStrokeColor(vg, color);
  nvgStrokeWidth(vg, width);
  nvgStroke(vg);
}

static inline void fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor *color, const NVGpaint *paint, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  if (color) nvgFillColor(vg, *color);
  if (paint) nvgFillPaint(vg, *paint);
  nvgFill(vg);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor &color, float radius) {
  fill_rect(vg, r, &color, nullptr, radius);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGpaint &paint, float radius) {
  fill_rect(vg, r, nullptr, &paint, radius);
}

static const char frame_vertex_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "in vec4 aPosition;\n"
  "in vec4 aTexCoord;\n"
  "uniform mat4 uTransform;\n"
  "out vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_Position = uTransform * aPosition;\n"
  "  vTexCoord = aTexCoord;\n"
  "}\n";

static const char frame_fragment_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "precision mediump float;\n"
  "uniform sampler2D uTexture;\n"
  "in vec4 vTexCoord;\n"
  "out vec4 colorOut;\n"
  "void main() {\n"
  "  colorOut = texture(uTexture, vTexCoord.xy);\n"
#ifdef QCOM
  "  vec3 dz = vec3(0.0627f, 0.0627f, 0.0627f);\n"
  "  colorOut.rgb = ((vec3(1.0f, 1.0f, 1.0f) - dz) * colorOut.rgb / vec3(1.0f, 1.0f, 1.0f)) + dz;\n"
#endif
  "}\n";

static const mat4 device_transform = {{
  1.0,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

void ui_nvg_init(UIState *s) {
  // init drawing

  // on EON, we enable MSAA
  s->vg = Hardware::EON() ? nvgCreate(0) : nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  // init fonts
  std::pair<const char *, const char *> fonts[] = {
      {"sans-regular", "../assets/fonts/opensans_regular.ttf"},
      {"sans-semibold", "../assets/fonts/opensans_semibold.ttf"},
      {"sans-bold", "../assets/fonts/opensans_bold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }

  // init images
  std::vector<std::pair<const char *, const char *>> images = {
    {"eye", "../assets/img_eye_open_white.png"},
    {"wheel", "../assets/img_chffr_wheel.png"},
    {"driver_face", "../assets/img_driver_face.png"},
    {"hands_on_wheel", "../assets/img_hands_on_wheel.png"},
    {"turn_left_icon", "../assets/img_turn_left_icon.png"},
    {"turn_right_icon", "../assets/img_turn_right_icon.png"},
    {"map_source_icon", "../assets/img_world_icon.png"},
    {"brake_disk", "../assets/img_brake.png"},
    {"one_pedal_mode", "../assets/offroad/icon_car_pedal.png"},
    {"lane_pos_left", "../assets/offroad/icon_lane_pos_left.png"},
    {"lane_pos_right", "../assets/offroad/icon_lane_pos_right.png"},
    {"img_nda", "../assets/img_nda.png"},
    {"img_hda", "../assets/img_hda.png"}
  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }

  // init gl
  s->gl_shader = std::make_unique<GLShader>(frame_vertex_shader, frame_fragment_shader);
  GLint frame_pos_loc = glGetAttribLocation(s->gl_shader->prog, "aPosition");
  GLint frame_texcoord_loc = glGetAttribLocation(s->gl_shader->prog, "aTexCoord");

  glViewport(0, 0, s->fb_w, s->fb_h);

  glDisable(GL_DEPTH_TEST);

  assert(glGetError() == GL_NO_ERROR);

  float x1 = 1.0, x2 = 0.0, y1 = 1.0, y2 = 0.0;
  const uint8_t frame_indicies[] = {0, 1, 2, 0, 2, 3};
  const float frame_coords[4][4] = {
    {-1.0, -1.0, x2, y1}, //bl
    {-1.0,  1.0, x2, y2}, //tl
    { 1.0,  1.0, x1, y2}, //tr
    { 1.0, -1.0, x1, y1}, //br
  };

  glGenVertexArrays(1, &s->frame_vao);
  glBindVertexArray(s->frame_vao);
  glGenBuffers(1, &s->frame_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, s->frame_vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(frame_coords), frame_coords, GL_STATIC_DRAW);
  glEnableVertexAttribArray(frame_pos_loc);
  glVertexAttribPointer(frame_pos_loc, 2, GL_FLOAT, GL_FALSE,
                        sizeof(frame_coords[0]), (const void *)0);
  glEnableVertexAttribArray(frame_texcoord_loc);
  glVertexAttribPointer(frame_texcoord_loc, 2, GL_FLOAT, GL_FALSE,
                        sizeof(frame_coords[0]), (const void *)(sizeof(float) * 2));
  glGenBuffers(1, &s->frame_ibo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s->frame_ibo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(frame_indicies), frame_indicies, GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  ui_resize(s, s->fb_w, s->fb_h);
}

void ui_resize(UIState *s, int width, int height) {
  s->fb_w = width;
  s->fb_h = height;

  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;

  float zoom = ZOOM / intrinsic_matrix.v[0];

  if (s->wide_camera) {
    zoom *= 0.5;
  }

  float zx = zoom * 2 * intrinsic_matrix.v[2] / width;
  float zy = zoom * 2 * intrinsic_matrix.v[5] / height;

  const mat4 frame_transform = {{
    zx, 0.0, 0.0, 0.0,
    0.0, zy, 0.0, -y_offset / height * 2,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
  }};

  s->rear_frame_mat = matmul(device_transform, frame_transform);

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  nvgTranslate(s->vg, width / 2, height / 2 + y_offset);
  // 2) Apply same scaling as video
  nvgScale(s->vg, zoom, zoom);
  // 3) Put (0, 0) in top left corner of video
  nvgTranslate(s->vg, -intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);

  nvgCurrentTransform(s->vg, s->car_space_transform);
  nvgResetTransform(s->vg);
}
