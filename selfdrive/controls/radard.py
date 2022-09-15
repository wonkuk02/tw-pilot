#!/usr/bin/env python3
import importlib
import math
import numpy as np
from collections import defaultdict, deque

import cereal.messaging as messaging
from cereal import car
from common.numpy_fast import interp
from common.params import Params
from common.realtime import Ratekeeper, Priority, config_realtime_process
from selfdrive.config import RADAR_TO_CAMERA
from selfdrive.controls.lib.cluster.fastcluster_py import cluster_points_centroid
from selfdrive.controls.lib.lane_planner import TRAJECTORY_SIZE
from selfdrive.controls.lib.radar_helpers import Cluster, Track
from selfdrive.swaglog import cloudlog
from selfdrive.hardware import TICI

LEAD_PATH_YREL_MAX_BP = [100., 200.] # [m] distance to lead
LEAD_PATH_YREL_MAX_V = [0.5, 1.5] # [m] an increasing tolerance for farther leads 
LEAD_PATH_DREL_MIN = 110 # [m] only care about far away leads
MIN_LANE_PROB = 0.6  # Minimum lanes probability to allow curvature prediction based on lanes.

class KalmanParams():
  def __init__(self, dt):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.1s and 1.0s
    assert dt > .01 and dt < .1, "Radar time step must be between .01s and 0.1s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [dt * 0.01 for dt in range(1, 11)]
    K0 = [0.12288, 0.14557, 0.16523, 0.18282, 0.19887, 0.21372, 0.22761, 0.24069, 0.2531, 0.26491]
    K1 = [0.29666, 0.29331, 0.29043, 0.28787, 0.28555, 0.28342, 0.28144, 0.27958, 0.27783, 0.27617]
    self.K = [[interp(dt, dts, K0)], [interp(dt, dts, K1)]]


def laplacian_cdf(x, mu, b):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_cluster(v_ego, lead, clusters):
  # match vision point to best statistical cluster match
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_cdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_cdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_cdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This is isn't exactly right, but good heuristic
    return prob_d * prob_y * prob_v

  cluster = max(clusters, key=prob)

  # if no 'sane' match is found return None
  # stationary radar points can be false positives
  dist_sane = abs(cluster.dRel - offset_vision_dist) < max([(offset_vision_dist)*.25, 5.0])
  vel_sane = (abs(cluster.vRel + v_ego - lead.v[0]) < 10) or (v_ego + cluster.vRel > 3)
  if dist_sane and vel_sane:
    return cluster
  else:
    return None


def match_model_path_to_cluster(v_ego, md, clusters):
  if md is None or len(md.position.x) != TRAJECTORY_SIZE or md.position.x[-1] < LEAD_PATH_DREL_MIN:
    return None
  
  # take clusters that are
  # 1) closer than the farthest model predicted distance
  # 2) at least near the edge of regular op lead detectability
  # 3) close enough to the predicted path at the cluster distance
  close_path_clusters = [c for c in clusters if \
      c.dRel <= md.position.x[-1] and \
      c.dRel >= LEAD_PATH_DREL_MIN and \
      abs(-c.yRel - interp(c.dRel, md.position.x, md.position.y)) \
        <= interp(c.dRel, LEAD_PATH_YREL_MAX_BP, LEAD_PATH_YREL_MAX_V)]
  if len(close_path_clusters) == 0:
    return None

  # only care about the closest lead on our path
  cluster = min(close_path_clusters, key=lambda x: x.dRel)

  # if no 'sane' match is found return None
  # model path gets shorter when you brake, so can't use this for very slow leads
  vel_sane = (v_ego + cluster.vRel > 8)
  if vel_sane:
    return cluster
  else:
    return None


def match_model_lanelines_to_cluster(v_ego, md, lane_width, clusters):
  if md is None or lane_width < 2. or len(clusters) == 0 or len(md.laneLines) < 4 or len(md.laneLines[1].x) != TRAJECTORY_SIZE:
    return None
  
  # get centerline approximation using one or both lanelines
  
  ll_x = md.laneLines[1].x  # left and right ll x is the same
  lll_y = np.array(md.laneLines[1].y)
  rll_y = np.array(md.laneLines[2].y)
  l_prob = md.laneLineProbs[1]
  r_prob = md.laneLineProbs[2]
  lll_std = md.laneLineStds[1]
  rll_std = md.laneLineStds[2]

  if l_prob > MIN_LANE_PROB and r_prob > MIN_LANE_PROB:
    # Reduce reliance on lanelines that are too far apart or will be in a few seconds
    width_pts = rll_y - lll_y
    prob_mods = []
    for t_check in [0.0, 1.5, 3.0]:
      width_at_t = interp(t_check * (v_ego + 7), ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

  # Find path from lanes as the average center lane only if min probability on both lanes is above threshold.
  if l_prob > MIN_LANE_PROB and r_prob > MIN_LANE_PROB:
    c_y = (lll_y + rll_y) / 2.
  elif l_prob > MIN_LANE_PROB:
    c_y = lll_y + (lane_width / 2)
  elif r_prob > MIN_LANE_PROB:
    c_y = rll_y - (lane_width / 2)
  else:
    return None
  
  # take clusters that are
  # 1) closer than the farthest model predicted distance
  # 2) at least near the edge of regular op lead detectability
  # 3) close enough to the predicted path at the cluster distance
  close_path_clusters = [c for c in clusters if \
      c.dRel <= ll_x[-1] and \
      c.dRel >= LEAD_PATH_DREL_MIN and \
      abs(-c.yRel - interp(c.dRel, ll_x, c_y.tolist())) \
        <= interp(c.dRel, LEAD_PATH_YREL_MAX_BP, LEAD_PATH_YREL_MAX_V)]
  if len(close_path_clusters) == 0:
    return None

  # only care about the closest lead on our path
  cluster = min(close_path_clusters, key=lambda x: x.dRel)

  # if no 'sane' match is found return None
  # model path gets shorter when you brake, so can't use this for very slow leads
  vel_sane = (v_ego + cluster.vRel > -0.5)
  if vel_sane:
    return cluster
  else:
    return None


def get_lead(v_ego, ready, clusters, lead_msg=None, low_speed_override=True, md=None, lane_width=-1.):
  # Determine leads, this is where the essential logic happens
  if len(clusters) > 0 and ready and lead_msg is not None and lead_msg.prob > .5:
    cluster = match_vision_to_cluster(v_ego, lead_msg, clusters)
  else:
    cluster = None

  lead_dict = {'status': False, 'source': 'radar'}
  if cluster is not None:
    lead_dict = cluster.get_RadarState(lead_msg.prob, source='radar', checkSource='modelLead')
  elif (cluster is None) and ready and (lead_msg is not None and lead_msg.prob > .5):
    lead_dict = Cluster().get_RadarState_from_vision(lead_msg, v_ego)
  else:
    if md is not None:
      # if no model-verified lead, look for lead close to predicted path
      cluster = match_model_path_to_cluster(v_ego, md, clusters)
      if cluster is not None:
        lead_dict = cluster.get_RadarState(source='vision', checkSource='modelPath')
      else:
        # if path not long enough or no match, check again based on lanelines
        cluster = match_model_lanelines_to_cluster(v_ego, md, lane_width, clusters)
        if cluster is not None:
          lead_dict = cluster.get_RadarState(source='vision', checkSource='modelLaneLines')
        

  if low_speed_override:
    low_speed_clusters = [c for c in clusters if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_clusters) > 0:
      closest_cluster = min(low_speed_clusters, key=lambda c: c.dRel)

      # Only choose new cluster if it is actually closer than the previous one
      if (not lead_dict['status']) or (closest_cluster.dRel < lead_dict['dRel']):
        lead_dict = closest_cluster.get_RadarState(source=lead_dict['source'], checkSource='lowSpeedOverride')

  return lead_dict


class RadarD():
  def __init__(self, radar_ts, delay=0):
    self.current_time = 0

    self.tracks = defaultdict(dict)
    self.kalman_params = KalmanParams(radar_ts)

    # v_ego
    self.v_ego = 0.
    self.v_ego_hist = deque([0], maxlen=delay+1)
    
    self._params = Params()
    self.long_range_leads_enabled = self._params.get_bool("LongRangeLeadsEnabled")
    self.params_check_dur = 5. # [s]
    self.params_check_t = 0.
    self.dt = radar_ts

    self.ready = False

  def update(self, sm, rr, enable_lead):
    self.current_time = 1e-9*max(sm.logMonoTime.values())

    if sm.updated['carState']:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
    if sm.updated['modelV2']:
      self.ready = True

    ar_pts = {}
    for pt in rr.points:
      ar_pts[pt.trackId] = [pt.dRel, pt.yRel, pt.vRel, pt.measured]

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3])

    idens = list(sorted(self.tracks.keys()))
    track_pts = list([self.tracks[iden].get_key_for_cluster() for iden in idens])

    # If we have multiple points, cluster them
    if len(track_pts) > 1:
      cluster_idxs = cluster_points_centroid(track_pts, 2.5)
      clusters = [None] * (max(cluster_idxs) + 1)

      for idx in range(len(track_pts)):
        cluster_i = cluster_idxs[idx]
        if clusters[cluster_i] is None:
          clusters[cluster_i] = Cluster()
        clusters[cluster_i].add(self.tracks[idens[idx]])
    elif len(track_pts) == 1:
      # FIXME: cluster_point_centroid hangs forever if len(track_pts) == 1
      cluster_idxs = [0]
      clusters = [Cluster()]
      clusters[0].add(self.tracks[idens[0]])
    else:
      clusters = []

    # if a new point, reset accel to the rest of the cluster
    for idx in range(len(track_pts)):
      if self.tracks[idens[idx]].cnt <= 1:
        aLeadK = clusters[cluster_idxs[idx]].aLeadK
        aLeadTau = clusters[cluster_idxs[idx]].aLeadTau
        self.tracks[idens[idx]].reset_a_lead(aLeadK, aLeadTau)

    # *** publish radarState ***
    dat = messaging.new_message('radarState')
    dat.valid = sm.all_alive_and_valid() and len(rr.errors) == 0
    radarState = dat.radarState
    radarState.mdMonoTime = sm.logMonoTime['modelV2']
    radarState.canMonoTimes = list(rr.canMonoTimes)
    radarState.radarErrors = list(rr.errors)
    radarState.carStateMonoTime = sm.logMonoTime['carState']
    
    if self.params_check_t > self.params_check_dur:
      self.params_check_t = 0.
      self.long_range_leads_enabled = self._params.get_bool("LongRangeLeadsEnabled")
    self.params_check_t += self.dt

    if enable_lead:
      if len(sm['modelV2'].leadsV3) > 1:
        radarState.leadOne = get_lead(self.v_ego, self.ready, clusters, sm['modelV2'].leadsV3[0], low_speed_override=True, \
                  md=sm['modelV2'] if self.long_range_leads_enabled else None, lane_width=sm['lateralPlan'].laneWidth if self.long_range_leads_enabled else None)
        radarState.leadTwo = get_lead(self.v_ego, self.ready, clusters, sm['modelV2'].leadsV3[1], low_speed_override=False, \
                  md=sm['modelV2'] if self.long_range_leads_enabled else None, lane_width=sm['lateralPlan'].laneWidth if self.long_range_leads_enabled else None)
      elif self.long_range_leads_enabled:
        radarState.leadOne = get_lead(self.v_ego, self.ready, clusters, lead_msg=None, low_speed_override=True, md=sm['modelV2'], lane_width=sm['lateralPlan'].laneWidth)
        radarState.leadTwo = get_lead(self.v_ego, self.ready, clusters, lead_msg=None, low_speed_override=False, md=sm['modelV2'], lane_width=sm['lateralPlan'].laneWidth)
    return dat


# fuses camera and radar data for best lead detection
def radard_thread(sm=None, pm=None, can_sock=None):
  config_realtime_process(5 if TICI else 2, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = car.CarParams.from_bytes(Params().get("CarParams", block=True))
  cloudlog.info("radard got CarParams")

  # import the radar from the fingerprint
  cloudlog.info("radard is importing %s", CP.carName)
  RadarInterface = importlib.import_module('selfdrive.car.%s.radar_interface' % CP.carName).RadarInterface

  # *** setup messaging
  if can_sock is None:
    can_sock = messaging.sub_sock('can')
  if sm is None:
    sm = messaging.SubMaster(['modelV2', 'carState','lateralPlan'], ignore_avg_freq=['modelV2', 'carState','lateralPlan'])  # Can't check average frequency, since radar determines timing
  if pm is None:
    pm = messaging.PubMaster(['radarState', 'liveTracks'])

  RI = RadarInterface(CP)

  rk = Ratekeeper(1.0 / CP.radarTimeStep, print_delay_threshold=None)
  RD = RadarD(CP.radarTimeStep, RI.delay)

  # TODO: always log leads once we can hide them conditionally
  enable_lead = CP.openpilotLongitudinalControl or not CP.radarOffCan

  while 1:
    can_strings = messaging.drain_sock_raw(can_sock, wait_for_one=True)
    rr = RI.update(can_strings)

    if rr is None:
      continue

    sm.update(0)

    dat = RD.update(sm, rr, enable_lead)
    dat.radarState.cumLagMs = -rk.remaining*1000.

    pm.send('radarState', dat)

    # *** publish tracks for UI debugging (keep last) ***
    tracks = RD.tracks
    dat = messaging.new_message('liveTracks', len(tracks))

    for cnt, ids in enumerate(sorted(tracks.keys())):
      dat.liveTracks[cnt] = {
        "trackId": ids,
        "dRel": float(tracks[ids].dRel),
        "yRel": float(tracks[ids].yRel),
        "vRel": float(tracks[ids].vRel),
      }
    pm.send('liveTracks', dat)

    rk.monitor_time()


def main(sm=None, pm=None, can_sock=None):
  radard_thread(sm, pm, can_sock)


if __name__ == "__main__":
  main()
