// JSON shapes sent by teleop-host (host/src/teleop_host/server.py), which in
// turn mirror the host link messages (docs/host-protocol.md).

export interface JointInfo {
  servo_id: number;
  model: number; // 1 SCS0009, 2 HLS3606
  travel_min: number;
  travel_max: number;
  vmax: number;
  vsat: number;
  amax: number;
  max_speed: number;
  encoder_default: number;
  enabled_default: number;
  step_scale: number;
  degrees_per_step: number;
  name: string;
}

export interface DeviceInfo {
  protocol: number;
  firmware: string;
  control_hz: number;
  degrees_per_count: number;
  encoder_count: number;
  min_current_limit_ma: number;
  max_current_limit_ma: number;
  min_current_trip_pct: number;
  max_current_trip_pct: number;
  default_current_trip_pct: number;
  min_exo_span_counts: number;
  max_exo_span_counts: number;
  joints: JointInfo[];
  faults: string[];
  states: string[];
  events: string[];
  models: string[];
}

export interface TelemetryJoint {
  state: number;
  fault: number;
  flags: number;
  enc_raw: number;
  exo_counts: number;
  exo_filtered_counts: number;
  target: number;
  goal: number;
  sent_goal: number;
  reference: number;
  speed_command: number;
  grip_lead: number;
  position: number;
  speed: number;
  load_deci_pct: number;
  current_ma: number;
  voltage_deci_v: number;
  temperature_c: number;
  status: number;
  error_byte: number;
  read_age_ms: number;
}

export interface Telemetry {
  cycle: number;
  t_us: number;
  uptime_ms: number;
  armed_count: number;
  hand_fault: number;
  hand_flags: number;
  exec_us: number;
  joints: TelemetryJoint[];
}

export interface SlowJoint {
  enabled: number;
  encoder_index: number;
  exo_ref: number;
  exo_b: number;
  servo_a: number;
  servo_b: number;
  vmax: number;
  amax: number;
  vsat: number;
  track_steps: number;
  grip_steps: number;
  current_limit_ma: number;
  current_trip_pct: number;
  detected_model: number;
  effective_current_ma: number;
  jump_rejects: number;
}

export interface SlowEncoder {
  address: number;
  state: number; // 0 online, 1 no magnet, 2 offline
  valid: number;
  angle: number;
  agc: number;
  magnet_ok: number;
  errors: number;
}

export interface SlowStatus {
  lead_ms: number;
  min_cutoff_hz: number;
  beta: number;
  jump_deg: number;
  grip_load_pct: number;
  load_trip: number;
  cycles: number;
  overruns: number;
  last_period_us: number;
  max_jitter_us: number;
  p99_jitter_us: number;
  last_exec_us: number;
  max_exec_us: number;
  average_exec_us: number;
  bus_sent: number;
  bus_received: number;
  bus_timeouts: number;
  bus_parse_errors: number;
  bus_echo_errors: number;
  bus_last_reply_us: number;
  bus_max_reply_us: number;
  bus_echo: number;
  encoder_average_us: number;
  encoder_max_us: number;
  encoder_max_skew_us: number;
  dropped_events: number;
  dropped_telemetry: number;
  rx_crc_errors: number;
  rx_malformed: number;
  monitor: number;
  monitor_ok_mask: number;
  monitor_position: number[];
  joints: SlowJoint[];
  encoders: SlowEncoder[];
}

export interface DeviceEvent {
  code: number;
  severity: number; // 0 info, 1 warning, 2 error
  joint: number; // servo id, 0 = none
  cmd_seq: number;
  args: [number, number, number, number];
  text: string;
}

export interface LinkInfo {
  state: 'disconnected' | 'connecting' | 'connected' | 'error';
  port: string;
  detail: string;
  firmware: string;
}

export interface CommandResult {
  text: string;
  ok: boolean;
  status: string; // ok | failed | parse_error | busy | bad_frame | timeout | not_connected
  error: string;
  events: DeviceEvent[];
  logs: string[];
  config: string | null;
}

export interface PluginInfo {
  id: string;
  title: string;
  description: string;
  status: 'planned' | 'active';
}

export interface PortInfo {
  device: string;
  description: string;
  is_esp32: boolean;
}

export const NO_ENCODER = 0xff;
export const MODEL_SCS = 1;
export const MODEL_HLS = 2;

export const JointFlag = {
  ENABLED: 1 << 0,
  READY: 1 << 1,
  REF_CAPTURED: 1 << 2,
  HOMED: 1 << 3,
  SINGLE_TURN: 1 << 4,
  ENC_FRESH: 1 << 5,
  CONTACT: 1 << 6,
  GOAL_SENT: 1 << 7,
  MODEL_OK: 1 << 8,
  SERVO_OK: 1 << 9,
  TORQUE_ON: 1 << 10,
  MOVING: 1 << 11,
  HAS_CURRENT: 1 << 12,
  SERVO_FRESH: 1 << 13,
} as const;

export const HandFlag = {
  TORQUE_OFF_CONFIRMED: 1 << 0,
  MONITOR: 1 << 1,
  SWEEP_ACTIVE: 1 << 2,
  TELEMETRY_DROPPED: 1 << 3,
} as const;

export const EventCode = {
  GENERIC: 0,
  REFUSED: 1,
  ARMED: 2,
  DISARMED: 3,
  TORQUE_OFF: 4,
  FAULT: 5,
  JOINT_STATE: 6,
  CAL_CAPTURED: 7,
  HOMED: 8,
  PING_RESULT: 9,
  LIMITS_ROW: 10,
  SWEEP_RUN: 11,
  SWEEP_DONE: 12,
  CONFIG_SAVED: 13,
  CONFIG_LOADED: 14,
  DEFAULTS: 15,
  SETTING_CHANGED: 16,
  MONITOR: 17,
} as const;

export const JointState = { OFF: 0, ALIGN: 1, FOLLOW: 2, HOLD: 3, SWEEP: 4 } as const;

export const has = (flags: number, flag: number): boolean => (flags & flag) !== 0;
