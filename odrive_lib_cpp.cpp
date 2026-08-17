#include "odrive_lib_cpp.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <utility>

namespace odrive {

namespace {

/* Best-effort dotted property paths for fw 0.6.x -- verify against
 * `odrivetool` (tab-complete on `odrv0.`) if a getter comes back wrong. */
constexpr const char *kIqSetpointProperty     = "motor.foc.Iq_setpoint";
constexpr const char *kIqMeasuredProperty     = "motor.foc.Iq_measured";
constexpr const char *kFetTempProperty        = "motor.fet_thermistor.temperature";
constexpr const char *kMotorTempProperty      = "motor.motor_thermistor.temperature";
constexpr const char *kTorqueSetpointProperty = "controller.torque_setpoint";
constexpr const char *kTorqueEstimateProperty = "motor.torque_estimate";
constexpr const char *kElectricalPowerProperty = "motor.electrical_power";
constexpr const char *kMechanicalPowerProperty = "motor.mechanical_power";
constexpr const char *kVbusVoltageProperty    = "vbus_voltage";
constexpr const char *kIbusProperty           = "ibus";
constexpr const char *kFwVersionMajorProperty = "fw_version_major";
constexpr const char *kFwVersionMinorProperty = "fw_version_minor";
constexpr const char *kFwVersionRevisionProperty = "fw_version_revision";
constexpr const char *kHwVersionMajorProperty = "hw_version_major";
constexpr const char *kHwVersionMinorProperty = "hw_version_minor";
constexpr const char *kHwVersionVariantProperty = "hw_version_variant";

std::string format_float(float v)
{
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.6f", v);
    return buf;
}

std::string build_cmd(const char *letter, uint8_t axis, std::initializer_list<float> args)
{
    std::string s = std::string(letter) + " " + std::to_string(axis);
    for (float a : args) {
        s += " ";
        s += format_float(a);
    }
    return s;
}

LogFn &log_sink()
{
    static LogFn fn;
    return fn;
}

} // namespace

/* ---- strings ---- */

const char *axis_state_str(uint8_t state)
{
    switch (static_cast<AxisState>(state)) {
    case AxisState::Undefined:                return "UNDEFINED";
    case AxisState::Idle:                     return "IDLE";
    case AxisState::StartupSequence:          return "STARTUP_SEQUENCE";
    case AxisState::FullCalibration:          return "FULL_CALIBRATION_SEQUENCE";
    case AxisState::MotorCalibration:         return "MOTOR_CALIBRATION";
    case AxisState::EncoderIndexSearch:       return "ENCODER_INDEX_SEARCH";
    case AxisState::EncoderOffsetCalib:       return "ENCODER_OFFSET_CALIBRATION";
    case AxisState::ClosedLoopControl:        return "CLOSED_LOOP_CONTROL";
    case AxisState::LockinSpin:               return "LOCKIN_SPIN";
    case AxisState::EncoderDirFind:           return "ENCODER_DIR_FIND";
    case AxisState::Homing:                   return "HOMING";
    case AxisState::EncoderHallPolarityCalib: return "ENCODER_HALL_POLARITY_CALIBRATION";
    case AxisState::EncoderHallPhaseCalib:    return "ENCODER_HALL_PHASE_CALIBRATION";
    case AxisState::AnticoggingCalib:         return "ANTICOGGING_CALIBRATION";
    case AxisState::HarmonicCalib:            return "HARMONIC_CALIBRATION";
    case AxisState::HarmonicCalibCommutation: return "HARMONIC_CALIBRATION_COMMUTATION";
    default:                                  return "UNKNOWN";
    }
}

const char *procedure_result_str(uint8_t result)
{
    switch (static_cast<ProcedureResult>(result)) {
    case ProcedureResult::Success:                  return "SUCCESS";
    case ProcedureResult::Busy:                      return "BUSY";
    case ProcedureResult::Cancelled:                 return "CANCELLED";
    case ProcedureResult::Disarmed:                  return "DISARMED";
    case ProcedureResult::NoResponse:                return "NO_RESPONSE";
    case ProcedureResult::PolePairCprMismatch:       return "POLE_PAIR_CPR_MISMATCH";
    case ProcedureResult::PhaseResistanceOutOfRange: return "PHASE_RESISTANCE_OUT_OF_RANGE";
    case ProcedureResult::PhaseInductanceOutOfRange: return "PHASE_INDUCTANCE_OUT_OF_RANGE";
    case ProcedureResult::UnbalancedPhases:          return "UNBALANCED_PHASES";
    case ProcedureResult::InvalidMotorType:          return "INVALID_MOTOR_TYPE";
    case ProcedureResult::IllegalHallState:          return "ILLEGAL_HALL_STATE";
    case ProcedureResult::Timeout:                   return "TIMEOUT";
    case ProcedureResult::HomingWithoutEndstop:      return "HOMING_WITHOUT_ENDSTOP";
    case ProcedureResult::InvalidState:              return "INVALID_STATE";
    case ProcedureResult::NotCalibrated:             return "NOT_CALIBRATED";
    case ProcedureResult::NotConverging:             return "NOT_CONVERGING";
    case ProcedureResult::RequestedCurrentTooHigh:   return "REQUESTED_CURRENT_TOO_HIGH";
    default:                                         return "UNKNOWN";
    }
}

const char *control_mode_str(uint8_t mode)
{
    switch (static_cast<ControlMode>(mode)) {
    case ControlMode::Voltage:  return "VOLTAGE_CONTROL";
    case ControlMode::Torque:   return "TORQUE_CONTROL";
    case ControlMode::Velocity: return "VELOCITY_CONTROL";
    case ControlMode::Position: return "POSITION_CONTROL";
    default:                    return "UNKNOWN";
    }
}

const char *input_mode_str(uint8_t mode)
{
    switch (static_cast<InputMode>(mode)) {
    case InputMode::Inactive:    return "INACTIVE";
    case InputMode::Passthrough: return "PASSTHROUGH";
    case InputMode::VelRamp:     return "VEL_RAMP";
    case InputMode::PosFilter:   return "POS_FILTER";
    case InputMode::MixChannels: return "MIX_CHANNELS";
    case InputMode::TrapTraj:    return "TRAP_TRAJ";
    case InputMode::TorqueRamp:  return "TORQUE_RAMP";
    case InputMode::Mirror:      return "MIRROR";
    case InputMode::Tuning:      return "TUNING";
    default:                     return "UNKNOWN";
    }
}

int error_str(uint32_t err, char *buf, size_t buf_len)
{
    static const struct { uint32_t mask; const char *name; } tbl[] = {
        { ERROR_INITIALIZING,              "INITIALIZING" },
        { ERROR_SYSTEM_LEVEL,              "SYSTEM_LEVEL" },
        { ERROR_TIMING_ERROR,              "TIMING_ERROR" },
        { ERROR_MISSING_ESTIMATE,          "MISSING_ESTIMATE" },
        { ERROR_BAD_CONFIG,                "BAD_CONFIG" },
        { ERROR_DRV_FAULT,                 "DRV_FAULT" },
        { ERROR_MISSING_INPUT,             "MISSING_INPUT" },
        { ERROR_DC_BUS_OVER_VOLTAGE,       "DC_BUS_OVER_VOLTAGE" },
        { ERROR_DC_BUS_UNDER_VOLTAGE,      "DC_BUS_UNDER_VOLTAGE" },
        { ERROR_DC_BUS_OVER_CURRENT,       "DC_BUS_OVER_CURRENT" },
        { ERROR_DC_BUS_OVER_REGEN_CURRENT, "DC_BUS_OVER_REGEN_CURRENT" },
        { ERROR_CURRENT_LIMIT_VIOLATION,   "CURRENT_LIMIT_VIOLATION" },
        { ERROR_MOTOR_OVER_TEMP,           "MOTOR_OVER_TEMP" },
        { ERROR_INVERTER_OVER_TEMP,        "INVERTER_OVER_TEMP" },
        { ERROR_VELOCITY_LIMIT_VIOLATION,  "VELOCITY_LIMIT_VIOLATION" },
        { ERROR_POSITION_LIMIT_VIOLATION,  "POSITION_LIMIT_VIOLATION" },
        { ERROR_REQUESTED_CURRENT_TOO_HIGH,"REQUESTED_CURRENT_TOO_HIGH" },
        { ERROR_WATCHDOG_TIMER_EXPIRED,    "WATCHDOG_TIMER_EXPIRED" },
        { ERROR_ESTOP_REQUESTED,           "ESTOP_REQUESTED" },
        { ERROR_SPINOUT_DETECTED,          "SPINOUT_DETECTED" },
        { ERROR_BRAKE_RESISTOR_DISARMED,   "BRAKE_RESISTOR_DISARMED" },
        { ERROR_THERMISTOR_DISCONNECTED,   "THERMISTOR_DISCONNECTED" },
        { ERROR_CALIBRATION_ERROR,         "CALIBRATION_ERROR" },
    };
    if (!buf || buf_len == 0) return 0;
    if (err == 0u) return std::snprintf(buf, buf_len, "none");

    int total = 0;
    uint32_t seen = 0u;
    size_t n = sizeof(tbl) / sizeof(tbl[0]);
    for (size_t i = 0; i < n; ++i) {
        if (err & tbl[i].mask) {
            size_t off = ((size_t)total < buf_len) ? (size_t)total : buf_len - 1;
            total += std::snprintf(buf + off, buf_len - off, "%s%s",
                                    total ? "|" : "", tbl[i].name);
            seen |= tbl[i].mask;
        }
    }
    uint32_t rem = err & ~seen;
    if (rem) {
        size_t off = ((size_t)total < buf_len) ? (size_t)total : buf_len - 1;
        total += std::snprintf(buf + off, buf_len - off, "%s0x%lX",
                                total ? "|" : "", (unsigned long)rem);
    }
    return total;
}

int heartbeat_str(const Heartbeat &hb, char *buf, size_t buf_len)
{
    if (!buf || buf_len == 0) return 0;
    int total = std::snprintf(buf, buf_len,
                               "state=%s(%u) proc=%s(%u) traj_done=%u error=",
                               axis_state_str(hb.axis_state), (unsigned)hb.axis_state,
                               procedure_result_str(hb.procedure_result), (unsigned)hb.procedure_result,
                               (unsigned)hb.trajectory_done_flag);
    if (total < 0) return total;
    size_t off = ((size_t)total < buf_len) ? (size_t)total : buf_len - 1;
    total += error_str(hb.axis_error, buf + off, buf_len - off);
    return total;
}

/* ---- logger ---- */

void attach_logger(LogFn fn)
{
    log_sink() = std::move(fn);
}

/* ---- Bus ---- */

Bus::Bus(SerialWorker &worker) : worker_(worker)
{
    worker_.set_terminators("\n", "\n");
}

Status Bus::send_line_locked(const std::string &line)
{
    if (!worker_.isOpen()) return Status::ErrNotConnected;
    worker_.sendData(line);
    return Status::Ok;
}

Status Bus::send_line(const std::string &line)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return send_line_locked(line);
}

Status Bus::request(const std::string &line, ReplyHandler on_reply)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Status s = send_line_locked(line);
    if (s != Status::Ok) return s;
    pending_.push_back({ std::move(on_reply), std::chrono::steady_clock::now() });
    return Status::Ok;
}

void Bus::set_reply_timeout(int timeout_ms) { reply_timeout_ms_ = timeout_ms; }

void Bus::poll()
{
    for (;;) {
        std::string line = worker_.get_data();
        if (line.empty()) break;

        ReplyHandler handler;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (pending_.empty()) continue; /* unsolicited line; discard */
            handler = std::move(pending_.front().handler);
            pending_.pop_front();
        }
        if (handler) handler(line);
    }

    auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!pending_.empty() &&
               now - pending_.front().issued_at > std::chrono::milliseconds(reply_timeout_ms_)) {
            pending_.pop_front();
            // if (log_sink()) log_sink()("bus: dropped stale pending reply (no response within timeout)");
        }
    }

    std::vector<Axis *> axes_copy;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        axes_copy = axes_;
    }
    for (Axis *axis : axes_copy) axis->poll_periodic(now);
}

Status Bus::clear_errors() { return send_line("sc"); }
Status Bus::save_config()  { return send_line("ss"); }
Status Bus::erase_config() { return send_line("se"); }
Status Bus::reboot()       { return send_line("sr"); }

void Bus::register_axis(Axis *axis)
{
    std::lock_guard<std::mutex> lock(mutex_);
    axes_.push_back(axis);
}

void Bus::unregister_axis(Axis *axis)
{
    std::lock_guard<std::mutex> lock(mutex_);
    axes_.erase(std::remove(axes_.begin(), axes_.end(), axis), axes_.end());
}

/* ---- Axis ---- */

Axis::Axis(Bus &bus, uint8_t axis_index, const std::string &name)
    : bus_(bus), axis_index_(axis_index), name_(name)
{
    bus_.register_axis(this);
}

Axis::~Axis()
{
    bus_.unregister_axis(this);
}

std::string Axis::prefix() const
{
    return "axis" + std::to_string(axis_index_) + ".";
}

void Axis::logf(const char *fmt, ...)
{
    if (!log_sink() || !log_enabled_) return;
    char buf[192];
    int n = 0;
    if (!name_.empty()) {
        n = std::snprintf(buf, sizeof buf, "%s: ", name_.c_str());
        if (n < 0 || (size_t)n >= sizeof buf) n = 0;
    }
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, ap);
    va_end(ap);
    log_sink()(buf);
}

void Axis::set_conversion(float turns_per_unit, bool invert)
{
    turns_per_unit_ = (turns_per_unit == 0.0f) ? (invert ? -1.0f : 1.0f)
                                                : (invert ? -turns_per_unit : turns_per_unit);
}

void Axis::enable_logging(bool enable) { log_enabled_ = enable; }

/* ---- setpoints ---- */

Status Axis::set_input_pos(float pos, float vel_ff, float torque_ff)
{
    logf("input_pos %.3f (vff=%.3f tff=%.3f)", pos, vel_ff, torque_ff);
    return bus_.send_line(build_cmd("p", axis_index_,
                                     { pos * turns_per_unit_, vel_ff * turns_per_unit_, torque_ff }));
}

Status Axis::set_input_vel(float vel, float torque_ff)
{
    logf("input_vel %.3f (tff=%.3f)", vel, torque_ff);
    return bus_.send_line(build_cmd("v", axis_index_, { vel * turns_per_unit_, torque_ff }));
}

Status Axis::set_input_torque(float torque)
{
    logf("input_torque %.3f", torque);
    return bus_.send_line(build_cmd("c", axis_index_, { torque }));
}

Status Axis::set_absolute_position(float pos)
{
    logf("absolute_position %.3f", pos);
    return write_property(prefix() + "pos_estimate ",
                           format_float(pos));
    // return bus_.send_line(build_cmd("es", axis_index_, { pos }));
}

Status Axis::set_relative_pos(float delta)
{
    if (!pos_valid_) {
        logf("relative_pos rejected: no position feedback yet");
        return Status::ErrBadArg;
    }
    logf("relative_pos %.3f", delta);
    return set_input_pos(feedback.pos_estimate + delta, 0.0f, 0.0f);
}

/* ---- control/config ---- */

Status Axis::set_axis_state(AxisState state)
{
    logf("axis_state %u", (unsigned)state);
    return write_property(prefix() + "requested_state", std::to_string((unsigned)state));
}

Status Axis::set_closed_loop(bool enable)
{
    return set_axis_state(enable ? AxisState::ClosedLoopControl : AxisState::Idle);
}

Status Axis::set_controller_mode(ControlMode control_mode, InputMode input_mode)
{
    logf("controller_mode ctrl=%u input=%u", (unsigned)control_mode, (unsigned)input_mode);
    Status s1 = write_property(prefix() + "controller.config.control_mode",
                                std::to_string((unsigned)control_mode));
    Status s2 = write_property(prefix() + "controller.config.input_mode",
                                std::to_string((unsigned)input_mode));
    return (s1 != Status::Ok) ? s1 : s2;
}

Status Axis::set_limits(float vel_limit, float current_limit)
{
    logf("limits vel=%.3f cur=%.3f", vel_limit, current_limit);
    Status s1 = write_property(prefix() + "controller.config.vel_limit",
                                format_float(std::fabs(vel_limit * turns_per_unit_)));
    Status s2 = write_property(prefix() + "motor.config.current_lim",
                                format_float(current_limit));
    return (s1 != Status::Ok) ? s1 : s2;
}

Status Axis::set_traj_vel_limit(float vel_limit)
{
    logf("traj_vel_limit %.3f", vel_limit);
    return write_property(prefix() + "trap_traj.config.vel_limit ",
                           format_float(std::fabs(vel_limit * turns_per_unit_)));
}

Status Axis::set_traj_accel_limits(float accel, float decel)
{
    logf("traj_accel_limits accel=%.3f decel=%.3f", accel, decel);
    Status s1 = write_property(prefix() + "trap_traj.config.accel_limit",
                                format_float(std::fabs(accel * turns_per_unit_)));
    Status s2 = write_property(prefix() + "trap_traj.config.decel_limit",
                                format_float(std::fabs(decel * turns_per_unit_)));
    return (s1 != Status::Ok) ? s1 : s2;
}

Status Axis::set_defaults(float vel_limit, float accel, float decel)
{
    default_vel_limit_ = vel_limit;
    default_accel_ = accel;
    default_decel_ = decel;
    logf("defaults vel=%.3f accel=%.3f decel=%.3f", vel_limit, accel, decel);
    return restore_defaults();
}

Status Axis::restore_defaults()
{
    Status rc = set_traj_vel_limit(default_vel_limit_);
    Status rc2 = set_traj_accel_limits(default_accel_, default_decel_);
    return (rc != Status::Ok) ? rc : rc2;
}

Status Axis::clear_errors()
{
    logf("clear_errors");
    return bus_.clear_errors();
}

Status Axis::estop()
{
    logf("estop (best-effort: requested_state=IDLE; ASCII protocol has no hard e-stop)");
    return write_property(prefix() + "requested_state", std::to_string((unsigned)AxisState::Idle));
}

Status Axis::reboot()
{
    logf("reboot");
    return bus_.reboot();
}

Status Axis::save_config()
{
    logf("save_config");
    return bus_.save_config();
}

Status Axis::erase_config()
{
    logf("erase_config");
    return bus_.erase_config();
}

/* ---- generic property access ---- */

Status Axis::write_property(const std::string &path, const std::string &value)
{
    return bus_.send_line("w " + path + " " + value);
}

Status Axis::read_property(const std::string &path, PropertyHandler on_value)
{
    return request_many({ path }, [on_value](const std::vector<std::string> &v) {
        if (on_value) on_value(v[0]);
    });
}

/* Sends one "r <path>" per entry back-to-back; combine runs once every
 * reply is dispatched, in path order regardless of arrival order. */
Status Axis::request_many(std::vector<std::string> paths, PropertiesHandler combine)
{
    auto count = paths.size();
    auto state = std::make_shared<std::pair<std::vector<std::string>, size_t>>(
        std::vector<std::string>(count), size_t(0));

    Status rc = Status::Ok;
    for (size_t i = 0; i < count; ++i) {
        std::string path = paths[i];
        Status s = bus_.request("r " + path,
            [this, state, path, i, count, combine](const std::string &line) {
                feedback.last_property_path = path;
                feedback.last_property_value = line;
                if (cb_property_) cb_property_(*this);
                state->first[i] = line;
                if (++state->second == count) combine(state->first);
            });
        if (s != Status::Ok && rc == Status::Ok) rc = s;
    }
    return rc;
}

/* ---- getters ---- */

Status Axis::request_encoder()
{
    return bus_.request("f " + std::to_string(axis_index_), [this](const std::string &line) {
        float pos = 0.0f, vel = 0.0f;
        if (std::sscanf(line.c_str(), "%f %f", &pos, &vel) != 2) return;
        feedback.pos_estimate = pos / turns_per_unit_;
        feedback.vel_estimate = vel / turns_per_unit_;
        pos_valid_ = true;
        if (cb_encoder_) cb_encoder_(*this);
    });
}

Status Axis::request_iq()
{
    return request_many({ prefix() + kIqSetpointProperty, prefix() + kIqMeasuredProperty },
        [this](const std::vector<std::string> &v) {
            feedback.iq_setpoint = std::strtof(v[0].c_str(), nullptr);
            feedback.iq_measured = std::strtof(v[1].c_str(), nullptr);
            if (cb_iq_) cb_iq_(*this);
        });
}

Status Axis::request_temperature()
{
    return request_many({ prefix() + kFetTempProperty, prefix() + kMotorTempProperty },
        [this](const std::vector<std::string> &v) {
            feedback.fet_temperature = std::strtof(v[0].c_str(), nullptr);
            feedback.motor_temperature = std::strtof(v[1].c_str(), nullptr);
            if (cb_temperature_) cb_temperature_(*this);
        });
}

Status Axis::request_bus_vi()
{
    return request_many({ kVbusVoltageProperty, kIbusProperty },
        [this](const std::vector<std::string> &v) {
            feedback.bus_voltage = std::strtof(v[0].c_str(), nullptr);
            feedback.bus_current = std::strtof(v[1].c_str(), nullptr);
            if (cb_bus_vi_) cb_bus_vi_(*this);
        });
}

Status Axis::request_torques()
{
    return request_many({ prefix() + kTorqueSetpointProperty, prefix() + kTorqueEstimateProperty },
        [this](const std::vector<std::string> &v) {
            feedback.torque_target = std::strtof(v[0].c_str(), nullptr);
            feedback.torque_estimate = std::strtof(v[1].c_str(), nullptr);
            if (cb_torques_) cb_torques_(*this);
        });
}

Status Axis::request_powers()
{
    return request_many({ prefix() + kElectricalPowerProperty, prefix() + kMechanicalPowerProperty },
        [this](const std::vector<std::string> &v) {
            feedback.electrical_power = std::strtof(v[0].c_str(), nullptr);
            feedback.mechanical_power = std::strtof(v[1].c_str(), nullptr);
            if (cb_powers_) cb_powers_(*this);
        });
}

Status Axis::request_error()
{
    return request_many({ prefix() + "active_errors", prefix() + "disarm_reason" },
        [this](const std::vector<std::string> &v) {
            feedback.active_errors = (uint32_t)std::strtoul(v[0].c_str(), nullptr, 0);
            feedback.disarm_reason = (uint32_t)std::strtoul(v[1].c_str(), nullptr, 0);
            if (cb_error_) cb_error_(*this);
        });
}

Status Axis::request_version()
{
    return request_many({ kFwVersionMajorProperty, kFwVersionMinorProperty, kFwVersionRevisionProperty,
                           kHwVersionMajorProperty, kHwVersionMinorProperty, kHwVersionVariantProperty },
        [this](const std::vector<std::string> &v) {
            feedback.fw_version_major    = (uint8_t)std::strtoul(v[0].c_str(), nullptr, 0);
            feedback.fw_version_minor    = (uint8_t)std::strtoul(v[1].c_str(), nullptr, 0);
            feedback.fw_version_revision = (uint8_t)std::strtoul(v[2].c_str(), nullptr, 0);
            feedback.hw_version_major    = (uint8_t)std::strtoul(v[3].c_str(), nullptr, 0);
            feedback.hw_version_minor    = (uint8_t)std::strtoul(v[4].c_str(), nullptr, 0);
            feedback.hw_version_variant  = (uint8_t)std::strtoul(v[5].c_str(), nullptr, 0);
            if (cb_version_) cb_version_(*this);
        });
}

Status Axis::poll_heartbeat()
{
    return request_many({ prefix() + "current_state", prefix() + "procedure_result",
                           prefix() + "controller.trajectory_done", prefix() + "active_errors" },
        [this](const std::vector<std::string> &v) {
            uint8_t n_st = (uint8_t)std::strtoul(v[0].c_str(), nullptr, 0);
            uint8_t n_proc = (uint8_t)std::strtoul(v[1].c_str(), nullptr, 0);
            uint8_t n_traj = (v[2] == "1" || v[2] == "true") ? 1u : 0u;
            uint32_t n_err = (uint32_t)std::strtoul(v[3].c_str(), nullptr, 0);

            if (log_sink() && log_enabled_) {
                std::string msg;
                if (n_st != feedback.hb.axis_state) {
                    msg += (msg.empty() ? "" : " ");
                    msg += "axis_state changed: " + std::string(axis_state_str(feedback.hb.axis_state))
                         + " -> " + std::string(axis_state_str(n_st));
                }
                if (n_proc != feedback.hb.procedure_result) {
                    msg += (msg.empty() ? "" : " ");
                    msg += "procedure_result=" + std::string(procedure_result_str(n_proc)) + "(" + std::to_string(n_proc) + ")";
                }
                if (n_err != feedback.hb.axis_error) {
                    char eb[96];
                    error_str(n_err, eb, sizeof eb);
                    msg += (msg.empty() ? "" : " ");
                    msg += "axis_error=" + std::string(eb);
                }
                if (n_traj != feedback.hb.trajectory_done_flag) {
                    msg += (msg.empty() ? "" : " ");
                    msg += "trajectory_done=" + std::to_string(n_traj);
                }
                if (!msg.empty()) logf("%s", msg.c_str());
            }

            feedback.hb.axis_state = n_st;
            feedback.hb.procedure_result = n_proc;
            feedback.hb.trajectory_done_flag = n_traj;
            feedback.hb.axis_error = n_err;

            if (cb_heartbeat_) cb_heartbeat_(*this);
        });
}

/* ---- host-side emulated periodic message rates ---- */

Status Axis::set_msg_rate(MsgRate msg, uint32_t rate_ms)
{
    size_t idx = static_cast<size_t>(msg);
    if (idx >= static_cast<size_t>(MsgRate::Count)) return Status::ErrBadArg;
    logf("msg_rate[%d]=%u ms", (int)idx, (unsigned)rate_ms);
    msg_rate_ms_[idx] = rate_ms;
    return Status::Ok;
}

Status Axis::set_all_msg_rates(uint32_t rate_ms)
{
    logf("set_all_msg_rates=%u ms", (unsigned)rate_ms);
    msg_rate_ms_.fill(rate_ms);
    return Status::Ok;
}

void Axis::poll_periodic(std::chrono::steady_clock::time_point now)
{
    for (size_t i = 0; i < static_cast<size_t>(MsgRate::Count); ++i) {
        uint32_t rate = msg_rate_ms_[i];
        if (rate == 0 || now < next_due_[i]) continue;
        next_due_[i] = now + std::chrono::milliseconds(rate);

        switch (static_cast<MsgRate>(i)) {
        case MsgRate::Version:     request_version(); break;
        case MsgRate::Heartbeat:   poll_heartbeat(); break;
        case MsgRate::Encoder:     request_encoder(); break;
        case MsgRate::Iq:          request_iq(); break;
        case MsgRate::Error:       request_error(); break;
        case MsgRate::Temperature: request_temperature(); break;
        case MsgRate::BusVoltage:  request_bus_vi(); break;
        case MsgRate::Torques:     request_torques(); break;
        case MsgRate::Powers:      request_powers(); break;
        default: break;
        }
    }
}

/* ---- callback registration ---- */

void Axis::on_heartbeat(AxisCallback fn)   { cb_heartbeat_ = std::move(fn); }
void Axis::on_encoder(AxisCallback fn)     { cb_encoder_ = std::move(fn); }
void Axis::on_iq(AxisCallback fn)          { cb_iq_ = std::move(fn); }
void Axis::on_temperature(AxisCallback fn) { cb_temperature_ = std::move(fn); }
void Axis::on_bus_vi(AxisCallback fn)      { cb_bus_vi_ = std::move(fn); }
void Axis::on_torques(AxisCallback fn)     { cb_torques_ = std::move(fn); }
void Axis::on_powers(AxisCallback fn)      { cb_powers_ = std::move(fn); }
void Axis::on_error(AxisCallback fn)       { cb_error_ = std::move(fn); }
void Axis::on_version(AxisCallback fn)     { cb_version_ = std::move(fn); }
void Axis::on_property(AxisCallback fn)    { cb_property_ = std::move(fn); }

} // namespace odrive
