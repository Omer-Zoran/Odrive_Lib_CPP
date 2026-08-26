/* Controls an ODrive (fw 0.6.x) over ASCII serial via SerialWorker. Every
 * call is non-blocking; getters only send the request -- results land in
 * `feedback` once Bus::poll() dispatches the reply. */
#ifndef ODRIVE_LIB_CPP_H
#define ODRIVE_LIB_CPP_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "SerialWorker.h"

namespace odrive {

enum class Status {
    Ok = 0,
    ErrBadArg,
    ErrNotConnected,
    ErrResyncing /* request refused: bus is realigning replies, retry next poll */
};

enum class AxisState : uint8_t {
    Undefined                     = 0,
    Idle                          = 1,
    StartupSequence               = 2,
    FullCalibration               = 3,
    MotorCalibration              = 4,
    EncoderIndexSearch            = 6,
    EncoderOffsetCalib            = 7,
    ClosedLoopControl             = 8,
    LockinSpin                    = 9,
    EncoderDirFind                = 10,
    Homing                        = 11,
    EncoderHallPolarityCalib      = 12,
    EncoderHallPhaseCalib         = 13,
    AnticoggingCalib              = 14,
    HarmonicCalib                 = 15,
    HarmonicCalibCommutation      = 16
};

enum class ProcedureResult : uint8_t {
    Success                      = 0,
    Busy                         = 1,
    Cancelled                    = 2,
    Disarmed                     = 3,
    NoResponse                   = 4,
    PolePairCprMismatch          = 5,
    PhaseResistanceOutOfRange    = 6,
    PhaseInductanceOutOfRange    = 7,
    UnbalancedPhases             = 8,
    InvalidMotorType             = 9,
    IllegalHallState             = 10,
    Timeout                      = 11,
    HomingWithoutEndstop         = 12,
    InvalidState                 = 13,
    NotCalibrated                = 14,
    NotConverging                = 15,
    RequestedCurrentTooHigh      = 16
};

enum class ControlMode : uint8_t {
    Voltage  = 0,
    Torque   = 1,
    Velocity = 2,
    Position = 3
};

enum class InputMode : uint8_t {
    Inactive     = 0,
    Passthrough  = 1,
    VelRamp      = 2,
    PosFilter    = 3,
    MixChannels  = 4,
    TrapTraj     = 5,
    TorqueRamp   = 6,
    Mirror       = 7,
    Tuning       = 8
};

/* Emulated periodic messages -- ASCII has no CAN-style cyclic broadcast, so
 * set_msg_rate() makes Bus::poll() request on an interval instead. */
enum class MsgRate {
    Version = 0,
    Heartbeat,
    Encoder,
    Iq,
    Error,
    Temperature,
    BusVoltage,
    Torques,
    Powers,
    Count
};

/* ODrive error bitfield (ODriveError enum, fw 0.6.x). */
constexpr uint32_t ERROR_NONE                        = 0x00000000u;
constexpr uint32_t ERROR_INITIALIZING                = 0x00000001u;
constexpr uint32_t ERROR_SYSTEM_LEVEL                = 0x00000002u;
constexpr uint32_t ERROR_TIMING_ERROR                = 0x00000004u;
constexpr uint32_t ERROR_MISSING_ESTIMATE            = 0x00000008u;
constexpr uint32_t ERROR_BAD_CONFIG                  = 0x00000010u;
constexpr uint32_t ERROR_DRV_FAULT                   = 0x00000020u;
constexpr uint32_t ERROR_MISSING_INPUT               = 0x00000040u;
constexpr uint32_t ERROR_DC_BUS_OVER_VOLTAGE         = 0x00000100u;
constexpr uint32_t ERROR_DC_BUS_UNDER_VOLTAGE        = 0x00000200u;
constexpr uint32_t ERROR_DC_BUS_OVER_CURRENT         = 0x00000400u;
constexpr uint32_t ERROR_DC_BUS_OVER_REGEN_CURRENT   = 0x00000800u;
constexpr uint32_t ERROR_CURRENT_LIMIT_VIOLATION     = 0x00001000u;
constexpr uint32_t ERROR_MOTOR_OVER_TEMP             = 0x00002000u;
constexpr uint32_t ERROR_INVERTER_OVER_TEMP          = 0x00004000u;
constexpr uint32_t ERROR_VELOCITY_LIMIT_VIOLATION    = 0x00008000u;
constexpr uint32_t ERROR_POSITION_LIMIT_VIOLATION    = 0x00010000u;
constexpr uint32_t ERROR_REQUESTED_CURRENT_TOO_HIGH  = 0x00020000u;
constexpr uint32_t ERROR_WATCHDOG_TIMER_EXPIRED      = 0x01000000u;
constexpr uint32_t ERROR_ESTOP_REQUESTED             = 0x02000000u;
constexpr uint32_t ERROR_SPINOUT_DETECTED            = 0x04000000u;
constexpr uint32_t ERROR_BRAKE_RESISTOR_DISARMED     = 0x08000000u;
constexpr uint32_t ERROR_THERMISTOR_DISCONNECTED     = 0x10000000u;
constexpr uint32_t ERROR_CALIBRATION_ERROR           = 0x40000000u;

struct Heartbeat {
    uint32_t axis_error           = 0;
    uint8_t  axis_state           = 0;
    uint8_t  procedure_result     = 0;
    uint8_t  trajectory_done_flag = 0;
};

/* Fields only update once Bus::poll() dispatches a matching reply -- enable
 * set_msg_rate() for a field to keep it fresh on its own. */
struct Feedback {
    Heartbeat hb;

    float pos_estimate      = 0.0f;  /* converted to user units */
    float vel_estimate      = 0.0f;  /* converted to user units */
    float iq_setpoint       = 0.0f;
    float iq_measured       = 0.0f;
    float fet_temperature   = 0.0f;
    float motor_temperature = 0.0f;
    float bus_voltage       = 0.0f;
    float bus_current       = 0.0f;
    float torque_target     = 0.0f;
    float torque_estimate   = 0.0f;
    float electrical_power  = 0.0f;
    float mechanical_power  = 0.0f;

    uint32_t active_errors  = 0;
    uint32_t disarm_reason  = 0;

    /* Last generic read_property() result. */
    std::string last_property_path;
    std::string last_property_value;

    uint8_t hw_version_major = 0, hw_version_minor = 0, hw_version_variant = 0;
    uint8_t fw_version_major = 0, fw_version_minor = 0, fw_version_revision = 0;
};

const char *axis_state_str(uint8_t state);
const char *procedure_result_str(uint8_t result);
const char *control_mode_str(uint8_t mode);
const char *input_mode_str(uint8_t mode);

/* Formats an error bitfield as "NAME|NAME|0xHEX" (or "none"); snprintf semantics. */
int error_str(uint32_t err, char *buf, size_t buf_len);

/* Formats a whole heartbeat, every field decoded to a name; snprintf semantics. */
int heartbeat_str(const Heartbeat &hb, char *buf, size_t buf_len);

/* ---- logger ---- */
using LogFn = std::function<void(const std::string &)>;
/* Attaches the one log sink for the whole library; pass an empty fn to detach. */
void attach_logger(LogFn fn);

class Axis;

using ReplyHandler = std::function<void(const std::string &line)>;
using PropertyHandler = std::function<void(const std::string &value)>;

/* One serial connection to an ODrive board; axis0/axis1 share it. Not
 * thread-safe for poll() -- call it from a single thread. */
class Bus {
public:
    explicit Bus(SerialWorker &worker);

    /* Fire-and-forget: send one command line, no reply expected. O(1). */
    Status send_line(const std::string &line);

    /* Sends one line and matches the next reply to on_reply (FIFO order).
     * Returns ErrResyncing (without sending) while the bus is realigning. */
    Status request(const std::string &line, ReplyHandler on_reply);

    /* Call once per control-loop iteration: dispatches buffered replies,
     * drops pending requests past the reply timeout, and runs due
     * set_msg_rate() getters. Never sleeps or blocks. */
    void poll();

    /* How long a request() may go unanswered before poll() drops it (default 200 ms). */
    void set_reply_timeout(int timeout_ms);

    /* True while poll() is discarding lines to realign replies with requests.
     * Clears itself on the first poll() pass that sees no incoming data. */
    bool is_resyncing() const;

    /* Device-wide system commands -- affect BOTH axes on the board. */
    Status clear_errors();
    Status save_config();
    Status erase_config();
    Status reboot();

    SerialWorker &worker() { return worker_; }

private:
    friend class Axis;
    void register_axis(Axis *axis);
    void unregister_axis(Axis *axis);

    struct Pending {
        ReplyHandler handler;
        std::chrono::steady_clock::time_point issued_at;
    };

    Status send_line_locked(const std::string &line);

    SerialWorker &worker_;
    mutable std::mutex mutex_; /* guards worker_ writes + pending_ + axes_ + resync state */
    std::deque<Pending> pending_;
    std::vector<Axis *> axes_;
    int reply_timeout_ms_ = 200;

    /* ASCII replies carry no request tag, so pairing is positional: one lost
     * or unsolicited line would mis-pair every later reply, permanently.
     * A dropped reply therefore invalidates the whole queue -- clear it and
     * swallow inbound lines until the port goes quiet, which re-aligns. */
    bool resyncing_ = false;
    std::chrono::steady_clock::time_point resync_quiet_since_{};
    int resync_quiet_ms_ = 50; /* silence that proves nothing is still in flight */
    size_t max_pending_ = 32;  /* backstop: replies not coming back at all */
};

using AxisCallback = std::function<void(Axis &)>;

/* One axis (motor channel) on a Bus. */
class Axis {
public:
    Axis(Bus &bus, uint8_t axis_index, const std::string &name = "");
    ~Axis();

    Axis(const Axis &) = delete;
    Axis &operator=(const Axis &) = delete;

    /* Motor turns per user unit; applies to position/velocity only. 0 == 1.0. */
    void set_conversion(float turns_per_unit, bool invert);
    void enable_logging(bool enable);
    uint8_t index() const { return axis_index_; }

    /* ---- setpoints  ---- */
    Status set_input_pos(float pos, float vel_ff = 0.0f, float torque_ff = 0.0f);
    Status set_input_vel(float vel, float torque_ff = 0.0f);
    Status set_input_torque(float torque);
    Status set_absolute_position(float pos);
    /* Moves delta relative to the last received encoder estimate; ErrBadArg
     * until a request_encoder() reply has been dispatched at least once. */
    Status set_relative_pos(float delta);

    /* ---- control/config---- */
    Status set_axis_state(AxisState state);
    Status set_closed_loop(bool enable);
    Status set_controller_mode(ControlMode control_mode, InputMode input_mode);
    Status set_limits(float vel_limit, float current_limit);
    Status set_traj_vel_limit(float vel_limit);
    Status set_traj_accel_limits(float accel, float decel);
    Status set_defaults(float vel_limit, float accel, float decel);
    Status restore_defaults();
    Status clear_errors();  /* forwards to Bus -- affects both axes */
    /* Best-effort: no hard e-stop over ASCII -- this just requests AxisState::Idle. */
    Status estop();
    Status reboot();       /* forwards to Bus -- affects both axes */
    Status save_config();  /* forwards to Bus -- affects both axes */
    Status erase_config(); /* forwards to Bus -- affects both axes */

    /* ---- getters -- send the request and return immediately; `feedback`
     * and the matching callback update once Bus::poll() dispatches the reply. ---- */
    Status request_version();
    Status request_error();
    Status request_encoder();
    Status request_iq();
    Status request_temperature();
    Status request_bus_vi();
    Status request_torques();
    Status request_powers();
    Status poll_heartbeat();

    /* ---- generic property access ---- */
    Status write_property(const std::string &path, const std::string &value);

    /* Non-blocking: sends "r path"; on_value (optional) runs with the raw
     * reply once poll() dispatches it. */
    Status read_property(const std::string &path, PropertyHandler on_value = nullptr);

    /* ---- host-side emulated periodic message rates ---- */
    Status set_msg_rate(MsgRate msg, uint32_t rate_ms);

    /* Sets every MsgRate kind (Heartbeat, Encoder, Iq, ...) to the same rate_ms. */
    Status set_all_msg_rates(uint32_t rate_ms);

    /* ---- callback registration ---- */
    void on_heartbeat(AxisCallback fn);
    void on_encoder(AxisCallback fn);
    void on_iq(AxisCallback fn);
    void on_temperature(AxisCallback fn);
    void on_bus_vi(AxisCallback fn);
    void on_torques(AxisCallback fn);
    void on_powers(AxisCallback fn);
    void on_error(AxisCallback fn);
    void on_version(AxisCallback fn);
    
    /* Fires after every dispatched read_property() reply, including internal
     * reads behind request_iq()/request_temperature()/etc. */
    void on_property(AxisCallback fn);

    Feedback feedback;

private:
    friend class Bus;
    using PropertiesHandler = std::function<void(const std::vector<std::string> &values)>;

    std::string prefix() const; /* "axis0." / "axis1." */
    void logf(const char *fmt, ...);
    /* Sends "r " for each path back-to-back; combine() runs once every
     * reply has been dispatched, in path order regardless of arrival order. */
    Status request_many(std::vector<std::string> paths, PropertiesHandler combine);
    void poll_periodic(std::chrono::steady_clock::time_point now);

    Bus &bus_;
    uint8_t axis_index_;
    std::string name_;
    bool log_enabled_ = true;

    float turns_per_unit_ = 1.0f; /* signed: motor turns per user unit + inversion */
    bool pos_valid_ = false;      /* set once a request_encoder() reply is dispatched */

    /* Heartbeat samples that failed the plausibility check (mis-paired reply);
     * edge-logged so a desync reports twice, not once per poll. */
    bool hb_rejecting_ = false;
    uint32_t hb_rejected_ = 0;

    float default_vel_limit_ = 0.0f;
    float default_accel_     = 0.0f;
    float default_decel_     = 0.0f;

    AxisCallback cb_heartbeat_, cb_encoder_, cb_iq_, cb_temperature_, cb_bus_vi_,
        cb_torques_, cb_powers_, cb_error_, cb_version_, cb_property_;

    std::array<uint32_t, static_cast<size_t>(MsgRate::Count)> msg_rate_ms_{};
    std::array<std::chrono::steady_clock::time_point, static_cast<size_t>(MsgRate::Count)> next_due_{};
};

} // namespace odrive

#endif // ODRIVE_LIB_CPP_H
