/* Example: set every emulated periodic message (heartbeat, encoder, iq,
 * error, temperature, bus_vi, torques, powers) to a 100ms rate in one call,
 * then service the bus from a detached background thread. Free-standing
 * reference file -- not wired into the CMake build. */
#include "odrive_lib_cpp.h"
#include "SerialWorker.h"
#include <iostream>
#include <thread>
#include <chrono>

SerialWorker worker(false, false);
odrive::Bus bus(worker);
odrive::Axis axis0(bus, 0, "axis0");

void setup()
{
    worker.set_connection_params("COM8", 115200);
    worker.set_terminators("\r\n", "\n");
    worker.start();

    while (!worker.isOpen()) {
        std::cout << "Waiting for serial connection..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    /* Requests heartbeat/encoder/iq/error/temperature/bus_vi/torques/powers
     * every 100 ms, instead of calling set_msg_rate() once per MsgRate. */
    axis0.set_all_msg_rates(100);

    axis0.clear_errors();
    axis0.set_limits(10, 9);
    axis0.set_traj_vel_limit(10);

    /* The 100ms cadence is enforced inside Bus::poll() (poll_periodic checks
     * elapsed time per message), so this thread must call poll() faster than
     * 100ms or the schedule will slip -- 10ms gives it headroom. */
    std::thread odrive_feedback([&] {
        while (true) {
            bus.poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    odrive_feedback.detach();

    axis0.set_axis_state(odrive::AxisState::ClosedLoopControl);
}

void loop()
{
    while (true) {
        std::cout << "pos=" << axis0.feedback.pos_estimate
                  << " vel=" << axis0.feedback.vel_estimate
                  << " state=" << odrive::axis_state_str(axis0.feedback.hb.axis_state)
                  << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main()
{
    setup();
    loop();
    return 0;
}