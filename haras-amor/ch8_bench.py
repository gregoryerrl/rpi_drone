"""Repeatable props-off, disarmed CH8 bench runs at 20% that last until CH8 is lowered.

Run venv/bin/python ch8_bench.py from haras-amor. Lower CH8 or raise CH6 to stop a run.
Cycle CH8 low then high for another run. Ctrl-C exits the session.
"""
import asyncio
import signal
import time
from motor_preview import motors
from motor_preview.stop import stop_marker
from pymavlink.dialects.v20 import common as mav

class Ch8BenchLink(motors.MotorLink):
    def _bench_pulse(self, percent):
        # No session cutoff: a run lasts while CH8 stays high. Each command is a
        # short PX4 pulse that the loop renews, so the motors stop if renewals stop.
        return motors.PULSE_SECONDS


async def once(device=motors.DEFAULT_DEVICE):
    listening = False
    link = Ch8BenchLink(device)
    low_seen = running = False
    last_radio = 0
    stamp = None
    healthy = True
    stop_reason = None
    started = None
    stop_time = None
    accepted = acks = 0
    releases = set()

    exit_requested = False

    def stop(why):
        nonlocal stop_reason, stop_time, exit_requested
        if why in ('operator stop', 'armed'):
            exit_requested = True
        if stop_reason is None:
            stop_reason = why
            stop_time = time.monotonic()
            link.stop_outputs()

    def observe(message):
        nonlocal last_radio, stamp, low_seen, running, healthy, acks
        if (message.get_srcSystem(), message.get_srcComponent()) != (link.system, mav.MAV_COMP_ID_AUTOPILOT1):
            return
        kind = message.get_type()
        if kind == 'COMMAND_ACK' and stop_reason and (message.command == mav.MAV_CMD_ACTUATOR_TEST) and (message.result == 0) and (message.target_system in (0, 255)) and (message.target_component in (0, 190)):
            acks += 1
        if kind == 'HEARTBEAT' and message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
            stop('armed')
        if kind == 'SYS_STATUS':
            bit = mav.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
            healthy = not (message.onboard_control_sensors_enabled & message.onboard_control_sensors_present & bit and (not message.onboard_control_sensors_health & bit))
            if not healthy:
                low_seen = False
                if running:
                    stop('RC unhealthy')
        if kind != 'RC_CHANNELS':
            return
        now = time.monotonic()
        switches = (message.chan5_raw, message.chan6_raw, message.chan8_raw)
        gap = now - last_radio > 0.25
        valid = message.chancount >= 8 and message.rssi != 0 and all((800 <= params <= 2200 for params in switches)) and (stamp is None or 0 < (message.time_boot_ms - stamp) % (1 << 32) < 1 << 31)
        last_radio = now if valid else 0
        stamp = message.time_boot_ms
        if gap or not valid or (not healthy) or (max(switches[:2]) > 1250) or (running and switches[2] < 1850):
            low_seen = False
            if running:
                if gap or not valid or not healthy or switches[0] > 1250:
                    stop('radio/CH5 interlock')
                elif switches[1] > 1250:
                    stop('CH6 stop')
                else:
                    stop('CH8 lowered')
        elif listening:
            if switches[2] <= 1250:
                low_seen = True
            if low_seen and switches[2] >= 1850:
                running = True
    link.wire.set_callback(observe)
    send_command = link._send

    def write(cmd, *params):
        nonlocal started
        if cmd == mav.MAV_CMD_ACTUATOR_TEST:
            if params[4] not in motors.MOTORS:
                raise motors.BenchError('invalid output')
            if params[:2] == (0, 0):
                send_command(cmd, *params)
                releases.add(params[4])
                return
            if stop_reason or not running or (not healthy) or link.jig.armed or (time.monotonic() - last_radio > 0.25) or stop_marker().exists() or (params[0] != 0.2) or (not 0 < params[1] <= 0.6):
                raise motors.BenchError('interlock')
            if started is None:
                started = time.monotonic()
        elif cmd not in (mav.MAV_CMD_REQUEST_MESSAGE, mav.MAV_CMD_SET_MESSAGE_INTERVAL):
            raise motors.BenchError('bench only')
        send_command(cmd, *params)
    link._send = write
    for s in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(s, stop, 'operator stop')
    try:
        await link.ready()
        # Ask for 20 Hz receiver updates so the 0.25 s radio-loss checks are not
        # tripped by a slower RC_CHANNELS rate left configured by another tool.
        await link._confirmed(1.5, mav.MAV_CMD_SET_MESSAGE_INTERVAL,
                              mav.MAVLINK_MSG_ID_RC_CHANNELS, 50000, 0, 0, 0, 0, 0)
        listening = True
        print('Ready. Keep CH5 and CH6 low, then move CH8 from low to high.', flush=True)
        until = time.monotonic() + 120
        next_send = 0
        while not stop_reason:
            link.check()
            if stop_marker().exists():
                stop('motor stop latched')
                break
            if time.monotonic() - last_radio > 0.25:
                low_seen = False
                if running:
                    stop('radio lost')
                    break
            if not running and time.monotonic() > until:
                stop('no trigger')
                break
            if running and time.monotonic() >= next_send:
                await link.set_percent(20)
                accepted += 1
                if accepted == 1:
                    print('PX4 accepted all four at 20%', flush=True)
                next_send = time.monotonic() + 0.1
            await asyncio.sleep(0.012)
    except (motors.BenchError, OSError) as e:
        stop(str(e))
    finally:
        stop('ended')
        await asyncio.sleep(0.35)
        link.close()
        for sig in (signal.SIGINT, signal.SIGTERM):
            asyncio.get_running_loop().remove_signal_handler(sig)
        print(stop_reason, 'elapsed', round(stop_time - started, 3) if started else None, 'batches', accepted, 'released', sorted(releases), 'acks', acks, 'armed', link.jig.armed)

    return (not exit_requested and not link.jig.armed and not link.fault
            and stop_reason in ('CH6 stop', 'CH8 lowered')
            and releases == set(motors.MOTORS) and acks >= len(releases))


async def session(device=motors.DEFAULT_DEVICE):
    while await once(device):
        print('Run finished. Cycle CH8 low and high for another run, or press Ctrl-C to exit.', flush=True)


if __name__ == '__main__':
    asyncio.run(session())
