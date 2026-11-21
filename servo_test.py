#!/usr/bin/env python3
# servo_test.py — test servos on PCA9685 channels 0 and 1
#
#   python servo_test.py            # auto sweep test of Ch.0 then Ch.1
#   python servo_test.py -i         # interactive: type "0 1500", "1 90deg", "c", "q"
#
# Wiring (Pi 5 40-pin header):  PCA9685 VCC -> pin 1 (3.3V)   GND -> pin 6
#                               SDA -> pin 3 (GPIO2)          SCL -> pin 5 (GPIO3)
#                               V+ (servo power) -> separate 5-6V supply, GND shared
import argparse, sys, time
from smbus2 import SMBus

MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
OSC_HZ = 25_000_000
PWM_HZ = 50                      # standard analog servo frame rate
PERIOD_US = 1_000_000 / PWM_HZ   # 20000 us

# Safe pulse limits — widen slowly if your servo doesn't reach its full travel
MIN_US, MID_US, MAX_US = 1000, 1500, 2000
CHANNELS = (0, 1)


class PCA9685:
    def __init__(self, bus=1, addr=0x40):
        self.bus, self.addr = SMBus(bus), addr
        self.bus.write_byte_data(addr, MODE1, 0x00)          # reset to normal
        self.set_freq(PWM_HZ)
        self.bus.write_byte_data(addr, MODE2, 0x04)          # totem-pole outputs

    def set_freq(self, hz):
        prescale = round(OSC_HZ / (4096 * hz)) - 1
        old = self.bus.read_byte_data(self.addr, MODE1)
        self.bus.write_byte_data(self.addr, MODE1, (old & 0x7F) | 0x10)  # sleep
        self.bus.write_byte_data(self.addr, PRESCALE, prescale)
        self.bus.write_byte_data(self.addr, MODE1, old)
        time.sleep(0.005)
        self.bus.write_byte_data(self.addr, MODE1, old | 0xA0)           # restart + auto-increment

    def set_pulse_us(self, ch, us):
        us = max(MIN_US, min(MAX_US, us))
        ticks = int(us * 4096 / PERIOD_US)
        self.bus.write_i2c_block_data(self.addr, LED0_ON_L + 4 * ch,
                                      [0, 0, ticks & 0xFF, ticks >> 8])
        return us

    def off(self, ch):
        self.bus.write_i2c_block_data(self.addr, LED0_ON_L + 4 * ch, [0, 0, 0, 0x10])  # full OFF

    def close(self):
        for ch in CHANNELS:
            self.off(ch)
        self.bus.close()


def angle_to_us(deg):
    deg = max(0, min(180, deg))
    return MIN_US + (MAX_US - MIN_US) * deg / 180


def sweep_test(pca):
    for ch in CHANNELS:
        print(f"\n--- Ch.{ch} ---")
        for label, us in (("center", MID_US), ("min", MIN_US), ("center", MID_US),
                          ("max", MAX_US), ("center", MID_US)):
            print(f"  {label:6s} {us} us")
            pca.set_pulse_us(ch, us)
            time.sleep(1)
        print("  smooth sweep...")
        for us in list(range(MIN_US, MAX_US + 1, 10)) + list(range(MAX_US, MIN_US - 1, -10)):
            pca.set_pulse_us(ch, us)
            time.sleep(0.01)
        pca.set_pulse_us(ch, MID_US)

    print("\n--- Both channels together ---")
    for us in (MIN_US, MAX_US, MID_US):
        print(f"  Ch.0 & Ch.1 -> {us} us")
        for ch in CHANNELS:
            pca.set_pulse_us(ch, us)
        time.sleep(1)


def interactive(pca):
    print("Commands:  <ch> <us>   e.g. '0 1500'   |  <ch> <deg>deg  e.g. '1 90deg'"
          "  |  c = center both  |  q = quit")
    while True:
        try:
            cmd = input("> ").strip().lower()
        except EOFError:
            break
        if cmd in ("q", "quit", "exit"):
            break
        if cmd == "c":
            for ch in CHANNELS:
                pca.set_pulse_us(ch, MID_US)
            print(f"  both -> {MID_US} us")
            continue
        try:
            ch_s, val = cmd.split()
            ch = int(ch_s)
            if ch not in CHANNELS:
                raise ValueError
            us = angle_to_us(float(val[:-3])) if val.endswith("deg") else float(val)
            print(f"  Ch.{ch} -> {pca.set_pulse_us(ch, us):.0f} us")
        except ValueError:
            print("  ?  try '0 1500', '1 45deg', 'c' or 'q'")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--bus", type=int, default=1, help="I2C bus (default 1 = /dev/i2c-1)")
    ap.add_argument("-a", "--addr", type=lambda x: int(x, 0), default=0x40)
    ap.add_argument("-i", "--interactive", action="store_true")
    args = ap.parse_args()

    try:
        pca = PCA9685(args.bus, args.addr)
    except FileNotFoundError:
        sys.exit(f"/dev/i2c-{args.bus} missing — enable I2C (dtparam=i2c_arm=on) and reboot")
    except OSError as e:
        sys.exit(f"No PCA9685 at 0x{args.addr:02x} on bus {args.bus} ({e}) — check wiring, "
                 f"run 'i2cdetect -y {args.bus}'")

    print(f"PCA9685 ready on bus {args.bus} @ 0x{args.addr:02x}, {PWM_HZ} Hz")
    try:
        interactive(pca) if args.interactive else sweep_test(pca)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nCentering and releasing servos")
        for ch in CHANNELS:
            pca.set_pulse_us(ch, MID_US)
        time.sleep(0.5)
        pca.close()


if __name__ == "__main__":
    main()
