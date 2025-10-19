#!/usr/bin/env python3
#Naim Lindsay
# CS350 Final Project - Thermostat (
# Buttons:  MODE (yellow)=GPIO12   UP (red)=GPIO25   DOWN (blue)=GPIO24   → other side to GND
# LEDs:     RED=GPIO18  BLUE=GPIO23 (each through 220Ω to GND)
# I2C:      AHT20 @0x38  |  16x2 I2C LCD (PCF8574) @0x27  (SDA=GPIO2 pin3, SCL=GPIO3 pin5)
# UART:     /dev/serial0 @115200  (CSV every 30s)
#
# LCD line1: date/time; line2 alternates every 2s between "Now:xx.x°F" and "<STATE>  Set:72°F"
# LED logic: HEAT→ red fades if temp<setpoint else solid; COOL→ blue fades if temp>setpoint else solid; OFF→ both off

import sys, time, threading
from enum import Enum, auto

# ----------------- Pin / device config -----------------
PIN_BTN_MODE  = 12   # yellow
PIN_BTN_UP    = 25   # red
PIN_BTN_DOWN  = 24   # blue

PIN_LED_RED   = 18
PIN_LED_BLUE  = 23

I2C_BUS_NUM   = 1
AHT20_ADDR    = 0x38
LCD_I2C_ADDR  = 0x27       # change to 0x3F if needed

SERIAL_DEV    = "/dev/serial0"
SERIAL_BAUD   = 115200

DEFAULT_SETPOINT_F = 72.0
SETPOINT_MIN_F     = 50.0
SETPOINT_MAX_F     = 85.0
LCD_SWITCH_SECS    = 2.0
UART_PERIOD_SECS   = 30.0

def c_to_f(c): return c*9/5 + 32.0
def clamp(v, lo, hi): return max(lo, min(hi, v))

# ----------------- Optional libraries (allow simulation) -----------------
try:
    from gpiozero import Button, PWMLED
except Exception:
    Button = PWMLED = None

try:
    import serial
except Exception:
    serial = None

try:
    from smbus2 import SMBus, i2c_msg
except Exception:
    SMBus = i2c_msg = None

# ----------------- AHT20 temp sensor -----------------
class AHT20:
    def __init__(self, bus_num=1, addr=0x38):
        if SMBus is None: raise RuntimeError("smbus2 missing")
        self.addr = addr
        self.bus  = SMBus(bus_num)
        self._init()

    def _init(self):
        try:
            self.bus.write_byte(self.addr, 0xBA)    # soft reset
            time.sleep(0.02)
        except Exception:
            pass
        time.sleep(0.02)

    def _write(self, data):
        self.bus.write_i2c_block_data(self.addr, data[0], data[1:])

    def _read(self, n):
        msg = i2c_msg.read(self.addr, n)
        self.bus.i2c_rdwr(msg)
        return list(msg)

    def read_temp_c(self):
        self._write([0xAC, 0x33, 0x00])
        time.sleep(0.08)
        d = self._read(6)
        if len(d) != 6: raise IOError("AHT20 bad read")
        t_raw = ((d[3] & 0x0F) << 16) | (d[4] << 8) | d[5]
        return ((t_raw / 1048576.0) * 200.0) - 50.0

    def close(self):
        try: self.bus.close()
        except Exception: pass

# ----------------- I2C 16x2 LCD (PCF8574) -----------------
class I2CLcd1602:
    BL = 0x08; EN = 0x04; RW = 0x02; RS = 0x01
    def __init__(self, bus_num=1, addr=0x27):
        if SMBus is None: raise RuntimeError("smbus2 missing")
        self.bus  = SMBus(bus_num)
        self.addr = addr
        self.backlight = True
        self._init_lcd()

    def _w(self, val): self.bus.write_byte(self.addr, val | (self.BL if self.backlight else 0))
    def _pulse(self, data):
        self._w(data | self.EN); time.sleep(0.0005)
        self._w(data & ~self.EN); time.sleep(0.0001)
    def _write4(self, nib, rs=0):
        self._pulse((nib & 0xF0) | rs)
    def _cmd(self, cmd):
        self._write4(cmd & 0xF0, 0); self._write4((cmd<<4) & 0xF0, 0)
    def _data(self, data):
        self._write4(data & 0xF0, self.RS); self._write4((data<<4) & 0xF0, self.RS)

    def _init_lcd(self):
        time.sleep(0.05)
        for _ in range(3):
            self._write4(0x30); time.sleep(0.004)
        self._write4(0x20)
        self._cmd(0x28)   # 2 lines
        self._cmd(0x0C)   # display on, cursor off
        self.clear()
        self._cmd(0x06)   # entry mode

    def clear(self):
        self._cmd(0x01); time.sleep(0.002)

    def set_cursor(self, col, row):
        self._cmd((0x80 if row==0 else 0xC0) + col)

    def print_line(self, text, row=0):
        s = text.ljust(16)[:16]
        self.set_cursor(0, row)
        for ch in s: self._data(ord(ch))

    def close(self):
        try: self.bus.close()
        except Exception: pass

# ----------------- State enum -----------------
class Mode(Enum):
    OFF = auto()
    HEAT = auto()
    COOL = auto()

# ----------------- Thermostat controller -----------------
class Thermostat:
    def __init__(self):
        self.mode = Mode.OFF
        self.set_f = DEFAULT_SETPOINT_F
        self.temp_f = DEFAULT_SETPOINT_F
        self._last_lcd_toggle = 0.0
        self._lcd_show_temp   = True
        self._last_uart_emit  = 0.0
        self._stop = threading.Event()

        # GPIO
        self.simulate = False
        if Button is None or PWMLED is None:
            self.simulate = True
            self.btn_mode = self.btn_up = self.btn_down = None
            self.redLight = self.blueLight = None
            print("GPIO not available → simulation mode.")
        else:
            self.btn_mode = Button(PIN_BTN_MODE, pull_up=True, bounce_time=0.05)
            self.btn_up   = Button(PIN_BTN_UP,   pull_up=True, bounce_time=0.05)
            self.btn_down = Button(PIN_BTN_DOWN, pull_up=True, bounce_time=0.05)
            self.btn_mode.when_pressed = self.on_mode
            self.btn_up.when_pressed   = self.on_up
            self.btn_down.when_pressed = self.on_down
            self.redLight  = PWMLED(PIN_LED_RED,  frequency=200)
            self.blueLight = PWMLED(PIN_LED_BLUE, frequency=200)

        # I2C devices
        self.sensor = None
        try:
            self.sensor = AHT20(I2C_BUS_NUM, AHT20_ADDR)
        except Exception as e:
            print(f"AHT20 init failed: {e} → simulate temp.")

        self.lcd = None
        try:
            self.lcd = I2CLcd1602(I2C_BUS_NUM, LCD_I2C_ADDR)
        except Exception as e:
            print(f"LCD init failed: {e} → console display.")

        # UART
        self.ser = None
        if serial is not None:
            try: self.ser = serial.Serial(SERIAL_DEV, SERIAL_BAUD, timeout=0)
            except Exception as e: print(f"UART open failed: {e} → console emit.")

        # UI init
        self.update_leds(active=False, solid_off=True)
        self.draw_lcd(force=True)

        # Start loop
        self._thread = threading.Thread(target=self.loop, daemon=True)
        self._thread.start()

    # -------- Button callbacks --------
    def on_mode(self):
        if self.mode == Mode.OFF: self.transition(Mode.HEAT)
        elif self.mode == Mode.HEAT: self.transition(Mode.COOL)
        else: self.transition(Mode.OFF)

    def on_up(self):
        self.set_f = clamp(self.set_f + 1.0, SETPOINT_MIN_F, SETPOINT_MAX_F)
        self.draw_lcd(force=True)

    def on_down(self):
        self.set_f = clamp(self.set_f - 1.0, SETPOINT_MIN_F, SETPOINT_MAX_F)
        self.draw_lcd(force=True)

    # -------- State transitions --------😎
    def transition(self, new_mode: Mode):
        if new_mode == self.mode: return
        # exit old
        if self.mode == Mode.HEAT: self.on_exit_heat()
        if self.mode == Mode.COOL: self.on_exit_cool()
        if self.mode == Mode.OFF:  self.on_exit_off()
        # switch
        self.mode = new_mode
        # enter new
        if self.mode == Mode.HEAT: self.on_enter_heat()
        if self.mode == Mode.COOL: self.on_enter_cool()
        if self.mode == Mode.OFF:  self.on_enter_off()
        self.draw_lcd(force=True)

    # -------- Entry/Exit hooks (all have bodies) --------
    def on_enter_heat(self):
        if self.redLight:  self.redLight.pulse(fade_in_time=1.0, fade_out_time=1.0, n=None, background=True)
        if self.blueLight: self.blueLight.off()

    def on_exit_heat(self):
        if self.redLight: self.redLight.off()

    def on_enter_cool(self):
        if self.blueLight: self.blueLight.pulse(fade_in_time=1.0, fade_out_time=1.0, n=None, background=True)
        if self.redLight:  self.redLight.off()

    def on_exit_cool(self):
        if self.blueLight: self.blueLight.off()

    def on_enter_off(self):
        if self.redLight:  self.redLight.off()
        if self.blueLight: self.blueLight.off()

    def on_exit_off(self):
        pass

    # -------- Main loop --------
    def loop(self):
        while not self._stop.is_set():
            # read/update temperature
            if self.sensor:
                try:
                    self.temp_f = c_to_f(self.sensor.read_temp_c())
                except Exception:
                    pass
            else:
                # simulate drift toward setpoint
                delta = self.set_f - self.temp_f
                if abs(delta) > 0.1:
                    self.temp_f += 0.05 if delta > 0 else -0.05

            # decide active
            active = False
            if self.mode == Mode.HEAT: active = self.temp_f < self.set_f
            elif self.mode == Mode.COOL: active = self.temp_f > self.set_f

            self.update_leds(active=active)

            # LCD alternation
            now = time.time()
            if now - self._last_lcd_toggle >= LCD_SWITCH_SECS:
                self._lcd_show_temp = not self._lcd_show_temp
                self._last_lcd_toggle = now
                self.draw_lcd()

            # UART emit
            if now - self._last_uart_emit >= UART_PERIOD_SECS:
                self.emit_uart()
                self._last_uart_emit = now

            time.sleep(0.2)

    # -------- LEDs --------
    def update_leds(self, active=False, solid_off=False):
        if not self.redLight or not self.blueLight:
            return
        self.redLight.off(); self.blueLight.off()
        if solid_off or self.mode == Mode.OFF:
            return
        if self.mode == Mode.HEAT:
            if active: self.redLight.pulse(fade_in_time=1.0, fade_out_time=1.0, n=None, background=True)
            else: self.redLight.value = 1.0
        elif self.mode == Mode.COOL:
            if active: self.blueLight.pulse(fade_in_time=1.0, fade_out_time=1.0, n=None, background=True)
            else: self.blueLight.value = 1.0

    # -------- LCD --------
    def draw_lcd(self, force=False):
        ts = time.strftime("%Y-%m-%d %H:%M")
        state = "off" if self.mode == Mode.OFF else ("heat" if self.mode == Mode.HEAT else "cool")
        temp_str = f"{self.temp_f:0.1f}°F"
        setp_str = f"Set:{int(round(self.set_f))}°F"
        if self.lcd:
            try:
                self.lcd.print_line(ts, 0)
                if self._lcd_show_temp or force:
                    self.lcd.print_line(f"Now:{temp_str}".ljust(16)[:16], 1)
                else:
                    self.lcd.print_line(f"{state.upper():<5}{setp_str}".ljust(16)[:16], 1)
            except Exception as e:
                print(f"LCD error: {e}")
        else:
            # console fallback
            if self._lcd_show_temp or force:
                sys.stdout.write(f"\r{ts} | Now:{temp_str}    ")
            else:
                sys.stdout.write(f"\r{ts} | {state.upper()}  {setp_str}    ")
            sys.stdout.flush()

    # -------- UART --------
    def emit_uart(self):
        state = "off" if self.mode == Mode.OFF else ("heat" if self.mode == Mode.HEAT else "cool")
        line = f"{state},{self.temp_f:0.1f},{int(round(self.set_f))}\n"
        if self.ser:
            try: self.ser.write(line.encode("utf-8"))
            except Exception as e: print(f"\nUART write failed: {e}")
        else:
            print(f"\nUART -> {line.strip()}")

    # -------- Cleanup --------
    def close(self):
        self._stop.set()
        try: self._thread.join(timeout=2.0)
        except Exception: pass
        try:
            if self.redLight: self.redLight.off()
            if self.blueLight: self.blueLight.off()
        except Exception: pass
        try:
            if self.ser: self.ser.close()
        except Exception: pass
        try:
            if self.lcd: self.lcd.clear(); self.lcd.close()
        except Exception: pass
        try:
            if self.sensor: self.sensor.close()
        except Exception: pass

# ----------------- main -----------------
def main():
    print("Starting CS350 Thermostat …")
    t = Thermostat()
    try:
        while True: time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nShutting down …")
    finally:
        t.close()

if __name__ == "__main__":
    main()

