#!/usr/bin/env python3
# ============================================================
#  KY170DD01005-08G  –  Klavye ile Pozisyon + LED Kontrolü
#  Donanım : Raspberry Pi 5 + MCP2515 (SocketCAN)
#            + GPIO LED'ler (LED1, LED2, LED8-11)
#            + Arduino (LED3-7 analog voltaj kontrolü)
#
#  Motor:
#    A → Sola    D → Sağa    Q → Çıkış
#
#  LED (GPIO - Raspberry Pi):
#    E → LED1  toggle
#    F → LED2  toggle
#    J → LED8  yanar, 2 sn sonra otomatik söner
#    K → LED9  toggle
#    L → LED10 toggle
#    M → LED11 toggle
#
#  LED (Arduino - Analog Voltaj):
#    G → LED3=3.5V  LED4=3.5V  LED5=1.49V  LED6=0V  LED7=1.49V
#    H → LED3=1.49V LED4=3.5V  LED5=3.5V   LED6=0V  LED7=1.49V
#    I → LED3=1.49V LED4=0V    LED5=1.49V  LED6=0V  LED7=1.49V
#
#  Bağlantılar:
#    Raspberry Pi Pin 8  (TX) → Arduino Pin 0 (RX)
#    Raspberry Pi Pin 10 (RX) → Arduino Pin 1 (TX)
#    Raspberry Pi Pin 6  (GND) → Arduino GND
#
#  GPIO Pin Atamaları:
#    LED1  → GPIO17 (Pin 11)
#    LED2  → GPIO18 (Pin 12)
#    LED8  → GPIO5  (Pin 29)
#    LED9  → GPIO6  (Pin 31)
#    LED10 → GPIO13 (Pin 33)
#    LED11 → GPIO19 (Pin 35)
# ============================================================

import can
import time
import struct
import sys
import signal
import logging
import termios
import tty
import select
import threading
import serial
import RPi.GPIO as GPIO

# ============================================================
#  LOGGING  (sadece WARNING ve üzeri)
# ============================================================
logging.basicConfig(
    level=logging.WARNING,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger(__name__)

# ============================================================
#  MOTOR CAN ID TANIMLARI
# ============================================================
MOTOR_ID  = 0x01
CAN_TX_ID = 0x06000000 | MOTOR_ID
CAN_RX_ID = 0x05800000 | MOTOR_ID
CAN_HB_ID = 0x07000000 | MOTOR_ID

# ============================================================
#  POZİSYON SINIRI  (10000 birim = 1 tur)
# ============================================================
POS_MAX =  50000
POS_MIN = -50000

# ============================================================
#  AYARLAR
# ============================================================
CAN_INTERFACE  = "can0"
STEP_SIZE      = 1000
KEEPALIVE_S    = 0.500
ARDUINO_PORT   = "/dev/ttyUSB0"   # ls /dev/ttyUSB* ile kontrol et
ARDUINO_BAUD   = 9600

# ============================================================
#  GPIO PIN ATAMALARI (BCM) – Sadece dijital LED'ler
# ============================================================
LED_PINS = {
    1:  17,   # LED1  → GPIO17 (Pin 11)
    2:  18,   # LED2  → GPIO18 (Pin 12)
    8:   5,   # LED8  → GPIO5  (Pin 29)
    9:   6,   # LED9  → GPIO6  (Pin 31)
    10: 13,   # LED10 → GPIO13 (Pin 33)
    11: 19,   # LED11 → GPIO19 (Pin 35)
}


# ============================================================
#  NON-BLOCKING KLAVYE OKUMA
# ============================================================
class KeyboardReader:
    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old_settings = None

    def __enter__(self):
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def get_key(self) -> str | None:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            return sys.stdin.read(1).lower()
        return None


# ============================================================
#  KY170 MOTOR + LED KONTROL SINIFI
# ============================================================
class KY170Controller:
    def __init__(self):
        self.current_position: int            = 0
        self.motor_enabled: bool              = False
        self.last_cmd_time: float             = 0.0
        self.led_states: dict                 = {i: False for i in LED_PINS}
        self._led8_timer: threading.Timer | None = None


        # --- GPIO kurulum ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in LED_PINS.values():
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        # --- Arduino seri bağlantı ---
        print("  Arduino bağlanıyor...", end=" ", flush=True)
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=2)
            time.sleep(2)   # Arduino reset süresi
            self.arduino.reset_input_buffer()
            print("OK ✓")
        except serial.SerialException as e:
            print(f"HATA ✖  ({e})")
            print("  ⚠ Arduino bağlantısı yok, G/H/I tuşları çalışmaz.")
            self.arduino = None

        # --- CAN kurulum ---
        print("  CAN arayüzü açılıyor...", end=" ", flush=True)
        try:
            self.bus = can.interface.Bus(
                channel=CAN_INTERFACE,
                interface="socketcan",
                receive_own_messages=False
            )
            self.notifier = can.Notifier(self.bus, [self._can_listener])
            print("OK ✓")
        except Exception as e:
            print(f"HATA ✖  ({e})")
            sys.exit(1)

    # ----------------------------------------------------------
    #  CAN GÖNDERME
    # ----------------------------------------------------------
    def _send_can(self, can_id: int, data: bytes):
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
        try:
            self.bus.send(msg)
            self.last_cmd_time = time.monotonic()
        except can.CanError as e:
            print(f"  ✖ Motora ulaşılamadı: {e}")

    # ----------------------------------------------------------
    #  CAN ALICI  (sadece hata durumunda ekrana yazar)
    # ----------------------------------------------------------
    def _can_listener(self, msg: can.Message):
        aid = msg.arbitration_id
        if aid == CAN_HB_ID and len(msg.data) >= 8:
            err = struct.unpack(">H", msg.data[6:8])[0]
            if err != 0:
                print(f"  ⚠ Motor hata kodu: 0x{err:04X}")

    # ----------------------------------------------------------
    #  MOTOR KOMUTLARI
    # ----------------------------------------------------------
    def enable_motor(self):
        self._send_can(CAN_TX_ID, bytes([0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]))
        self.motor_enabled = True

    def disable_motor(self):
        self._send_can(CAN_TX_ID, bytes([0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]))
        self.motor_enabled = False

    def send_position(self, pos: int):
        pos_u32 = pos & 0xFFFFFFFF
        b0 = (pos_u32 >>  8) & 0xFF
        b1 = (pos_u32 >>  0) & 0xFF
        b2 = (pos_u32 >> 24) & 0xFF
        b3 = (pos_u32 >> 16) & 0xFF
        self._send_can(CAN_TX_ID, bytes([0x23, 0x02, 0x20, 0x01, b0, b1, b2, b3]))

    # ----------------------------------------------------------
    #  GPIO LED FONKSİYONLARI
    # ----------------------------------------------------------
    def _led_set(self, led_num: int, state: bool):
        pin = LED_PINS.get(led_num)
        if pin is None:
            return
        GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
        self.led_states[led_num] = state

    def _toggle_led(self, led_num: int):
        new_state = not self.led_states.get(led_num, False)
        self._led_set(led_num, new_state)
        durum = "AÇIK 💡" if new_state else "KAPALI ○"
        print(f"  LED{led_num}  →  {durum}")

    # ----------------------------------------------------------
    #  ARDUINO KOMUT GÖNDERME
    # ----------------------------------------------------------
    def _send_arduino(self, cmd: str):
        if self.arduino is None:
            print("  ✖ Arduino bağlı değil.")
            return
        try:
            self.arduino.write(cmd.encode())
        except serial.SerialException as e:
            print(f"  ✖ Arduino iletişim hatası: {e}")

    # ----------------------------------------------------------
    #  G / H / I  –  Arduino analog voltaj grupları
    # ----------------------------------------------------------
    def key_g(self):
        self._send_arduino('G')
        print("  [G] LED3=3.5V  LED4=3.5V  LED5=1.49V  LED6=0V  LED7=1.49V")

    def key_h(self):
        self._send_arduino('H')
        print("  [H] LED3=1.49V  LED4=3.5V  LED5=3.5V  LED6=0V  LED7=1.49V")

    def key_i(self):
        self._send_arduino('I')
        print("  [I] LED3=1.49V  LED4=0V  LED5=1.49V  LED6=0V  LED7=1.49V")

    # ----------------------------------------------------------
    #  J  –  LED8 → 2 saniye yak, otomatik söndür
    # ----------------------------------------------------------
    def key_j(self):
        if self._led8_timer is not None and self._led8_timer.is_alive():
            self._led8_timer.cancel()
        self._led_set(8, True)
        print("  LED8  →  AÇIK 💡  (2 sn sonra söner)")
        self._led8_timer = threading.Timer(2.0, self._led8_timeout)
        self._led8_timer.daemon = True
        self._led8_timer.start()

    def _led8_timeout(self):
        self._led_set(8, False)
        print("\n  LED8  →  KAPALI ○  (otomatik söndü)")

    # ----------------------------------------------------------
    #  ANA DÖNGÜ
    # ----------------------------------------------------------
    def run(self):
        self.disable_motor()
        time.sleep(0.1)
        self.enable_motor()
        time.sleep(0.1)
        self.send_position(0)
        self.current_position = 0

        print("\n" + "="*60)
        print("  MOTOR  :  A → Sola        D → Sağa        Q → Çıkış")
        print("  ──────────────────────────────────────────────────────")
        print("  E → LED1 toggle          F → LED2 toggle")
        print("  G → LED3=3.5V LED4=3.5V LED5=1.49V LED6=0V LED7=1.49V")
        print("  H → LED3=1.49V LED4=3.5V LED5=3.5V LED6=0V LED7=1.49V")
        print("  I → LED3=1.49V LED4=0V LED5=1.49V LED6=0V LED7=1.49V")
        print("  J → LED8 (2 sn)          K → LED9 toggle")
        print("  L → LED10 toggle         M → LED11 toggle")
        print("="*60 + "\n")

        with KeyboardReader() as kb:
            while True:
                now = time.monotonic()
                key = kb.get_key()

                # ---------- Motor ----------
                if key == 'a':
                    new_pos = max(POS_MIN, self.current_position - STEP_SIZE)
                    if new_pos != self.current_position:
                        self.current_position = new_pos
                        self.send_position(self.current_position)
                        print(f"  ← Sola dönüyor  (konum: {self.current_position:+d})")
                    else:
                        print(f"  ⚠ Sol sınıra ulaşıldı ({POS_MIN})")

                elif key == 'd':
                    new_pos = min(POS_MAX, self.current_position + STEP_SIZE)
                    if new_pos != self.current_position:
                        self.current_position = new_pos
                        self.send_position(self.current_position)
                        print(f"  → Sağa dönüyor  (konum: {self.current_position:+d})")
                    else:
                        print(f"  ⚠ Sağ sınıra ulaşıldı ({POS_MAX})")

                elif key == 'q':
                    print("\n  Çıkılıyor...")
                    break

                # ---------- LED toggle (GPIO) ----------
                elif key == 'e':
                    self._toggle_led(1)

                elif key == 'f':
                    self._toggle_led(2)

                # ---------- Analog voltaj (Arduino) ----------
                elif key == 'g':
                    self.key_g()

                elif key == 'h':
                    self.key_h()

                elif key == 'i':
                    self.key_i()

                # ---------- J: 2 sn LED8 ----------
                elif key == 'j':
                    self.key_j()

                # ---------- LED toggle (GPIO) ----------
                elif key == 'k':
                    self._toggle_led(9)

                elif key == 'l':
                    self._toggle_led(10)

                elif key == 'm':
                    self._toggle_led(11)

                # ---------- Watchdog keep-alive ----------
                if now - self.last_cmd_time >= KEEPALIVE_S:
                    self.send_position(self.current_position)

    # ----------------------------------------------------------
    #  TEMİZ KAPATMA
    # ----------------------------------------------------------
    def shutdown(self):
        try:
            if self._led8_timer and self._led8_timer.is_alive():
                self._led8_timer.cancel()
            # Tüm GPIO LED'leri söndür
            for led_num in LED_PINS:
                self._led_set(led_num, False)
            GPIO.cleanup()
            # Arduino analog çıkışlarını sıfırla
            if self.arduino and self.arduino.is_open:
                self._send_arduino('X')
                time.sleep(0.1)
                self.arduino.close()
            # Motoru durdur
            self.disable_motor()
            time.sleep(0.1)
        except Exception:
            pass
        self.notifier.stop()
        self.bus.shutdown()
        print("  Bağlantılar kapatıldı.")


# ============================================================
#  GİRİŞ NOKTASI
# ============================================================
def main():
    controller = KY170Controller()

    def signal_handler(sig, frame):
        controller.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        controller.run()
    except Exception as e:
        print(f"  ✖ Beklenmeyen hata: {e}")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()