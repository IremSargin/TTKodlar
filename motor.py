#!/usr/bin/env python3
# ============================================================
#  KY170DD01005-08G  –  Klavye ile Pozisyon Kontrolü
#  Donanım : Raspberry Pi 5 + MCP2515 (SocketCAN)
#
#  Kontroller:
#    A tuşu  → Sola hareket (pozisyon azalt)
#    D tuşu  → Sağa hareket (pozisyon artır)
#    Q tuşu  → Çıkış
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

# ============================================================
#  LOGGING
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger(__name__)

# ============================================================
#  MOTOR CAN ID TANIMLARI
# ============================================================
MOTOR_ID  = 0x01
CAN_TX_ID = 0x06000000 | MOTOR_ID   # 0x06000001
CAN_RX_ID = 0x05800000 | MOTOR_ID   # 0x05800001
CAN_HB_ID = 0x07000000 | MOTOR_ID   # 0x07000001

# ============================================================
#  POZİSYON SINIRI  (10000 birim = 1 tur)
# ============================================================
POS_MAX =  50000
POS_MIN = -50000

# ============================================================
#  KLAVYE AYARLARI
# ============================================================
CAN_INTERFACE  = "can0"
STEP_SIZE      = 1000    # Tuş başına pozisyon adımı (0.1 tur)
KEEPALIVE_S    = 0.500   # Watchdog keep-alive periyodu


# ============================================================
#  NON-BLOCKING KLAVYE OKUMA
# ============================================================
class KeyboardReader:
    """
    Terminal'i raw moda alarak bloklamadan tek tuş okur.
    'with' bloğu çıkışında terminal ayarları otomatik geri yüklenir.
    """
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
        """
        Basılı tuşu döndürür. Tuş yoksa None döner (non-blocking).
        select() ile 10ms timeout kullanır → CPU'yu boşa harcamaz.
        """
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            return sys.stdin.read(1).lower()
        return None


# ============================================================
#  KY170 MOTOR KONTROL SINIFI
# ============================================================
class KY170Controller:
    def __init__(self):
        self.current_position: int = 0
        self.motor_enabled: bool   = False
        self.last_cmd_time: float  = 0.0

        log.info(f"CAN arayüzü açılıyor: {CAN_INTERFACE}")
        self.bus = can.interface.Bus(
            channel=CAN_INTERFACE,
            interface="socketcan",
            receive_own_messages=False
        )
        self.notifier = can.Notifier(self.bus, [self._can_listener])

    # ----------------------------------------------------------
    #  CAN GÖNDERME
    # ----------------------------------------------------------
    def _send_can(self, can_id: int, data: bytes):
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
        try:
            self.bus.send(msg)
            self.last_cmd_time = time.monotonic()
        except can.CanError as e:
            log.error(f"CAN gönderme hatası: {e}")

    # ----------------------------------------------------------
    #  CAN ALICI
    # ----------------------------------------------------------
    def _can_listener(self, msg: can.Message):
        aid = msg.arbitration_id
        if aid == CAN_HB_ID and len(msg.data) >= 8:
            angle = struct.unpack(">h", msg.data[0:2])[0]
            speed = struct.unpack(">h", msg.data[2:4])[0]
            curr  = struct.unpack(">h", msg.data[4:6])[0]
            err   = struct.unpack(">H", msg.data[6:8])[0]
            log.info(f"[HB] Açı:{angle:6d}  Hız:{speed:6d}  Akım:{curr:6d}  Hata:0x{err:04X}")
        elif aid == CAN_RX_ID:
            log.info(f"[ACK] {' '.join(f'{b:02X}' for b in msg.data)}")

    # ----------------------------------------------------------
    #  MOTOR KOMUTLARI
    # ----------------------------------------------------------
    def enable_motor(self):
        self._send_can(CAN_TX_ID, bytes([0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]))
        self.motor_enabled = True
        log.info("[CAN] Motor ENABLE")

    def disable_motor(self):
        self._send_can(CAN_TX_ID, bytes([0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]))
        self.motor_enabled = False
        log.info("[CAN] Motor DISABLE")

    def send_position(self, pos: int):
        pos_u32 = pos & 0xFFFFFFFF
        b0 = (pos_u32 >>  8) & 0xFF
        b1 = (pos_u32 >>  0) & 0xFF
        b2 = (pos_u32 >> 24) & 0xFF
        b3 = (pos_u32 >> 16) & 0xFF
        self._send_can(CAN_TX_ID, bytes([0x23, 0x02, 0x20, 0x01, b0, b1, b2, b3]))
        log.info(f"[CAN] Pozisyon: {pos:+7d}  ({pos/10000:.2f} tur)")

    # ----------------------------------------------------------
    #  ANA DÖNGÜ
    # ----------------------------------------------------------
    def run(self):
        log.info("=== KY170 Klavye Kontrol Başlatılıyor ===")

        self.disable_motor()
        time.sleep(0.1)
        self.enable_motor()
        time.sleep(0.1)
        self.send_position(0)
        self.current_position = 0

        print("\n" + "="*45)
        print("  A → Sola    D → Sağa    Q → Çıkış")
        print(f"  Adım büyüklüğü: {STEP_SIZE} birim ({STEP_SIZE/10000:.1f} tur)")
        print("="*45 + "\n")

        with KeyboardReader() as kb:
            while True:
                now = time.monotonic()
                key = kb.get_key()

                if key == 'a':
                    new_pos = max(POS_MIN, self.current_position - STEP_SIZE)
                    if new_pos != self.current_position:
                        self.current_position = new_pos
                        self.send_position(self.current_position)
                    else:
                        print(f"  ⚠ Sol sınıra ulaşıldı ({POS_MIN})")

                elif key == 'd':
                    new_pos = min(POS_MAX, self.current_position + STEP_SIZE)
                    if new_pos != self.current_position:
                        self.current_position = new_pos
                        self.send_position(self.current_position)
                    else:
                        print(f"  ⚠ Sağ sınıra ulaşıldı ({POS_MAX})")

                elif key == 'q':
                    print("\n  Çıkılıyor...")
                    break

                # Watchdog keep-alive
                if now - self.last_cmd_time >= KEEPALIVE_S:
                    self.send_position(self.current_position)

    # ----------------------------------------------------------
    #  TEMİZ KAPATMA
    # ----------------------------------------------------------
    def shutdown(self):
        log.info("Kapatılıyor...")
        try:
            self.disable_motor()
            time.sleep(0.1)
        except Exception:
            pass
        self.notifier.stop()
        self.bus.shutdown()
        log.info("Bağlantılar kapatıldı.")


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
        log.error(f"Beklenmeyen hata: {e}")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
