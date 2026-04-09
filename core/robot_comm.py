import serial
import serial.tools.list_ports
import threading
import time

class RobotController:
    def __init__(self, baudrate=115200):
        self.serial_port = None
        self.baudrate = baudrate
        self.is_listening = False
        self.on_message_received = None # Callback function

    def get_available_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port):
        try:
            self.serial_port = serial.Serial(port, self.baudrate, timeout=1)
            time.sleep(2) # Wait for Arduino reset
            self.is_listening = True
            threading.Thread(target=self._listen, daemon=True).start()
            return True
        except Exception as e:
            print(f"Connect error: {e}")
            return False

    def disconnect(self):
        self.is_listening = False
        time.sleep(0.2)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def send_command(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            if not cmd.endswith('\n'):
                cmd += '\n'
            self.serial_port.write(cmd.encode())
            return True
        return False

    def _listen(self):
        while self.is_listening and self.serial_port:
            try:
                if self.serial_port.in_waiting:
                    resp = self.serial_port.readline().decode(errors='ignore').strip()
                    if resp and self.on_message_received:
                        self.on_message_received(resp)
            except Exception as e:
                if self.is_listening:
                    print(f"Serial error: {e}")
            time.sleep(0.01)

    def move_to(self, x, y, z, speed=500):
        return self.send_command(f"G1 X{x:.1f} Y{y:.1f} Z{z:.1f} F{speed}")

    def home(self):
        return self.send_command("G28")

    def disable_motors(self):
        return self.send_command("M18")
