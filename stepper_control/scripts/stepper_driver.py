import serial
import rclpy
import time

MICROSTEP = 6400

class StepperMotor:
    __port = ''
    __ser = serial.Serial(timeout=1, baudrate=115200)

    def __init__(self, serial_port_):
        StepperMotor.__port = serial_port_

    def openSerialPort(self):
        StepperMotor.__ser.port = StepperMotor.__port
        try:
            StepperMotor.__ser.open()
            rclpy.logging._root_logger.info('Opened the port ' + StepperMotor.__port)
        except Exception as e:
            rclpy.logging._root_logger.fatal('Cannot open the port ' + StepperMotor.__port + ': ' + str(e))
            rclpy.shutdown()

    def closeSerialPort(self):
        StepperMotor.__ser.close()

    def getMotorAcceleration(self):
        StepperMotor.__ser.write(b'A')
        self.d = StepperMotor.__ser.readline()
        rclpy.logging._root_logger.debug('A' + str((self.d)) + '\r')
        self.p = self.d.find(b'A')
        self.q = self.d.find(b' ')
        self.d = self.d[self.p + 1 :]
        self.d = float(self.d) - 65.0
        return self.d

    def setMotorAcceleration(self, accel):
        rclpy.logging._root_logger.debug('A' + str(float(accel)) + '\r')
        StepperMotor.__ser.reset_output_buffer()
        try:
            StepperMotor.__ser.write(('A' + str(float(accel))).encode())
            StepperMotor.__ser.reset_input_buffer()
            res = True
        except Exception as e:
            res = False
            rclpy.logging._root_logger.error('Failed to set motor acceleration: ' + str(e))
        return res

    def getMotorSpeed(self):
        StepperMotor.__ser.write(b'S')
        self.d = StepperMotor.__ser.readline()
        self.p = self.d.find(b'S')
        self.q = self.d.find(b' ')
        self.d = self.d[self.p + 1 :]
        self.d = float(self.d) - 83.0
        return self.d

    def setMotorSpeed(self, speed):
        rclpy.logging._root_logger.debug('S' + str(int(speed)) + '\r')
        StepperMotor.__ser.reset_output_buffer()
        try:
            StepperMotor.__ser.write(('S' + str(int(speed)) + '\r').encode())
            StepperMotor.__ser.reset_input_buffer()
        except Exception as e:
            rclpy.logging._root_logger.error('Failed to set motor speed: ' + str(e))

    def setAbsolutePosition(self, position1, position2, position3):
        try:
            rclpy.logging._root_logger.debug('G' + (str(position1) + 'A') + (str(position2) + 'B') + (str(position3) + 'C'))
            StepperMotor.__ser.reset_output_buffer()
            StepperMotor.__ser.write(('G' + (str(position1) + 'A') + (str(position2) + 'B') + (str(position3) + 'C')).encode())
            StepperMotor.__ser.reset_input_buffer()
        except Exception as e:
            rclpy.logging._root_logger.error('Failed to set absolute position: ' + str(e))

    def autoCalibrate(self):
        try:
            StepperMotor.__ser.write(b'X')
            return True
        except Exception as e:
            rclpy.logging._root_logger.error('Failed to auto calibrate: ' + str(e))
            return False

    def isAvailable(self):
        StepperMotor.__ser.reset_input_buffer()
        StepperMotor.__ser.write(b'X')
        time.sleep(1)

    def testTriangle(self, cnt):
        StepperMotor.__ser.reset_input_buffer()
        StepperMotor.__ser.write(('T' + str(cnt)).encode())
        time.sleep(1)

    def getMotorMicrostep(self):
        return MICROSTEP
