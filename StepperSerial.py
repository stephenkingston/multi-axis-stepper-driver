import serial


def sendCommandToMCU(s1_position, s2_position, s3_position, speed_factor):
    s1_a, s1_b = s1_position.to_bytes(2, 'big')
    s2_a, s2_b = s2_position.to_bytes(2, 'big')
    s3_a, s3_b = s3_position.to_bytes(2, 'big')
    ser.write([s1_a, s1_b, s2_a, s2_b, s3_a, s3_b, speed_factor])


ser = serial.Serial('COM4', 230400, serial.EIGHTBITS,
                    serial.PARITY_NONE, serial.STOPBITS_ONE)

while 1:
    try:
        a, b, c = list(map(int, input('Enter positions: ').split()))
        sendCommandToMCU(a, b, c, 2)
    except ValueError:
        pass
