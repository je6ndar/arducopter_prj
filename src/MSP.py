from pymultiwii import MultiWii

serialPort = "/dev/serial0"
board = MultiWii(serialPort)
while True:
    (board.getData(MultiWii.ATTITUDE))

