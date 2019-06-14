from Adafruit_BNO005 import BNO005

bno = BNO005.BNO055()

def read_euler():
  return bno.read_euler()
