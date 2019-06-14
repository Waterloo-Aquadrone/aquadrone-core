import BNO005

bno = BNO055.BNO055()

def read_euler():
  return bno.read_euler()