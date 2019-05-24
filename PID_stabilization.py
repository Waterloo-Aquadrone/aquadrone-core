from simple_pid import PID;

# the robot model that the program was programmed for
# A, B, C, D represent the bottom motors, and they are related to the front of the robot
# X axis runs front to back, (roll), Y axis goes left side to right side (pitch), Z axis goes top to bottom (yaw)
#
#                   E
#  -----------------------------------
#   B                               A
#
#                                           Front of Robot
#
#
#   D                               C
#  -----------------------------------
#                   F

class RotController:
    
    #ideal positions for x, y, z rotations
    xGoal = 0.0
    yGoal = 0.0
    zGoal = 0.0

    #input values for x, y, z axis directions
    #order goes x, y, z
    input = (0.0, 0.0, 0.0)

    #power values (-1 to 1) for motors ABCD
    aPow = 0.0
    bPow = 0.0
    cPow = 0.0
    dPow = 0.0
    ePow = 0.0
    fPow = 0.0

    #P, I, D values for x, y, z
    xValues = (0.0, 0.0, 0.0)
    yValues = (0.0, 0.0, 0.0)
    zValues = (0.0, 0.0, 0.0)

    #power values to assign to each ABCD for x, y, z axis corrections (for calculations only)
    AforX = 0.0
    BforX = 0.0
    CforX = 0.0
    DforX = 0.0

    AforY = 0.0
    BforY = 0.0
    CforY = 0.0
    DforY = 0.0

    EforZ = 0.0
    FforZ = 0.0

    #reverse settings (just in case I guessed wrong and the stabilization works the wrong way)
    xRev = 1
    yRev = 1
    zRev = 1


    #pid objects
    rollPID = PID(xValues, xGoal, 0.01, (-1,1), True, False)
    pitchPID = PID(yValues, yGoal, 0.01, (-1,1), True, False)
    yawPID = PID(zValues, zGoal, 0.01, (-1,1), True, False)
        
    #LOOP FUNCTIONS (THEY ARE TO BE USED IN A LOOP FOR THE PID STUFF)

        #outputs the pid outputs for the given inputs
    def getMotorControls(inputs):
        inputX, inputY, inputZ = inputs
        return (rollPID(inputX), pitchPID(inputY), yawPID(inputZ))

        #sets the axis independent motor values using the pid outputs
    def setAxisMotorPowers(controls):
        ctrlX, ctrlY, ctrlZ = controls

        #roll settings: when looking at the robot along the x axis, back to front, positive is ccw rotation
        AforX = BforX = ctrlX*xRev
        CforX = DforX = -ctrlX*xRev

        #pitch settings: nose up is positive
        AforY = CforY = ctrlY*yRev
        BforY = DforY = -ctrlY*yRev

        #yaw settings: top down view, ccw is positive
        FforZ = ctrlZ*zRev
        EforZ = -ctrlZ*zRev

        #returns values -1 to 1 as a array list thingy idk what this is
    def getMotorPowers(self):
        return((AforX + AforY)/2, (BforX + BforY)/2, (CforX + CforY)/2, (DforX + DforY)/2, EforZ, FforZ)

        #allows for things to set the input values for the x, y, z axis
    def setInputs(inputX, inputY, inputZ):
        input = (inputX, inputY, inputZ)

    def fullPID(roll, pitch, yaw):

        setInputs(roll, pitch, yaw)
        setAxisMotorPowers(getmotorPowers(input))
        return(getMotorPowers())


        

