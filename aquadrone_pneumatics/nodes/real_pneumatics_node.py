from aquadrone_pneumatics import Pneumatics

if __name__ == "main":
    pneumatics_contoller = Pneumatics(real=True)
    pneumatics_contoller.run()