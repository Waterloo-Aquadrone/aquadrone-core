from abc import ABC, abstractmethod

class CenterStrategyBase(ABC):
    def __init__(self):
        super().__init__()


    @abstractmethod
    def findCenter(self, image):
        pass
