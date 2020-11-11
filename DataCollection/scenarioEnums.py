from enum import Enum

class TestCase:
    def __init__(self, size, temp, color, distance):
        self.size = size
        self.objectTemperature = temp
        self.objectColor = color
        self.distance = distance
        

class Size(Enum):
    MM40 = 1
    MM80 = 2
    MM160 = 3
    NA = 4

class ObjectTemperature(Enum):
    C40 = 1
    C45 = 2
    C50 = 3
    Ambient = 4
    NA = 5

class ObjectColor(Enum):
    Black = 1
    RoseGold = 2
    Blue = 3
    Silver = 4
    White = 5

class Distance(Enum):
    FT2 = 1
    FT4 = 2
    FT6 = 3


