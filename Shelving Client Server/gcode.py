"""
Class used for gcode handling. Provides all necessary information in relation to a gcode

Written by Andrew Viola
"""


import math

class gcode:

    def __init__(self, name: str, author: str, height: float):
        """
        Struct to house all the information related to a single gcode file
        :param name: Name of the gcode
        :param author: Author of the gcode
        :param height: Height of the gcode
        """
        self.author = author
        self.height = math.ceil(float(height))
        self.name = name

    def getName(self):
        """
        :return: Name of the gcode
        """
        return self.name

    def getAuthor(self):
        """
        :return: Author of the gcode
        """
        return self.author

    def getHeight(self):
        """
        :return: Height of the gcode
        """
        return self.height