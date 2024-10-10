"""
Class used as the storage unit of gcode. Has functions that can add and remove parts. Will automatically
place gcode parts into correctly into the shelf and take up the correct amount of slots based on slot number
and height

Written by Andrew Viola
"""


import math
from mc2425.gcode import gcode

class partsShelf:
    def __init__(self,slotNumber: int,height: float):
        """

        :param slotNumber: Amount of slots the shelving unit has
        :param height: Total amount of height in mm
        """
        self.slotNumber = slotNumber
        self.height = height
        self.slots = [None]*self.slotNumber
        self.slotSize = self.height/self.slotNumber
        self.parts = {}

    def __str__(self):
        """
        :return: A string with the shelving unit, slot size, slot number and the current objects in the unit
        """
        return f"Part List: {self.slots}\nSlot Spacing: {self.slotSize}\nTotal Slots: {self.slotNumber}\nCurrent Objects: {list(self.parts.keys())}"

    def addPart(self,part: gcode):
        """
        When given a gcode object, add it to the shelf. Will check next available space and place the gcode
        where it fits
        :param part:
        :return: True/False and the slot number where the part starts
        """
        # Total amount of slots the current gcode will take up
        slotsTaken = math.ceil(part.getHeight()/self.slotSize)
        count = 0
        name = ""

        # Loop through all the slots
        for slot in range(0,self.slotNumber):
            count = 0
            # Check for an empty slot
            if self.slots[slot] is None:
                # If the slot number is less than the number of slots the gcode will take up, stop looking
                if self.slotNumber-slot < slotsTaken:
                    break
                count += 1
                # If gcode takes up more than one slot, loop through the remaining slots and determine
                # if there's enough space for the gcode to take up
                if count != slotsTaken:
                    for potSlot in range(slot, self.slotNumber+slot):
                        if count == slotsTaken:
                            break
                        if self.slots[potSlot] is None:
                            count += 1
                        else:
                            count = 0
                            break
                # If the count is reached, place the gcode in that location
                if count == slotsTaken:
                    duplicate = 0
                    name = part.getName()
                    base = part.getName()
                    # If this is a duplicate gcode, give it a unique name based off previous names
                    for key in list(self.parts.keys()):
                        if self.parts[key]["Base"] == name:
                            duplicate = self.parts[key]["Duplicate"] + 1
                            self.parts[key]["Duplicate"] = duplicate
                            name = key + str(duplicate)
                    self.parts[name] = {}
                    # Tell this gcode the start and end slots, this will be used for removal
                    self.parts[name]["Start"] = slot
                    self.parts[name]["End"] = slot + slotsTaken - 1
                    self.parts[name]["Duplicate"] = duplicate
                    self.parts[name]["Base"] = base
                    self.slots[slot] = name
                    for j in range(slot + 1, slotsTaken + slot):
                        self.slots[j] = name
                    break
        # Return the slot number and true if a gcode was added, else return false
        if count == 0:
            return False,None
        else:
            return True,self.parts[name]["Start"]

    def removePart(self, start: int):
       """
       Removes the gcode from the shelving unit and pops it from the gcode list
       :param start: Slot number where the gcode starts
       :return:AddPart.msg
       """
       for key in list(self.parts.keys()):
           if self.parts[key]["Start"] == start:
               for slot in range(start, self.parts[key]["End"]+1):
                   self.slots[slot] = None
               self.parts.pop(key)
               return True
       return False

    def shelfChecker(self, shelfNum: int):
        return self.slots[shelfNum]