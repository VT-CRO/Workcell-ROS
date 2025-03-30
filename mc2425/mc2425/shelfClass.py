"""
Class used as the storage unit of gcode. Has functions that can add and remove parts. Will automatically
place gcode parts into correctly into the shelf and take up the correct amount of slots based on slot number
and height

Written by Andrew Viola
"""


import math
import os
import json
from mc2425.gcode import gcode

class partsShelf:
    def __init__(
        self, slotNumber: int, height: float, database: str = "shelf.json"
    ):
        """
        Initializes the parts shelf.

        :param slotNumber: Amount of slots the shelving unit has
        :param height: Total amount of height in mm
        :param database: Path to the JSON file for persistence
        """
        self.database = database
        needNewDatabase = True
        if os.path.exists(database):
            try:
                with open(database, "r") as f:
                    data = json.load(f)
                # Basic validation of loaded data
                if (
                    isinstance(data, dict)
                    and data.get("slotNumber") == slotNumber
                    and data.get("height") == height
                    and "slotSize" in data
                    and "slots" in data
                    and "parts" in data
                    and len(data.get("slots", [])) == slotNumber
                ):
                    self.slotNumber = data["slotNumber"]
                    self.height = data["height"]
                    self.slotSize = data["slotSize"]
                    self.slots = data["slots"]
                    self.parts = data["parts"]
                    needNewDatabase = False
                    print(f"Loaded shelf state from '{database}'")
                else:
                    print(
                        f"Database '{database}' exists but has incompatible data or structure. Reinitializing."
                    )
            except (json.JSONDecodeError, TypeError, KeyError) as e:
                print(
                    f"Error reading or validating database '{database}': {e}. Reinitializing."
                )

        if needNewDatabase:
            self.slotNumber = slotNumber
            self.height = height
            if self.slotNumber > 0:
                # Ensure height is treated as float for division
                self.slotSize = float(self.height) / self.slotNumber
            else:
                self.slotSize = 0  # Avoid division by zero
            self.slots = [None] * self.slotNumber
            self.parts = {}
            print(f"Initialized new empty shelf. Saving to '{database}'")
            self._save()  # Save the initial empty state

    def __str__(self):
        """
        :return: A string representation of the shelf state.
        """
        # Format slots for better readability, showing index
        slot_repr = "\n".join(
            [f"  [{i}]: {self.slots[i] or 'Empty'}" for i in range(self.slotNumber)]
        )
        # Ensure slotSize is formatted correctly, handle potential zero division
        slot_size_str = f"{self.slotSize:.2f}" if self.slotSize else "N/A"
        return (
            f"Total Slots: {self.slotNumber}\n"
            f"Total Height: {self.height} mm\n"
            f"Slot Spacing: {slot_size_str} mm\n"
            f"Current Objects: {list(self.parts.keys())}\n"
            f"Shelf Layout (Index 0 is first/lowest):\n{slot_repr}"
        )

    def addPart(self, part: gcode):
        """
        Adds a gcode part to the shelf. Finds the first available block of
        slots starting from the lowest index (index 0) upwards.

        :param part: gcode object to add.
        :return: Tuple (success: bool, start_slot_index: int | None).
                 start_slot_index is the lowest index the part occupies.
        """
        if self.slotSize <= 0:
            print(
                "Error: Cannot add parts, slot size is zero or negative (check height/slotNumber)."
            )
            return False, None
        if not hasattr(part, 'getHeight') or not hasattr(part, 'getName'):
             print(f"Error: Provided part object {part} is not a valid gcode object.")
             return False, None

        part_height = part.getHeight()
        if part_height <= 0:
             print(f"Warning: Part '{part.getName()}' has zero or negative height ({part_height}mm). Skipping.")
             return False, None # Cannot place a part with no height

        slotsTaken = math.ceil(part_height / self.slotSize)

        if slotsTaken > self.slotNumber:
            print(
                f"Error: Part '{part.getName()}' ({part_height}mm, needs {slotsTaken} slots) "
                f"is too tall for the shelf ({self.height}mm, {self.slotNumber} slots)."
            )
            return False, None


        found_spot = False
        placement_start_slot = -1 # The lowest index where the part will start

        # Iterate from the bottom slot (lowest index 0) upwards
        for slot in range(self.slotNumber):
            # 'slot' is the potential *lowest* index for the part

            # Check if this slot is available AND if there's enough room above
            if self.slots[slot] is None and (slot + slotsTaken <= self.slotNumber):
                # Assume it fits and verify the required slots upwards
                fits = True
                for i in range(slotsTaken): # Check slots slot, slot+1, ..., slot+slotsTaken-1
                    check_slot_index = slot + i
                    if self.slots[check_slot_index] is not None:
                        fits = False
                        break # This potential spot doesn't work, continue outer loop

                if fits:
                    # Found a suitable spot!
                    placement_start_slot = slot
                    found_spot = True
                    break # Stop searching once the first suitable spot is found

        if found_spot:
            # Calculate start (lowest index) and end (highest index)
            start_index = placement_start_slot
            end_index = placement_start_slot + slotsTaken - 1

            # --- Handle duplicate names ---
            base_name = part.getName()
            current_name = base_name
            duplicate_counter = 0
            existing_keys = list(self.parts.keys()) # Get keys once for efficiency

            # Find the highest existing duplicate number for this base name
            max_existing_dup_num = -1
            # Check if the exact base name exists
            if base_name in existing_keys:
                max_existing_dup_num = 0 # Base name exists, next dup is at least 1

            # Check for names like base_name<number>
            for key in existing_keys:
                if key.startswith(base_name):
                    num_part = key[len(base_name):]
                    if num_part.isdigit():
                         max_existing_dup_num = max(max_existing_dup_num, int(num_part))

            # Determine the actual name and duplicate number for the new part
            if max_existing_dup_num >= 0: # If base exists or base<num> exists
                 duplicate_counter = max_existing_dup_num + 1
                 current_name = f"{base_name}{duplicate_counter}"
                 # Ensure the generated name is truly unique (highly unlikely collision, but safe)
                 while current_name in existing_keys:
                     duplicate_counter += 1
                     current_name = f"{base_name}{duplicate_counter}"
            # --- End Handle duplicate names ---

            # Store part information
            self.parts[current_name] = {
                "Start": start_index, # Lowest index
                "End": end_index,     # Highest index
                "Duplicate": duplicate_counter, # The number used (0 if original)
                "Base": base_name,
            }

            # Fill the slots in the self.slots list
            for i in range(slotsTaken):
                fill_slot_index = start_index + i # Fill from bottom up
                self.slots[fill_slot_index] = current_name

            self._save()
            print(
                f"Part '{current_name}' added, occupying slots {start_index} to {end_index}."
            )
            return True, start_index
        else:
            # This message is reached only if the loop completes without finding a spot
            print(
                f"Error: No suitable contiguous space found for part '{part.getName()}' "
                f"(needs {slotsTaken} slots)."
            )
            return False, None

    def removePart(self, start: int):
        """
        Removes the gcode from the shelving unit based on its starting slot index.

        :param start: The lowest slot number (index) where the gcode starts.
        :return: True if removal was successful, False otherwise.
        """
        if not isinstance(start, int) or start < 0:
             print(f"Error: Invalid start slot index '{start}'. Must be a non-negative integer.")
             return False

        part_key_to_remove = None
        # Find the part key associated with the given start index
        for key, data in self.parts.items():
            # Ensure 'Start' key exists before accessing
            if data.get("Start") == start:
                part_key_to_remove = key
                break

        if part_key_to_remove:
            part_data = self.parts[part_key_to_remove]
            start_idx = part_data.get("Start", -1)
            end_idx = part_data.get("End", -1)

            # Validate indices before proceeding
            if start_idx == -1 or end_idx == -1 or end_idx < start_idx:
                 print(f"Error: Corrupted part data for '{part_key_to_remove}'. Cannot remove.")
                 # Optionally, attempt to clean up self.parts here
                 # self.parts.pop(part_key_to_remove, None)
                 # self._save()
                 return False

            # Clear the slots occupied by this part
            for slot_index in range(start_idx, end_idx + 1):
                # Boundary check for safety
                if 0 <= slot_index < self.slotNumber:
                    if self.slots[slot_index] == part_key_to_remove:
                         self.slots[slot_index] = None
                    else:
                        # This indicates an inconsistency, log a warning
                        print(f"Warning: Slot {slot_index} expected part '{part_key_to_remove}' but found '{self.slots[slot_index]}'. Clearing anyway.")
                        self.slots[slot_index] = None
                else:
                     print(f"Warning: Attempted to clear invalid slot index {slot_index} while removing '{part_key_to_remove}'.")


            # Remove the part from the dictionary
            removed_part_info = self.parts.pop(part_key_to_remove)
            print(f"Part '{part_key_to_remove}' removed from slots {start_idx} to {end_idx}.")
            self._save()
            return True
        else:
            print(f"Error: No part found starting at slot index {start}.")
            return False

    def clearShelf(self):
        """
        Clears the shelf and resets the parts list.
        """
        self.slots = [None] * self.slotNumber
        self.parts = {}
        self._save()
        print("Shelf cleared.")

    def shelfChecker(self, shelfNum: int):
        """
        Checks the content of a specific shelf slot.

        :param shelfNum: The index of the slot to check.
        :return: The name of the part in the slot, or None if empty or invalid index.
        """
        if 0 <= shelfNum < self.slotNumber:
            return self.slots[shelfNum]
        else:
            print(f"Error: Invalid shelf number {shelfNum}. Must be between 0 and {self.slotNumber - 1}.")
            return None # Indicate invalid index

    def _save(self):
        """Saves the current state of the shelf to the JSON database."""
        data_to_save = {
            "slotNumber": self.slotNumber,
            "height": self.height,
            "slotSize": self.slotSize,
            "slots": self.slots,
            "parts": self.parts,
        }
        try:
            with open(self.database, "w") as f:
                json.dump(data_to_save, f, indent=4) # Add indentation
        except IOError as e:
            print(f"Error saving database to '{self.database}': {e}")
        except TypeError as e:
             # This might happen if data in self.parts is not JSON serializable
             print(f"Error serializing data for saving: {e}")
             print("Data causing error:", data_to_save)