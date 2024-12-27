; Example G-code
G21 ; Set units to millimeters
G90 ; Use absolute positioning
G28 ; Home all axes

; Move to start position
G1 X10 Y10 F1500

; Draw a square
G1 X50 Y10 F1500
G1 X50 Y50 F1500
G1 X10 Y50 F1500
G1 X10 Y10 F1500

; End of program
M2 ; End of program