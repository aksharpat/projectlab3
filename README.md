# Smoothing Function Control
To change how the smoothing works, there are 2 important values to change:
- fingerAccel: Changes how fast the fingers accelerate; increasing this will likely lessen the smoothing effect
- maxVel: Changes the maximum velocity the fingers can move at; this may need to be increased from where it's currently set (50)

Each accel/velocity value operates in 1/100ths of a servo value, similar to the base of the arm (though the base only moves 10/100ths of a servo per loop). This could be made modular but I do not feel like doing that :)
