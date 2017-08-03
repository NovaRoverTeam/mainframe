# Compact message for user controller input.
#
# Type:
#       0: Button
#       1: Stick
#       2: Trigger

bool depress    # True if trigger depressed, false if released
bool trigger_left  # True if left trigger

# Added these on 07/05/17 *****************************
float64 axis_x  # Normalised stick x
float64 axis_y  # Normalised stick y
