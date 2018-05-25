# Message for raw user controller input.

bool start
bool back

bool trig_l  # Left trigger down?
bool trig_r  # Right trigger down?

float64 trig_l_val # Left trigger down?
float64 trig_r_val # Right trigger down?

bool bump_l  # Left bumper down?
bool bump_r  # Right bumper down?

bool but_x   # "X" button has been triggered?
bool but_y   # "Y" button has been triggered?
bool but_a   # "A" button has been triggered?
bool but_b   # "B" button has been triggered?

float64 axis_lx  # Normalised left stick x, -1 to 1
float64 axis_ly  # Normalised left stick y, -1 to 1

float64 axis_rx  # Normalised right stick x, -1 to 1
float64 axis_ry  # Normalised right stick y, -1 to 1

int8 axis_dx  # d-pad x, -1 left, 0 none/both or 1 right
int8 axis_dy  # d-pad y, -1 down, 0 none/both or 1 up
