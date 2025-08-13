# Wiadomość do szybkiej wizualizacji stanu regulatora PI
std_msgs/Header header

# Prędkość, do której dąży regulator
float64 setpoint_speed

# Aktualna, zmierzona prędkość pojazdu
float64 current_speed

# Wyjście z regulatora (sygnał sterujący)
float64 control_output
