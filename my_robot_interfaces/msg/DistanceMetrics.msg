# my_robot_interfaces/msg/DistanceMetrics.msg
std_msgs/Header header

# Odległość w linii prostej (euklidesowa) między pojazdami w metrach.
float64 distance_straight

# Odległość wzdłużna (wzdłuż osi jazdy sieczkarni) w metrach.
# Wartość dodatnia oznacza, że ciągnik jest z przodu sieczkarni.
# Wartość ujemna oznacza, że ciągnik jest z tyłu sieczkarni.
float64 distance_longitudinal

# Odległość poprzeczna (prostopadła do osi jazdy sieczkarni) w metrach.
# Wartość dodatnia oznacza, że ciągnik jest po prawej stronie sieczkarni.
# Wartość ujemna oznacza, że ciągnik jest po lewej stronie sieczkarni.
float64 distance_lateral