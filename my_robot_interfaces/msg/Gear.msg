# Wiadomość do przesyłania stanu biegu i sprzęgła
std_msgs/Header header
uint8 gear          # Aktualny bieg (0=Neutralny, 1-4=Biegi)
uint8 clutch_state  # Stan sprzęgła (0=Zwolnione, 1=Wciśnięte)