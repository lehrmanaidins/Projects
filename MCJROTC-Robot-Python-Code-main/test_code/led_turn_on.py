
# Import modules
from pitop.pma import LED  # LED control module
from time import sleep  # Allows us to tell the program to wait

# Set up the LED on port D1 of the foundation plate
led = LED("D1")

# Now for the algorithm
led.on()
sleep(4)
led.off()
