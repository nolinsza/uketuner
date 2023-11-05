# Ukulele tuner 

This is a fully functional ukulele tuner. 

[Demonstration - YouTube Link]()
<br>  

## Project Overview

There are 4 buttons corresponding to each string on the ukulele. The user selects a button and strums the desired string. The ukulele tuner then displays one of the 3 leds indicating if the ukulele needs to be tuned down, tuned up, or is in tune. 

How it functions
* When a button is pressed an interrupt is triggered resulting in the corresponding function setting the value of the target frequency
* Returns to normal operation storing a series of microphone voltages
* Fast Fourier Transform is used to find the largest frequency using the series of stored amplitudes
* The largest frequency is compared to the target frequency triggering the corresponding led
* The process repeats as normal until a different button is pressed 
