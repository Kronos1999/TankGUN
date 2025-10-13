# TankGUN
PS3 controlled Tank with a Nurf gun on top
the Cannon file has the three action files as sub folders. These are dirived from 3action_nurf and what I got from Drone bot work shop. PS3 controller. 
the only file I have gotten to work is the 3action_nurf. that did work at one time and then I fried the UNOR3 it was attached to. 

I haven't work on this in about a year. but I did get Drone bots code to kinda work on an esp32. But what happened was that the esp32 freaked out all the servos and broke things. I thought it was sending mixed PWM signals. So I went down a rabit hole of SPI comunication and cutting up the code into individual bytes and trying to have a master with the PS3 library and many slaves creating the PWM signals for each function. So it's been almost a year and I am overwhelmed with the thought of even starting this project again. oye!!!!

I've read the three actions. I feel like there is something missing from the start up process in the drone motors. I need to go back to the code that was written when I was getting them to work. 
Found it this needs to be in the start up code. 
    // Initialize the signals to 1000 (original direction)
esc1.writeMicroseconds(1000);
esc2.writeMicroseconds(1000);
