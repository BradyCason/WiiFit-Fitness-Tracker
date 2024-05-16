# WiiFit-Fitness-Tracker
This was a project I made in ENGR 7B at UCI during the 2024 Winter Quarter. I led
an Engineering team of 5 people to design and fabricate a device that counted steps and flights
climbed, and displayed the results on an app. Our budget was $50.

We used an Arduino Wemos board, connected to an accelerometer and barometer for the measurments.
We also took user input from a button, and displayed our results on an OLED display along with a
piezo buzzer.

My main focus in the project was the C++ programing. I wrote all of the logic for the team,
including input from sensors, output to devices and the app, calculation of steps and flights,
calculating the steady state of the accelerometer, and more.

# FitnessTrackerWemosCode.ino
This is the main code for the tracker. You must be able to connect to the Blynk app through a local Wifi to use.

# FitnessTrackerWemoseCodeNoBlynk.ino
This is backup code that has all of the functions except for connecting to the Blynk App through Wifi. I made this so that I could use the fitness tracker outside of the lab.

# Design Video
Here is the link to a video that one of my teammates made that shows our design process: https://www.youtube.com/watch?v=Adl9SaT-O24
