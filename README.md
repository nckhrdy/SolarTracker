# Solar Tracker

Authors: Marybel Boujaoude, Riya Deokar, Hassan Hijazi, Nick Hardy

Date: 2023-02-07

### Summary
In this quest, our team created a solar tracker that uses a HUZZAH32 ESP32 Feather Board with a built-in hardware timer and two connected servos to measure azimuth and altitude. In a five second sampling period, each servo moved one at a time to scan the area both horizontally and vertically. The photoresistor measured the voltage, and the servos adjusted to position towards the highest reading. The angles of the servos are then displayed on an alphanumeric display.

Investigative question:
An approach that we could use to make setting the date and time not hard coded is to use a real-time clock module (RTC). An RTC has a backup power source that allows it to keep running even if the power from the main system was turned off in order for it to keep accurate track of the time. We could create a user interface that allows the user to enter the values and they would remain stored in the RTC so it wouldn't have to be hard coded into our program.

We also had a control loop for 2 servo motors connected to a microcontroller in servos.c. It starts by rotating the first servo motor and measuring the voltage output at each step, storing the maximum voltage and corresponding angle. After 45 steps, it sets the servo to the angle with the maximum voltage and starts rotating the second servo in the same manner. Then, it enters an infinite loop where it continuously moves the servos within their range and checks if the angle has gone beyond 90 degrees or below -90 degrees. If so, it resets the angle to its maximum or minimum position. The loop alternates between the two servos, moving one at a time and measuring the voltage output. The loop increases and decreases the servo angle in 2 degree increments, with a counter to keep track of the steps. Once the counter reaches 10 or 20, the loop switches to the other servo and starts over.

### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1 |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 1 |  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1 |  1     | 
| Objective Seven | 1 |  1     | 


### Solution Design
In order to create this solar tracker, we used a hardware timer, two servos, a photoresistor, and an alphanumeric display. The timer had a five second sampling period and was initialized and implemented in our code based on the example provided to us from GitHub. The two servos were initialized separately so they could operate one at a time. The two servos did an initial scan to determine the maximum reading. Servo one moved horizontally 90 degrees in both the negative and positive direction while servo two moved vertically 90 degrees in both directions as well. After the initial scan, the servos move increasing by two degrees in each direction. Once it does a complete scan, the servos will move to the position with the highest reading of the photoresistor. The photoresistor is connected to pin 34 which is an ADC channel of the microcontroller. This takes the analog signal from the photoresistor and converts it into a digital signal, allowing the servos to read and move to the value. The alphanumeric displays the angle of the servos at the maximum voltage position.  

### Sketches/Diagrams
<p align="center">
<img src="./images/ece444.png" width="50%">
</p>
<p align="center">
Caption Here
</p>

![Untitled%20(Draft)](https://user-images.githubusercontent.com/91172956/217436042-b1dab42a-fb5a-4b41-9d82-fb21720a85c9.jpeg)

### Supporting Artifacts
- Link to video technical presentation: 
https://drive.google.com/file/d/1VUlubKEE6qse6-dDDOxeWTZh9MMyZwBB/view?usp=sharing

- Link to video demo and image
https://drive.google.com/file/d/19zh1denSAlujiloJcP3Fj55d0I6KpRgW/view?usp=sharing

https://drive.google.com/file/d/1J45UPDdOR7cjlR8RxE9IdXQHDRQ0kWsO/view?usp=sharing

### Modules, Tools, Source Used Including Attribution
HUZZAH32 ESP32 Feather Board, idf.py, xcode (C programming language)

### References
STL files: 
[Solar Tracker STL.zip](https://github.com/BU-EC444/Team1-Boujaoude-Deokar-Hijazi-Hardy/files/10676818/Solar.Tracker.STL.zip)

I2C display example code: 
https://github.com/BU-EC444/04-Code-Examples/blob/main/i2c-display/main/i2c_display.c

Timer example code:
https://github.com/BU-EC444/04-Code-Examples/blob/main/timer-example/main/timer-example.c

Servo example code: 
https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_servo_control/main/mcpwm_servo_control_example_main.c
