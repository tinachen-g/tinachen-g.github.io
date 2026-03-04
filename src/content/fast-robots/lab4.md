---
title: Lab 4
description: Motors and Open Loop Control
pubDate: 2026-03-04
---
<article class="article">

## Prelab

### Dual Motors & Pin Choice
Diagram with your intended connections between the motor drivers, Artemis, and battery (with specific pin numbers)

Since not every pin the Artemis can give PWM signal, I had to be careful to choose which pins I used for control.
In the [Redboard Artemis Nano pin documentation](https://cdn.sparkfun.com/assets/5/5/1/6/3/RedBoard-Artemis-Nano.pdf), any pin marked with a tilde symbol ~ is a PWN pin. Therefore, taking this and the physical placements of my board, I decided to use A0-A3.


### Battery 

Our DC motors have a lot of electrical noise (EMI) due to the fast switching & high current PWM signals which can disrupt our sensitive electronics such as the Artemis. To prevent this noise from affecting our microcontroller, we must power the Artemis and the motor drivers/motors from separate batteries.

The Artemis will be powered by the 750mAh battery and the motor drivers & motors are powered by the second 850mAh battery. 

## Motor Drivers & Oscilloscope Testing

To test, the motor driver are powered from an external power supply in place of using the 850mAh battery.

In the [DRV8833 motor driver documentation](https://www.ti.com/lit/ds/symlink/drv8833.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1740659196269), the power supply voltage range is from 2.7 to 10.8 V. Since the voltage of the 850mAh battery is 3.7V, the output of the power supply was set to 3.7V to represent the battery that will eventually power the motors.

```c
#define MOTOR1_PIN1 0  // PWM-capable pin
#define MOTOR1_PIN2 1  // PWM-capable pin

void setup() {
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
}

void loop() {
  digitalWrite(MOTOR1_PIN2, LOW);  

  analogWrite(MOTOR1_PIN1, 255);  // full ON
  delay(200);

  analogWrite(MOTOR1_PIN1, 0);    // full OFF
  delay(100); 

  analogWrite(MOTOR1_PIN1, 100);  // ~39% duty cycle
  delay(200);

  analogWrite(MOTOR1_PIN1, 0);    // full OFF
  delay(100);

}
```

One pin was set to switch between HIGH, less than HIGH, and LOW, while the other pin had an output of 0. This was repeated for both motor drivers, so four pins in total. If the PWM value is less than 255, the signal repeatedly switches between HIGH and LOW. For example, an `analogWrite(pin, 100)` results in a ~39% duty cycle where it is HIGH for 39% of the period LOW for 61% of the period.


Below is a close up of the oscilloscope and the correspond video of the setup.
![Oscilloscope output for a motor driver, alternating](../../../public/fast-robots/lab4/pwm1)

[![Oscilloscope setup, alternating](https://img.youtube.com/vi/-NDx379X860/0.jpg)](https://www.youtube.com/watch?v=-NDx379X860)

The following is another example of a different PWM signal value less than 255, zoomed in to see the clear switching. 
![Oscilloscope output for a motor driver, zoomed in](../../../public/fast-robots/lab4/pwm2)

This shows that power can be regulated on the motor driver output. 

### A New Spin on the Situation
I then connected the two drivers to the provided wheels. First, I connected a single motor driver to one side of the wheels and tested the behavior with the above code to show that I can run the motor in both directions. 

```c
#define MOTOR1_PIN1 0
#define MOTOR1_PIN2 1 

void setup() {
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
}

void loop() {

  analogWrite(MOTOR1_PIN1, 200);
  delay(500);

  analogWrite(MOTOR1_PIN1, 0);
  delay(2000); 

  analogWrite(MOTOR1_PIN2, 200); 
  delay(500);

  analogWrite(MOTOR1_PIN2, 0);
  delay(2000);

}
```

[![One side of car spinning both directions]](https://img.youtube.com/vi/H4CORc9SBE8/0.jpg)(https://www.youtube.com/watch?v=H4CORc9SBE8)

---

After confirming the first set worked, I then connected the second second motor driver and confirmed that both wheels can spin, with the battery driving the motor drivers. 


```c
#define MOTOR1_PIN1 0  // A1 IN, B1 IN: right
#define MOTOR1_PIN2 1  // A2 IN, B2 IN: right
#define MOTOR2_PIN1 2  // A1 IN, B1 IN: left
#define MOTOR2_PIN2 3  // A2 IN, B2 IN: left

void setup() {
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
}

void forward() {
  analogWrite(MOTOR1_PIN1,150); 
  analogWrite(MOTOR1_PIN2,0);
  analogWrite(MOTOR2_PIN1,150); 
  analogWrite(MOTOR2_PIN2,0);
}
void backward() {
  analogWrite(MOTOR1_PIN1,0); 
  analogWrite(MOTOR1_PIN2,150);

  analogWrite(MOTOR2_PIN1,0); 
  analogWrite(MOTOR2_PIN2,150);
}

void loop() {
  forward();
  delay(2000);
  backward();
  delay(2000);

}


```

[![Car spinning both directions]](https://img.youtube.com/vi/H4CORc9SBE8/0.jpg)(https://www.youtube.com/watch?v=H4CORc9SBE8)

### Installation
<!-- My full car with all hardware components attached can be seen below. -->
<!-- ![All components in the car, labeled](../../../public/fast-robots/lab4/full_car) -->


### PWM (Pretty Weak Movement?)
Explore the lower limit in PWM value for which the robot moves forward and on-axis turns while on the ground; note it may require slightly more power to start from rest compared to when it is running.
To find the minimum PWM value for which the car needs to move forward and on axis, I started with a small value and increasing it by 10 until the car moved. Then, I decreased the PWM by 5 to determine a more precise value. The video shows me using the PWM values to go forward and then spin on axis in both directions.


### Straight to the Point
[TODO] video going in straight line
Calibration demonstration (discussion, video, code, pictures as needed)

[TODO] video of Open loop code and video (open loop, untethered control of your robot - add in some turns.)

## 5000-Level Questions
(5000) analogWrite frequency discussion (include screenshots and code)


(5000) Lowest PWM value speed (once in motion) discussion (include videos where appropriate)

## Acknowledgements 

 I referenced the past lab reports from Katarina Duric and Aidan McNay from from Spring 2025. I give my thanks to Dyllan Hofflich for letting me borrow his soldering kit!