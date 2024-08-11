# Inverted_Pendulum_Cart_Simulation
Simulation of inverted pendulum on cart system with actuator dynamics with PD,LQR,MPC using MATLAB and Simulink. For detailed video explaination lof this work: [Youtube Video](https://youtu.be/VSngcnpMrpw?si=BNAlfhf5EGqBgAGB )


![lqr](https://github.com/user-attachments/assets/a8114429-0793-4a08-a74f-445929e4e548)


*Abstract* — Recently, a major interest has emerged for controlling underactuated mechanical systems, specifically inverted pendulum on a cart system, in many practical applications. However, studies used different and unrealistic models of the system, so the comparison between different controllers from different studies was difficult. Thus, the goal of this study was to design and analyze the performances of three controllers (i.e., LQR, PD, MPC) to achieve the control objective on a virtual inverted pendulum on cart system with actuator dynamics and constraints. The dynamics equations of motion were determined using Euler-Langrangian equations and linearized at the point of equilibrium to generate a linear time invariant state space model. Tracking performances (e.g., settling time, rise time), energy usage (e.g., cost of transportation), and computational time were calculated for each controller. The results indicated that MPC had the best tracking performance (e.g., shortest settling time of 11.9s and 0.1s for cart position and pendulum) and energy usage (e.g., 80% less cost of transportation for cart position) but longest computational time (e.g., 4.12 s), while LQR and PD controller displayed similar tracking performances and energy usage but shortest computational time. The LQR design process was the most intuitive due to its ability to directly tune the most relevant control parameters such as the weights on tracking performance (Q) and control effort (R).

## Table of Contents
1. [Introduction](#introduction)
2. [Using the Code](#using-the-code)
3. [Contact Information](#contact-information)
4. [License](#license)

## Introduction
This github repo was created primarily for students (or anyone interested) in control systems (particularly, self-balancing and underatuated systems) geared towards robotics. The system in this example consists of an inverted pendulum mounted to a motorized cart. The inverted pendulum system is an example commonly found in control system textbooks and research literature. Its popularity derives in part from the fact that it is unstable without control, that is, the pendulum will simply fall over if the cart isn't moved to balance it. Additionally, the dynamics of the system are nonlinear. The objective of the control system is to balance the inverted pendulum by applying a force to the cart that the pendulum is attached to. A real-world example that relates directly to this inverted pendulum system is the attitude control of a booster rocket at takeoff. This inverted pendulum model is a canonical example for quickly testing the performance of different controller designs (much like the MNIST dataset for classifying accuracy of different machine-learning models https://yann.lecun.com/exdb/mnist/).  

In this work, we introduced three different controllers: Linear-Quadratic Regulator (LQR), Proportional-Derivative (PD), and Model Predictive Control (MPC). The detailed equations and control design are explained in the paper and lower level functions. Also we added motor parameters (e.g., saturation voltage) to add more realism to the system. Given the inertial and geometric properties of the system, the main Matlab script ("MAIN.m") computes the states (e.g., cart position, cart speed, motor effort) of the cart and outputs 2D animation of the cart motion along with some helpful kinematic/kinetic plots. The code is structured so that you can add your own controller designs. Hope you have fun!


## Using the Code
Simply clone the git repot or download the zip file. Open the MATLAB software and set the directory to the "MATLAB_code" folder. The 

### Software Requirements
- MATLAB 2020
- SIMULINK 2020
- ode45 tool box


## Contact Information
For any questions regarding the study or the data processing code, please contact:
- Seung Yun (Leo) Song: ssong47@illinois.edu


## License
MIT License

Copyright (c) 2020 ssong47

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


