# HERMES - HElicopter Real-time Model for Engineering Simulation. 
This project made me even more passionate about helicopter flight but most importantly it convinced me it is better if I keep studying them instead of trying to fly one. Watch me as I desperately try to hold an hover:\
![](/media/hover_piloting.gif)\
quite the performance ah?\
Jokes aside, in this personal project I built a non-linear, 9-DoF dynamic model for real-time simulation of conventional configuration helicopters in MATLAB/Simulink. It can be used to perform piloted simulations using a joystick or for control system design and flying qualities evaluation (mainly the latter if your piloting skills are similar to mine).

## Brief description of the model
I implemented the mathematical model from a NASA paper[^1] to which interested readers are referred to for further details. The helicopter motion is described by the 6-DoF Euler dynamics equations for the rigid body to which are applied the forces and moments developed by each of the helicopter components: the main and tail rotors, the empennages (horizontal and vertical tail planes) and the fuselage.
### Main rotor
The main rotor is the protagonist of the model and is described through a tip-path-plane rapresentation and a hinge-offset + flapping-spring model. This provides a description of the flapping dynamics which considers the first harmonic of the motion and up to the second order time derivative, thus it calculates the three flapping angles: coning, longitudinal and lateral flapping along with their rates and accelerations by solving the flapping equation in the non-rotating hub frame. This model is suitable to describe teetering, articulated and rigid rotor heads, with the limitation that the first harmonic description will neglect any higher frequency vibrations. The main rotor thus adds 3DoF for a total of 9DoF of the entire model. The inflow is considered uniform and is calculated through the Glauert momentum theory for forward flight (although a linear correction is applied in the flapping equation to predict lateral flapping due to triangular inflow in forward flight). Forces and moments are then calculated using analytical formulas obtained integrating along azimuth and radius, considering up to the second flapping derivative. 
### Tail rotor


## Usage and examples 
In this repository you find the simulink model of the helicopter [helicopter_sim.slx](/helicopter_sim.slx) along with the parameters file [parameters.m](/parameters.m), which contains all the necessary variables to run the model. The parameters are taken from the example helicopter featured in the famous book from Raymond Prouty[^2]. Inside the model there is a block called '3D_visualization' which if uncommmented will set the simulation pace to real-time and provide a visualization of the helicopter in motion. Additionally, the block 'GCS_gamepad' provides control inputs by connecting a joystick or joypad (you will probably have to adjust the control mapping inside the block). This is all you need to run a piloted simulation of the heli.\
Then you can find two additional scripts:\
- [trim.m](/trim.m)\
- [linearization.m](7linearization.m)\

### Response to controls

### Stability analysis

[^1]: Peter.D. Talbot et al., "A Mathematical Model for a Single Main Rotor Helicopter for Piloted Simulation", NASA T.M. 84281, 1982 [link](https://ntrs.nasa.gov/citations/19830001781)
[^2]: Raymond Prouty, "Helicopter Performance, Stability and Control", 1986
