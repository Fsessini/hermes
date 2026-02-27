# HERMES - HElicopter Real-time Model for Engineering Simulation. 
This project made me even more passionate and curious about helicopters but most importantly it convinced me it is better if I keep studying them instead of trying to fly one. Watch me as I desperately try to hold a hover:
<p align="center">
<img src="/media/hover_piloting.gif" alt="Alt text" width="400"/><img src="/media/joystick.gif" alt="Alt text" width="200"/>
</p>
Quite the performance huh?
Jokes aside, in this personal project I built a non-linear, 9-DoF dynamic model for real-time simulation of conventional configuration helicopters in MATLAB/Simulink. It can be used to perform piloted simulations using a joystick or for control system design and flying qualities evaluation (mainly the latter if your piloting skills are similar to mine).

## Brief description of the model
I implemented the mathematical model from a NASA paper[^1] to which interested readers should refer for further details. The helicopter motion is described by the 6-DoF Euler equations of rigid body dynamics, forces and moments applied on the aircraft are derived for each of the helicopter components: the main and tail rotors, the empennages (horizontal and vertical tail planes) and the fuselage.
### Main rotor
The main rotor is the protagonist of the model and is described through a tip-path-plane rapresentation and a hinge-offset + flapping-spring model. This provides a description of the flapping dynamics which considers the first harmonic of the motion and up to the second order time derivative, thus it calculates the three flapping angles: coning, longitudinal and lateral flapping along with their rates and accelerations by solving the flapping equation in the non-rotating hub frame. This model is suitable to describe teetering, articulated and rigid rotor heads, with the limitation that the first harmonic description will neglect any higher frequency vibrations. The main rotor thus adds 3DoF for a total of 9DoF of the entire model. The inflow is considered uniform and is calculated through the Glauert momentum theory for forward flight (although a linear correction is applied in the flapping equation to predict lateral flapping due to triangular inflow in forward flight). Forces and moments are then calculated using analytical formulas obtained integrating along azimuth and radius, considering up to the second flapping derivative. 
### Tail rotor
The tail rotor is supposed to behave as a teetering rotor thus with zero hinge offset, and the flapping dynamics are neglected due to its high angular speed and consequently very small time constant. The flapping angles are obtained solving for the steady state values of the tip path plane equation. No cyclic pitch is of course present but the effect of pitch-flap coupling ($\delta_3$ hinge angle) is modeled. Coning is fixed and downwash from main rotor is not considered. Apart from these simplifications, the equations for forces and moments are the same as for the main rotor, properly rotated to consider the orientation of the tail rotor.
### Empennages and fuselage 
Horizontal and vertical stabilizers are modeled as three-dimensional wings with the lifting line theory, considering the post-stall behavior in order to consider all possible attack angles. The effect of main rotor downwash on the horizontal stabilizers is considered on direction and magnitude of the apparent wind, and the stabilizer is supposed to always be inside the completely developed rotor wake (valid for hover and low speed forward flight).\
The vertical stabilizer has a cambered profile to lighten the tail rotor load in string-centered forward flight, and the effect of tail rotor downwash is considered in the sideforce drag, while no effect of main rotor downwash is considered.\
The fuselage is modeled through constant aerodynamic coefficients of lift, drag and moment with respect to attack and sideslip angles, but the model is valid only in the range between -15 and 15 degrees. The effect of rotor downwash is considered only on the fuselage attack angle.

## Usage and examples 
In this repository you find the simulink model of the helicopter [helicopter_sim.slx](/helicopter_sim.slx) along with the parameters file [parameters.m](/parameters.m), which contains all the necessary variables to run the model. The parameters are taken from the example helicopter featured in the famous book from Raymond Prouty[^2]. Inside the model there is a block called '3D_visualization' which if uncommmented will set the simulation pace to real-time and provide a visualization of the helicopter in motion. Additionally, the block 'GCS_gamepad' provides control inputs by connecting a joystick or joypad (you will probably have to adjust the control mapping inside the block). This is all you need to run a piloted simulation of the heli.\
Then you can find two additional scripts:
- [trim.m](/trim.m): calculates the control inputs and the helicopter attitude necessary to keep the heli in a steady flight condition, specified in the parameters file by variable 'fc'(flight condition). It is limited to hover, level forward flight and steady climb/descent. It works by imposing the equilibrium of forces and moments on the steady state model, disabling the dynamics. The solution is found through a Newton-Raphson algorithm with numerical approximation of the Jacobian.
- [linearization.m](7linearization.m): calculates the stability and control derivatives of the heli with centered finite differences. The linear model is the classic 6 DoF linearized rigid body dynamics used in aircraft flight dynamics, see the classic textbook from Padfield[^3] for a complete derivation. Since flapping angles are not considered as state variables, the main rotor dynamics are effectively neglected and its behavior is considered to be quasi-static. What this means practicaly is that derivatives are calculated disabling rigid body dynamics but letting the flapping reach steady state. This linearized model requires a steady symmetryc flight condition with null angular rates. The trim script needs to be run first.

### Response to controls
By using the script [step.m](/step.m) you can test the response of the helicopter model to a step input of the controls. The script also compares it to the prediction of the linearized model.\
Here you see an example in a forward flight condition at 100ft altitude.
<p align="center">
<img src="/media/lon_cyc_step_ff_60kn.png" alt="Alt text" width="400"/>
<img src="/media/lat_cyc_step_ff_60kn.png" alt="Alt text" width="400"/>
<img src="/media/coll_step_ff_60kn.png" alt="Alt text" width="400"/>
<img src="/media/ped_step_ff_60kn.png" alt="Alt text" width="400"/>
</p>

### Stability analysis
One of the main porpouses of an aircraft model is to evaluate static and dynamic stability across the flight envelope. The script [stability.m](/stability.m) trims and linearizes the heli for a range of forward flight speeds, then saves and plots the control inputs and the eigenvalues.\
Here we see the control inputs in level forward flight plotted against the flight speed:
<p align="center">
<img src="/media/cyc_input_trim_ff.png" alt="Alt text" width="400"/>
<img src="/media/coll_input_trim_ff.png" alt="Alt text" width="400"/>
</p>
Here instead we have on the left the eigenvalues for the hover condition and on the right the movement of the system poles in forward flight as the speed grows from 60 to 160 knots:
<p align="center">
<img src="/media/eig_hov.png" alt="Alt text" width="500"/>
<img src="/media/eig_ff.png" alt="Alt text" width="500"/>
</p>
The phugoid and dutch roll modes in hover are named like this for convenience, but they actually are, respectively, a longitudinal/lateral and lateral/longitudinal coupled oscillation, and they later develope in the two familiar flight modes as the forward speed is increased. The same is true for the spiral, which in hover is a simple yaw subsidence mode.  
We can also separate the longitudinal and lateral directional dynamics, as is customary to do for fixed wing aircraft, and confront the predicted poles:
<p align="center">
<img src="/media/lon_eig_ff.png" alt="Alt text" width="500"/>
<img src="/media/lat_eig_ff.png" alt="Alt text" width="500"/>
</p>
The oscillatory dynamics (phugoid and dutch roll) are pretty well predicted by the uncoupled simplified models, as well as the slow heave and spiral modes, while the fast pitch and roll subsidence modes show quite a bit of error.

[^1]: Peter D. Talbot et al., "A Mathematical Model for a Single Main Rotor Helicopter for Piloted Simulation", NASA T.M. 84281, 1982 [link](https://ntrs.nasa.gov/citations/19830001781)
[^2]: Raymond Prouty, "Helicopter Performance, Stability and Control", 1986
[^3]: Gareth D. Padfield, "Helicopter Flight Dynamics", 2007
