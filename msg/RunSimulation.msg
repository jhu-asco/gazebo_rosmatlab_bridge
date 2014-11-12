# Request for running simulation
#uint32 nofcontrols #Number of controls provided. We will add interpolation if number of controls not equals number of simulation steps(For now constant control interpolation)
uint32[] steps #At every step[count], the control shifts to the next value
# Later different types of interpolation schemes will be introduced and if needed separate this from data collection freq
#uint32 simulation_steps #No of simulation steps to run
JointEfforts jointefforts #Joint Efforts to apply for the simulation should provide the efforts for simulation_steps
BodyWrenches bodywrenches #Body Wrenches to apply
#Later will add interpolation for now constant interpolation
