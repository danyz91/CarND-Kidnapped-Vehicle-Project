## Appendix: Simulation Protocol

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

### Input

Values provided by the simulator to the c++ program

```cpp    
    // sense noisy position data from the simulator
    ["sense_x"]
    ["sense_y"]
    ["sense_theta"]
    // get the previous velocity and yaw rate to predict the particle's transitioned state
    ["previous_velocity"]
    ["previous_yawrate"]
    // receive noisy observation data from the simulator, in a respective list of x/y values
    ["sense_observations_x"]
    ["sense_observations_y"]
```

## Output

Values provided by the c++ program to the simulator

```cpp
    // best particle values used for calculating the error evaluation
    ["best_particle_x"]
    ["best_particle_y"]
    ["best_particle_theta"]
    //Optional message data used for debugging particle's sensing and associations
    // for respective (x,y) sensed positions ID label
    ["best_particle_associations"]
    // for respective (x,y) sensed positions
    ["best_particle_sense_x"] <= list of sensed x positions
    ["best_particle_sense_y"] <= list of sensed y positions
```

> * Map data provided by 3D Mapping Solutions GmbH.

