L5: Vehicle Control and Actuation
========================================

**Bridge:** From Planned Path to Vehicle Motion

Learning Objectives
-------------------

- Master vehicle dynamics and modeling  
- Design lateral and longitudinal controllers  
- Implement Model Predictive Control (MPC)  
- Understand real-world actuation challenges  

Outline
-------

Part 1: Vehicle Dynamics Fundamentals
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Introduces kinematic and dynamic vehicle models. Covers bicycle and single-track approximations, tire forces, and the relationship between control inputs and motion states.

Part 2: Lateral Control Design
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Explores techniques for steering and path tracking including Pure Pursuit, Stanley Controller, and Linear Quadratic Regulator (LQR). Emphasizes stability, cross-track error, and curvature constraints.

Part 3: Longitudinal Control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Focuses on maintaining desired speed and distance through PID and adaptive cruise control principles. Discusses following behavior and braking safety.

Part 4: Model Predictive Control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Presents MPC as a unified framework for simultaneous lateral and longitudinal control. Discusses prediction horizons, constraint handling, and optimization formulations.

Part 5: Implementation and Testing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Details the integration of control algorithms with real actuators. Discusses delay compensation, sensor feedback loops, actuator saturation, and validation on test platforms.
