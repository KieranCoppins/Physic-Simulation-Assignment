# Physics Simulation Assignment

This is my assignment for my physics simulation assignment at University. It uses NVIDIA PhysX 3.4.2 for the physics simulation and is connected with an OpenGL renderer for testing. It cab also connected with the PhysX Visual Debugger.

The assignment made us create a dominos simulation as close to real life values as possible. The dominos are 48mm x 24mm x 7.5mm and weigh 8.5g. The simulation has the following actors:

 - Newtons Cradle - Starts the domino reaction, configured with aluminium balls 20mm in radius and a density of 656.52 kg/m^3. These balls have restitution coefficient of 1 and therefore create a perfectly elastic collision.
 - Particle System - Used to visualise the ending of the domino show.
 - Stairs - a compound shape which spawns dominos on each step.

Other features include:
 - Shooting Newton's Cradle balls from the camera's position in the direction the camera is looking - Press R.
 - Particle system elements have a billboard effect that follow the cameras position. Saves resources on rendering.
 - Joints: A mix of distance and revolute joints are used in this demo.

## Grade: [Not released]
