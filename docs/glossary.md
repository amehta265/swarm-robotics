---
layout: default
permalink: /glossary/
title: Glossary
eyebrow: Reference
subtitle: Every term used across the project, in one place. Each term has one meaning, as required by ASD-STE100.
---

<div class="tabmark" data-tab="Dynamics"></div>

## Dynamics and control

**Actuator.** The component that produces motion at a joint. On this robot, the actuator
produces a torque.

**Centrifugal force and Coriolis force.** These forces are not real physical forces. An
observer inside a rotating frame of reference senses these effects. The rotating earth is
one example. The outward pull that you feel when you turn in a circle is a second example.

**Centripetal force.** A real force. It points inward, to the centre of rotation. It
causes an object to move on a curved or circular path.

**Closed-form equation.** A mathematical formula that you can evaluate with a finite
number of standard operations.

**Degrees of freedom.** The number of independent parameters. This is the number of
unique ways that a robot can move.

**Derivative gain ($K_d$).** The gain that multiplies the rate of change of the error. It
reduces overshoot and oscillation.

**Euler-Lagrange equation.** The equation that comes from the Lagrangian formulation.

**Generalized coordinates.** The parameters that define the configuration of a physical
system in a unique way.

**Gravity feedforward.** A method that computes the gravity torque $g(q)$ analytically
and adds it directly to the controller output. The PID controller then handles only the
dynamic error. This method prevents integral windup.

**Integral gain ($K_i$).** The gain that multiplies the accumulated error over time. It
removes the steady-state error.

**Integral windup.** A failure mode of the integral term. If an obstacle stops the joint
before the target, the integrator increases without limit. When the obstacle moves away,
the joint receives all the stored torque at once. This can break joints on real robots.

**Lagrangian formulation.** A method that describes system behaviour with work and
energy, using generalized coordinates. It removes all workless forces. The resulting
equations are compact and in closed form.

**Linear velocity from length and angle.** $v = r\omega$, where $r$ is the radius to the
centre of rotation and $\omega$ is the angular velocity.

**LQR.** Linear Quadratic Regulator. An optimal model-based controller. The project
implements LQR on a cart-pole system, which has less complexity than a quadruped.

**Manipulator equation.** The rigid-body equation of motion:
$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau + J^{\mathsf{T}}f_{\text{ext}}$.
The simulator solves this equation at each timestep.

**Moment of inertia ($I$).** A measure of the distribution of mass relative to the axis of
rotation.

**Newton-Euler formulation.** A method that describes dynamic systems with force and
momentum. It comes from the second law of motion of Newton. The equations include
constraint forces, which you must remove with additional arithmetic operations.

**Overdamped.** A tuning condition. The angle moves slowly toward the target and never
reaches it. The cause is a $K_p$ value that is too low for the gravity load.

**PID.** Proportional-Integral-Derivative. A feedback control mechanism. It calculates
the error between the desired setpoint and the current state, and then makes corrections.

**Proportional gain ($K_p$).** The gain that multiplies the current error. It produces
most of the corrective torque.

**Rotational kinetic energy.** $E_k = \tfrac{1}{2}I\omega^2$.

**Setpoint.** The desired value of the controlled variable.

**Steady-state error.** The error that remains after the system stops moving. Gravity
usually causes it, because gravity is a constant external disturbance.

**Torque ($\tau$).** The rotational force that the controller commands at a joint.

**Translational kinetic energy.** $E_k = \tfrac{1}{2}mv^2$.

**Underdamped.** A tuning condition. The angle passes the target, oscillates, and then
stops.

**Well tuned.** A tuning condition. The angle increases quickly. The overshoot is small or
absent. The angle stops at the target.

---

<div class="tabmark" data-tab="Geometry"></div>

## Geometry and coordinate frames

**Body frame.** Coordinates relative to the robot. The body frame moves and rotates with
the robot. Sensor positions are defined in the body frame.

**Euler angles.** A set of angles that define an orientation by dividing the rotation into
one rotation for each axis. The order of the rotations is important. Euler angles cause
gimbal lock.

**Gimbal lock.** The loss of one degree of freedom in a three-dimensional system. It
occurs when two rotational axes become parallel.

**Norm of a vector.** The square root of the sum of the squares of the entries. It is the
length of the vector.

**Pitch.** Rotation about the Y axis. It moves the nose up or down.

**Quaternion.** A four-dimensional extension of the complex numbers, written as
$w + xi + yj + zk$ or as $(x, y, z, w)$. The scalar part gives the magnitude of the
rotation. The vector part gives the axis. The identity quaternion is `[0, 0, 0, 1]`.
PyBullet uses quaternions for all orientations, because they do not have gimbal lock.

**Roll.** Rotation about the X axis.

**Rotation matrix.** A matrix that rotates a vector by an angle. In two dimensions:
`new_x = cos(θ)x − sin(θ)y` and `new_y = sin(θ)x + cos(θ)y`.

**Unit vector, also called direction vector.** A vector with a length of exactly 1. It has
no dimensional units. Its only function is to describe a direction. To create one, divide
a vector by its norm.

**World frame, also called global frame.** Fixed global coordinates. The world frame never
moves. All collision queries are done in the world frame, because the physics of PyBullet
operates in world coordinates.

**Yaw.** Rotation about the Z axis.

---

<div class="tabmark" data-tab="Perception"></div>

## Perception

**Bearing index.** The index $i$ of the sensor that detected the target. Each sensor
points at a known angle $2\pi i / n$. Therefore the index gives the approximate direction
to the target.

**Detection.** The answer to the question: is an object present?

**Field of view.** The angular region that a sensor can observe.

**Gaussian noise.** Random error with a normal distribution. Real sensors have noise. The
simulation adds noise to make the readings realistic.

**Hit fraction.** A number between 0.0 and 1.0. It states how far along a ray the
collision occurred, as a fraction of the total ray length. A value of 1.0 means that the
ray hit nothing. The value is a fraction and not a distance, because the physics engine
does not know your units.

**Infrared beacon.** A marker on the target object. The sensors can identify this marker
and separate it from ordinary obstacles.

**Localization.** The answer to the question: where is the object, and in which frame?

**Ray casting.** A method that sends an invisible ray from an origin in a given direction.
It returns the first object that the ray hits, and the distance to that object. Real
infrared and lidar sensors operate in the same physical way.

**`rayTest`.** The PyBullet function that casts one ray. It returns a list with one tuple:
`(objectUniqueId, linkIndex, hitFraction, hitPosition, hitNormal)`.

**`rayTestBatch`.** The PyBullet function that casts many rays at the same time. It
accepts arrays of start positions and end positions. It returns one result for each ray.

**Recognition.** The answer to the question: what is the object? Is it an obstacle, or is
it the target?

**URDF.** Unified Robot Description Format. An XML file that describes a robot completely:
the links, the joints, the geometry, and the inertia values.

---

<div class="tabmark" data-tab="Estimation"></div>

## Estimation and probability

**Bayesian fusion.** The combination of two probability distributions to produce one
improved distribution.

**Covariance intersection (CI).** A conservative fusion rule that stays correct for any
correlation between the estimates. Julier and Uhlmann introduced it in 1997. The fused
variance is always larger, and therefore more conservative, than the naive product of
Gaussians. It prevents double counting.

**Distributed Kalman filtering.** A Kalman filter across multiple agents. Each agent has
partial measurements and keeps its own belief. The agents share beliefs occasionally. In
its full form, it assumes linear Gaussian dynamics and requires the solution of a Riccati
equation at each fusion step.

**Double counting, also called the rumour propagation problem.** The error that occurs
when a system fuses two estimates as independent estimates, although the estimates already
share information.

**Gaussian distribution.** A probability distribution defined by a mean $\mu$ and a
variance $\sigma^2$. The product of two Gaussians is another Gaussian. No other family of
distributions has this property in closed form.

**Information form fusion.** A fusion method that adds the certainties, where certainty is
$1/\sigma^2$. The combined certainty is always higher than either input.

**Kalman filter.** An algorithm that combines a motion model and noisy measurements in the
optimal way. Rudolf Kalman invented it in 1960. It has two steps: predict and update.

**Kalman gain ($K$).** The central term of the Kalman filter. Its value is between 0 and 1.
It states how much to trust the measurement compared with the prediction.

**Measurement noise ($R$).** The uncertainty in the sensor reading. A large $R$ produces a
small Kalman gain.

**Predict step.** The first step of the Kalman filter. Time passes and the uncertainty
increases. `mu_pred = F × mu` and `sigma²_pred = F² × sigma² + Q`.

**Process noise ($Q$).** The amount by which the uncertainty grows in each timestep.

**Riccati equation.** The equation that the full distributed Kalman filter must solve at
each fusion step.

**Staleness penalty.** An increase in the variance of a received belief, in proportion to
its age. It costs one float per message and one multiplication per cell.

**Track filter.** The four-state filter above the pheromone grid. It maintains a
persistent belief about the position and the velocity of the target. It never resets.

**Update step.** The second step of the Kalman filter. A measurement arrives and the
uncertainty decreases.

**Variance ($\sigma^2$).** A measure of uncertainty. A low variance means high confidence.

---

<div class="tabmark" data-tab="Communication"></div>

## Communication and coordination

**Asynchrony.** The condition in which beliefs from different agents are never at the same
timestep. The standard Kalman filter assumes synchronous measurements.

**Byzantine fault tolerance.** The ability of a system to operate correctly although some
agents send incorrect data.

**Centralized system.** A system with one coordinator or one shared memory region. It
fails if that central component fails.

**Decentralized system.** A system with no central coordinator and no shared memory. The
agents exchange messages only.

**Delay-tolerant networking (DTN).** A networking method for connections that are
intermittent or that have long delays.

**Emergence.** Useful global behaviour that appears from simple local rules, without a
central plan.

**Loopback (127.0.0.1).** The network interface that a computer uses to send messages to
itself. Messages move through the networking stack of the operating system. They never
touch a physical radio.

**Message schema.** The defined structure of a message. This project uses JSON with the
fields `robot_id`, `timestamp`, `position`, `heading`, `detections`, and `help_request`.

**Reputation system.** A method that weights the estimates of a peer by the historical
accuracy of that peer.

**Shared memory.** A memory region that several processes can read and write directly. This
project does not use shared memory for the swarm architecture. It uses shared memory only
for the PyBullet physics simulation, which is a simulation artifact.

**Stigmergy.** Coordination through changes to a shared environment, rather than through
direct messages. Ant colonies use pheromone trails. This project uses a probability grid.

**UDP.** User Datagram Protocol. A communication standard that operates over any physical
medium: Wi-Fi, Ethernet, or cellular. It is a software protocol only.

---

<div class="tabmark" data-tab="Navigation"></div>

## Navigation and search

**Artificial potential fields (APF).** A navigation method that treats the robot as a
particle in a force field. An attractive force pulls the robot to the goal. Repulsive
forces push the robot from obstacles.

**Cascade architecture.** A control structure with layers. In locomotion the layers are
trajectory generation and then tracking. A drone uses position, then attitude, then motor.

**Deliberative method.** A method that plans a route before it moves.

**Distributed area partitioning.** The division of a search area between several robots.

**Frontier.** A list of unexplored regions. A frontier cell has a low $\mu$ value and a
high variance.

**Gait generator.** A function that returns joint target angles as a function of time. It
replaces the constant target angles of a standing pose.

**Lawnmower sweep.** A search pattern of parallel straight lines that covers an area
completely.

**Lissajous curve.** A figure-eight path. Phase 4b uses this path for the target.

**Local minimum.** A position where the net potential field force is zero, but the robot is
not at the goal. The robot then stops. This is the main limitation of potential fields.

**ORCA.** Optimal Reciprocal Collision Avoidance. A method based on velocity obstacles. It
corrects the local minimum problem of potential fields.

**Pheromone grid.** A two-dimensional map. Each cell holds a belief about the presence of
the target in that cell. The belief has a mean and a variance.

**Reactive method.** A method that responds directly to the current sensor readings,
without a plan.

**Sliding window grid.** A fixed-size grid centred on the current position of the robot.
The grid moves with the robot. It gives O(1) memory use for any arena size.

**Stance state.** The part of the gait cycle in which the foot is on the ground.

**Swing state.** The part of the gait cycle in which the foot is in the air.

**Trot gait.** A gait in which the diagonal leg pairs move together. The phase offset
between the two pairs is $\pi$.

**Velocity obstacles.** A collision avoidance method that selects velocities which avoid a
future collision.

**Waypoint.** A fixed position that the robot must reach on its route.

---

<div class="tabmark" data-tab="Simulation"></div>

## Simulation and tooling

**Hardware-in-the-loop.** A method in which physical hardware interacts with a simulation
in real time. Phase 6 uses this method.

**Kinematic base.** A robot base whose position is set directly, rather than computed from
forces. Phase 4 uses a kinematic base to avoid the balance problem.

**PyBullet.** The physics simulator that this project uses.

**`calculateInverseDynamics()`.** The PyBullet function that computes the $g(q)$ term for
gravity feedforward.

**`getBasePositionAndOrientation()`.** The PyBullet function that returns the position and
the quaternion orientation of the robot base.

**`multiplyTransforms()`.** The PyBullet function that rotates a body-frame offset by the
orientation of the robot, and then adds the world position of the robot. It uses
quaternions.

**`POSITION_CONTROL`.** A PyBullet control mode. Internally it applies
$\tau = K_p(\text{target} - \text{current}) + K_d(0 - \text{velocity})$. This is a PD
controller with no integral term.

**Server mode.** A PyBullet mode that creates one shared physics world. Several client
processes can then connect to that world.

**`useFixedBase=True`.** A PyBullet option that attaches the robot body to the world. Only
the joints then move. Phase 1B uses this option to remove the balance problem.

**WSL.** Windows Subsystem for Linux. The development environment for this project.
