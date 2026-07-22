---
layout: default
permalink: /phase-0/
title: Phase 0 — Setup and Dynamics
eyebrow: Phase 0 · about 3 hours
subtitle: Understand the equation that the physics simulator solves at each timestep. Do not write a controller yet.
---

<div class="tabmark" data-tab="Overview"></div>

## Goal

Install PyBullet. Load a quadruped URDF model. Let the model stand under gravity.
Examine the model structure. Understand what you will control before you control it.

## What "state" means

The state of the robot has four parts:

1. The joint angles.
2. The joint velocities.
3. The pose of the base, which is the position and the orientation.
4. The twist of the base, which is the linear velocity and the angular velocity.

<div class="tabmark" data-tab="Manipulator equation"></div>

## The manipulator equation

The manipulator equation comes from the Newton-Euler formulation. It uses generalized
coordinates.

$$M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + g(q) = \tau + J^{\mathsf{T}} f_{\text{ext}}$$

Each term has a specific function:

| Term | Name | Function |
|---|---|---|
| $q$ | Joint angles | The state that you read from the simulator |
| $\ddot{q}$ | Joint accelerations | The value that the simulator solves for |
| $M(q)$ | Mass and inertia matrix | Opposes acceleration |
| $C(q,\dot{q})\,\dot{q}$ | Coriolis and centrifugal forces | Forces that occur in a rotating frame |
| $g(q)$ | Gravity torques | The gravity load on each joint |
| $\tau$ | Commanded torques | The output of your controller |
| $J^{\mathsf{T}} f_{\text{ext}}$ | External forces | Contact forces and collision forces |
| $f_{\text{ext}}$ | Contact force | The force from the ground on the foot |

The simulator solves this equation at each timestep. The equation gives the joint
accelerations. The simulator then integrates the accelerations to get the velocities and
the positions.

<div class="key" markdown="1">
<span class="callout-title">Why this matters later</span>
The $g(q)$ term returns in Phase 1. There, you compute $g(q)$ directly and add it to the
controller output. This method has the name gravity feedforward.
</div>

<div class="tabmark" data-tab="Equations of motion"></div>

## The equations of motion

**Definition.** The equations of motion give the relation between the input joint
torques and the output motion of the robot linkage.

Two methods give these equations. The first method is the Newton-Euler formulation. The
second method is the Lagrangian formulation.

### The Newton-Euler formulation

This method describes dynamic systems with force and momentum. It comes from the second
law of motion of Newton.

The equations include the constraint forces. Constraint forces are the coupling forces
and the coupling moments between the links. You must remove these forces with additional
arithmetic operations. Only then do you get the joint torques and the joint motion as a
function of joint displacement. The result is a closed-form equation.

### The Lagrangian formulation

This method describes the system behaviour with work and energy. It uses generalized
coordinates.

The method removes all workless forces, such as the coupling forces and the constraint
forces. The equations are compact. The equations are in closed form. The equations use
the joint torques and the joint displacements.

This method is also easier to derive than the Newton-Euler method.

The Lagrangian is:

$$L = T - U$$

$T$ is the kinetic energy. You write $T$ with generalized coordinates and generalized
velocities. $U$ is the potential energy. You write $U$ with generalized coordinates.

**Reference.** [MIT 2.12 Introduction to Robotics, Chapter 7 (PDF)](https://ocw.mit.edu/courses/2-12-introduction-to-robotics-fall-2005/c7caaa2376b8ec01e270328a3b80b029_chapter7.pdf)

<div class="tabmark" data-tab="Glossary"></div>

## Glossary for Phase 0

**Closed-form equation.** A mathematical formula. You can evaluate the formula with a
finite number of standard operations.

**Generalized coordinates.** The parameters that define the configuration of a physical
system. The definition must be unique.

**Euler-Lagrange equation.** The equation that comes from the Lagrangian formulation.

**Degrees of freedom.** The number of independent parameters. This is also the number of
unique ways that a robot can move.

**Translational kinetic energy.** The energy of motion in a straight line.

$$E_k = \tfrac{1}{2} m v^2$$

**Rotational kinetic energy.** The energy of motion around an axis.

$$E_k = \tfrac{1}{2} I \omega^2$$

$I$ is the moment of inertia. The moment of inertia measures the distribution of the
mass relative to the axis of rotation. $\omega$ is the angular velocity.

**Linear velocity from length and angle.**

$$v = r\omega$$

$r$ is the length, or the radius, to the centre of rotation. $\omega$ is the angular
velocity.

**Centrifugal force and Coriolis force.** These forces are not real physical forces. An
observer inside a rotating frame of reference senses these effects. The rotating earth
is one example of such a frame. The outward pull that you feel when you turn in a circle
is a second example.

**Centripetal force.** A real force. The force points inward, to the centre. The force
causes an object to move on a curved path or a circular path.

**Quaternion.** A four-dimensional extension of the complex numbers:

$$w + xi + yj + zk$$

The term $w$ is the scalar part. The scalar part gives the magnitude of the rotation.
Usually $w = \cos(\theta/2)$. The vector part gives the three-dimensional axis of the
rotation.

**URDF.** Unified Robot Description Format. This is an XML file. The file describes a
robot completely: the links, the joints, the geometry, and the inertia values.

## Next

Phase 1 adds the controller. Continue to
[Phase 1 — Control and Locomotion]({{ '/phase-1/' | relative_url }}).
