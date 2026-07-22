---
layout: default
permalink: /
title: Search and Rescue Swarm
eyebrow: Project Notes
subtitle: Two simulated quadruped robots find a moving target. They share what they know over a network. Neither robot has global information.
---

<div class="tabmark" data-tab="Overview"></div>

## What this project is

This project builds a decentralized search and rescue system. Two quadruped robots
operate in a simulated world. The robots must find a target that moves. No central
computer controls the robots. Each robot has its own sensors, its own memory, and its
own decisions. The robots exchange messages over a network protocol.

The project has seven phases. Each phase adds one capability. Each phase also teaches
one concept that transfers to other robot platforms, such as drones or boats.

<div class="key" markdown="1">
<span class="callout-title">Robot names</span>
The first robot has the name **Walnut**. The second robot has the name **Hazel**.
</div>

## The phases

<div class="cards">
<a class="card" href="{{ '/phase-0/' | relative_url }}">
  <span class="card-phase">Phase 0 &middot; ~3 h</span>
  <span class="card-title">Setup and Dynamics</span>
  <span class="card-desc">Install PyBullet. Load a quadruped model. Learn the equation that the simulator solves at each timestep.</span>
  <span class="card-status status-done">Complete</span>
</a>
<a class="card" href="{{ '/phase-1/' | relative_url }}">
  <span class="card-phase">Phase 1 &middot; ~9 h</span>
  <span class="card-title">Control and Locomotion</span>
  <span class="card-desc">Write a PID controller. Hold a standing pose with 12 joints. Generate a trot gait.</span>
  <span class="card-status status-done">Complete</span>
</a>
<a class="card" href="{{ '/phase-2/' | relative_url }}">
  <span class="card-phase">Phase 2 &middot; ~6 h</span>
  <span class="card-title">Perception</span>
  <span class="card-desc">Simulate infrared sensors with ray casts. Add noise. Detect a target beacon.</span>
  <span class="card-status status-done">Complete</span>
</a>
<a class="card" href="{{ '/phase-3/' | relative_url }}">
  <span class="card-phase">Phase 3 &middot; ~9 h</span>
  <span class="card-title">Communication</span>
  <span class="card-desc">Send UDP messages between two processes. Fuse beliefs with a distributed Kalman filter.</span>
  <span class="card-status status-done">Complete</span>
</a>
<a class="card" href="{{ '/phase-4/' | relative_url }}">
  <span class="card-phase">Phase 4 &middot; ~6 h</span>
  <span class="card-title">Navigation and Search</span>
  <span class="card-desc">Avoid obstacles with potential fields. Divide the search area between the robots.</span>
  <span class="card-status status-done">Complete</span>
</a>
<a class="card" href="{{ '/phase-5/' | relative_url }}">
  <span class="card-phase">Phase 5 &amp; 6</span>
  <span class="card-title">Scale and Hardware</span>
  <span class="card-desc">Run three to four agents. Measure the results. Connect one physical sensor to the simulation.</span>
  <span class="card-status status-plan">In progress</span>
</a>
</div>

<div class="tabmark" data-tab="Simulation video"></div>

## The system running

These two recordings show the Phase 4b forest run. The arena is 11 m × 11 m and contains
18 tree obstacles. Walnut and Hazel search independently. The red sphere is the target,
which follows a Lissajous figure-eight path. The thin green and red lines are the
infrared rays from the sensor ring of each robot.

Neither robot knows where the target is at the start. Neither robot can read the memory
of the other robot. All coordination happens through the UDP messages described in
[Phase 3]({{ '/phase-3/' | relative_url }}).

<div class="videos">

<div class="clip">
  <video controls preload="none" playsinline poster="{{ '/assets/video/poster-run-1.jpg' | relative_url }}">
    <source src="{{ '/assets/video/forest-run-1.mp4' | relative_url }}" type="video/mp4">
    Your browser cannot play this video.
    <a href="{{ '/assets/video/forest-run-1.mp4' | relative_url }}">Download the file instead.</a>
  </video>
  <div class="clip-meta">
    <span class="tag">Phase 4b</span>
    <span class="tag-alt tag">1 min 30 s</span>
    <span class="tag-alt tag">No sound</span>
  </div>
  <p class="clip-cap"><b>Forest run 1 — both robots search.</b> The two quadrupeds start
  near the centre and move outward through the trees. Watch the infrared rays change
  colour as they meet a trunk. The artificial potential field keeps each robot clear of
  the trunks and clear of the other robot.</p>
</div>

<div class="clip">
  <video controls preload="none" playsinline poster="{{ '/assets/video/poster-run-2.jpg' | relative_url }}">
    <source src="{{ '/assets/video/forest-run-2.mp4' | relative_url }}" type="video/mp4">
    Your browser cannot play this video.
    <a href="{{ '/assets/video/forest-run-2.mp4' | relative_url }}">Download the file instead.</a>
  </video>
  <div class="clip-meta">
    <span class="tag">Phase 4b</span>
    <span class="tag-alt tag">1 min 39 s</span>
    <span class="tag-alt tag">No sound</span>
  </div>
  <p class="clip-cap"><b>Forest run 2 — approach to the target.</b> One robot moves close
  to the red sphere. The camera then pulls back to show the full arena. The target
  continues to move along its figure-eight path, so the robots must keep tracking it
  rather than stop at one position.</p>
</div>

</div>

## The simulated world

<div class="gallery">

<a class="shot" href="{{ '/phase-1/#1b-standing' | relative_url }}">
  <img src="{{ '/assets/images/sim-single-robot.jpg' | relative_url }}" alt="A PyBullet view of one quadruped robot standing on a blue and white checked floor. A red sphere sits on the back of the robot. Thin green and red lines extend outward from the body.">
  <span class="shot-cap"><b>One robot, empty arena</b>The early configuration. The green and red lines are the eight infrared rays from the sensor ring.</span>
</a>

<a class="shot" href="{{ '/phase-4/#forest-results' | relative_url }}">
  <img src="{{ '/assets/images/sim-forest-pair.jpg' | relative_url }}" alt="A PyBullet view of a forest of tall trees on a checked floor. Two small quadruped robots stand near the centre. A red sphere floats between the trees.">
  <span class="shot-cap"><b>Walnut and Hazel in the forest</b>Both robots operate in the same physics world, as separate processes connected to a PyBullet server.</span>
</a>

<a class="shot" href="{{ '/phase-4/#forest-results' | relative_url }}">
  <img src="{{ '/assets/images/sim-forest-close.jpg' | relative_url }}" alt="A close view of two quadruped robots between tree trunks. A red sphere sits to the right. Red and green sensor lines cross the floor.">
  <span class="shot-cap"><b>Close view of the sensor rays</b>Green means the ray met nothing within range. Red means the ray met a trunk or a wall.</span>
</a>

<a class="shot" href="{{ '/phase-4/#forest-results' | relative_url }}">
  <img src="{{ '/assets/images/sim-forest-wide.jpg' | relative_url }}" alt="A wide view of the whole walled arena from above and to the side. Eighteen trees are spread across the floor. Two robots and a red sphere are visible among them.">
  <span class="shot-cap"><b>The full arena</b>11 m × 11 m, four walls, and 18 trees at 1.8 m spacing. The spacing lets a robot pass between any two trunks.</span>
</a>

</div>

<div class="tabmark" data-tab="Results"></div>

## Results at a glance

Each phase produced measured results. Select a plot to open the phase that explains it.

<div class="gallery">

<a class="shot" href="{{ '/phase-1/#1a-pid-control' | relative_url }}">
  <img src="{{ '/assets/images/pid_result.png' | relative_url }}" alt="Two plots. The upper plot shows a joint angle rising to about 37 degrees and holding near the target. The lower plot shows the commanded torque, with a large spike at the start and then a value near zero.">
  <span class="shot-cap"><b>Phase 1A — tuned PID joint</b>Kp = 8.0, Ki = 0.5, Kd = 3.0. The torque settles near zero, because the gravity feedforward term carries the static load.</span>
</a>

<a class="shot" href="{{ '/phase-1/#1b-standing' | relative_url }}">
  <img src="{{ '/assets/images/standing_result.png' | relative_url }}" alt="Two plots. The upper plot shows a joint angle oscillating around a dashed target line at 38.4 degrees. The lower plot shows the base height as a flat line at 0.35 metres.">
  <span class="shot-cap"><b>Phase 1B — standing pose, 12 joints</b>The flat base height line confirms that the body is fixed. The joint oscillation comes from ground contact.</span>
</a>

<a class="shot" href="{{ '/phase-1/#1c-trot-gait' | relative_url }}">
  <img src="{{ '/assets/images/gait_result.png' | relative_url }}" alt="Two plots. The upper plot shows two hip joints oscillating in opposite phase. The lower plot shows one hip tracking a sinusoidal target.">
  <span class="shot-cap"><b>Phase 1C — trot gait at 1 Hz</b>The front-right hip and the front-left hip are in anti-phase. This is the trot pattern.</span>
</a>

<a class="shot" href="{{ '/phase-2/#results' | relative_url }}">
  <img src="{{ '/assets/images/sensor_result.png' | relative_url }}" alt="Two plots. The upper plot shows the distance from one infrared sensor. The lower plot shows six magenta pulses that mark target detection events.">
  <span class="shot-cap"><b>Phase 2 — infrared sensor array</b>Six detection pulses, one for each pass of the target across a sensor ray.</span>
</a>

<a class="shot" href="{{ '/phase-3/#results' | relative_url }}">
  <img src="{{ '/assets/images/uncertainty_comparison.png' | relative_url }}" alt="Two plots. The upper plot compares dashed grid uncertainty traces that reset to 1.0 against solid track filter traces that stay low. The lower plot shows estimated target speed around 1.0 metres per second.">
  <span class="shot-cap"><b>Phase 3 — the two filter layers</b>The dashed grid traces reset to 1.0. The solid track traces never reset. That separation is what Layer 2 adds.</span>
</a>

<a class="shot" href="{{ '/phase-3/#results' | relative_url }}">
  <img src="{{ '/assets/images/trajectories.png' | relative_url }}" alt="Two scatter plots. Each shows a dashed circular target path with green estimate dots on it, and a triangle that marks the robot position.">
  <span class="shot-cap"><b>Phase 3 — track estimate against truth</b>Dark green means low uncertainty. Both robots track the target accurately from opposite sides of the arena.</span>
</a>

<a class="shot" href="{{ '/phase-3/#results' | relative_url }}">
  <img src="{{ '/assets/images/grid_snapshots.png' | relative_url }}" alt="Four panels. The left panels show pheromone grids as mostly black fields. The right panels show uncertainty grids with small dark blue squares at matching positions.">
  <span class="shot-cap"><b>Phase 3 — covariance intersection, visible</b>The confident cells sit at the same world positions in both grids, although the robots share no memory.</span>
</a>

<a class="shot" href="{{ '/phase-4/#forest-results' | relative_url }}">
  <img src="{{ '/assets/images/forest_search_paths_flank.png' | relative_url }}" alt="Two plots of an 11 by 11 metre arena. Each shows green dots that mark 18 tree obstacles, a figure-eight target path, and a robot path that moves around the obstacles.">
  <span class="shot-cap"><b>Phase 4 — search in a forest</b>18 obstacles, an 11 m × 11 m arena, and a figure-eight target path. Walnut found the target at t = 10 s, Hazel at t = 12 s.</span>
</a>

</div>

<div class="tabmark" data-tab="The full plan"></div>

## The full plan

### Phase 0 — Setup and the dynamics (about 3 hours)

Install PyBullet. Load a quadruped URDF model. Let the model stand under gravity.
Learn the rigid-body equation:

$$M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + g(q) = \tau$$

Learn what each term means. Learn why the simulator solves this equation at each
timestep. Examine the URDF file. Identify the links and the joints. Learn what "state"
means. The state is the joint angles, the joint velocities, the base pose, and the base
twist. Do not write a controller in this phase. First understand what you will control.

### Phase 1 — Single-robot control and locomotion (about 9 hours)

Write a joint PID position controller. Tune one joint. Observe the overshoot. Observe
the steady-state error. Learn the effect of each gain.

Then build a gait generator. The gait generator uses foot trajectories with parameters.
Each leg has a different phase offset. A trot gait moves the diagonal legs together.

Locomotion has two parts. The first part is trajectory generation, which decides where
the feet must go. The second part is tracking, which is the PID controller that moves
the joints to the necessary positions. This structure is a cascade. A drone uses the
same structure: position, then attitude, then motor.

Also write an LQR controller for a cart-pole system. The cart-pole has less complexity
than the quadruped. This makes optimal model-based control easier to learn.

The cascade structure and the difference between PID and LQR apply to all robot
platforms.

### Phase 2 — Perception (about 6 hours)

Simulate infrared sensors with ray casts from the robot body. Use the `rayTest`
function. Real infrared sensors and lidar sensors work in the same way: they send a
ray and they measure the distance to the first object.

Add a field of view. Add Gaussian noise, because real sensors are not exact.

Give the target an infrared beacon. The sensor must identify this beacon. The sensor
returns the bearing and the range when the beacon is in view.

Learn the difference between three operations:

- **Detection** — is an object present?
- **Localization** — where is the object, and in which frame?
- **Recognition** — what is the object?

Also learn the coordinate transforms from the sensor frame to the body frame, and from
the body frame to the world frame.

The same sensor model becomes obstacle detection on a drone, or sonar on a boat.

### Phase 3 — The decentralized communication protocol (about 9 hours)

This phase is the most important part of the project.

Divide the system into two independent agent processes. Design a message schema. Start
with JSON:

```json
{ "robot_id": "walnut", "timestamp": 12.4, "position": [1.2, 0.8],
  "heading": 0.35, "detections": [], "help_request": null }
```

Send the messages over UDP. Then add shared-belief fusion. Each agent adds the
detections of the other agents to its own world model. Handle messages that are late or
lost. Use timestamps and timeouts.

Learn the difference between centralized systems and decentralized systems. Learn
protocol design. Learn why local-only knowledge is the core problem.

This layer transfers without change to drones and boats. It is the most expandable part
of the project.

### Phase 4 — Collision avoidance and search strategy (about 6 hours)

Write artificial potential fields. An attractive force pulls the robot to the target. A
repulsive force pushes the robot away from obstacles and away from the other robot.

The robot will stop in a local minimum. This failure is part of the lesson. It shows why
engineers use Velocity Obstacles or ORCA instead.

For the search problem, divide the arena between the two robots. This is distributed
area partitioning. Then run a lawnmower sweep pattern.

Learn the difference between reactive methods and deliberative methods. Learn the limits
of purely local methods.

### Phase 5 — Integrate, scale, and measure (about 5 hours)

Run the full mission. Both robots search. One robot finds the target. That robot calls
the other robot. Both robots move to the target. The robots do not collide.

Then start three or four agent processes. If the Phase 3 design is correct, this works
without new code. This result proves that decentralization scales.

The communication volume increases as O(n²). Add range-limited messaging to correct
this.

Measure the time to find the target, the number of collisions, and the messages per
second.

Learn about emergence and about the trade-offs of scale.

### Phase 6 — The hardware test (optional, about 3 hours)

Use one ESP32 microcontroller (approximately $8), one infrared sensor (approximately
$2), and a breadboard. Connect the components. Read the sensor in code. Then make the
ESP32 send a real UDP message into the simulated swarm.

This is a simulation-to-reality bridge. A physical sensor changes the behaviour of
virtual agents. This method has the name hardware-in-the-loop.

<div class="tabmark" data-tab="Repository"></div>

## Repository layout

```text
swarm_robotics/
├── phase1/
│   ├── pid_control.py          # Phase 1A — single joint PID
│   ├── standing.py             # Phase 1B — 12-joint standing pose
│   ├── gait_generator.py       # Phase 1C — trot gait
│   └── plot_gait.py
├── phase2/
│   ├── ir_sensor.py            # IRSensor and IRSensorArray classes
│   └── plot_sensor.py
├── phase3/
│   ├── pheromone_grid.py       # per-cell Kalman filter grid
│   └── plot_phase3.py
├── phase4/
│   ├── navigators.py           # Frontier, APF, Waypoint
│   └── phase4_forest.py
└── docs/                       # this site (GitHub Pages)
```

## How to read these pages

Each phase page is divided into tabs. The tab bar stays at the top of the page while you
scroll. Select a tab to replace the content below it. Use the previous and next controls
at the foot of each tab to move through a phase in order.

The theme control is at the foot of the sidebar. It has three states: system, light, and
dark. The system state follows your operating system setting.

## A note on the writing

These pages use ASD-STE100 Simplified Technical English. This standard uses short
sentences, the active voice, and one meaning for each word. The standard removes idioms
and metaphors. The purpose is clear technical communication.

Some sentences keep a comparison, such as the comparison to ant colonies. These
comparisons are necessary to explain the concept. They are written as plain statements.
