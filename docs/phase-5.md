---
layout: default
permalink: /phase-5/
title: Phase 5 and Phase 6 — Planned Work
eyebrow: Phase 5 · about 5 hours · Phase 6 · about 3 hours
subtitle: Integrate the full mission, scale to four agents, measure the results, and connect one physical sensor to the simulation.
---
<div class="warn-box" markdown="1">
<span class="callout-title">Status</span>
Phase 5 has started. The phase is divided into three sections. The detailed notes for
those sections are not yet written. This page holds the plan.
</div>

<div class="tabmark" data-tab="Phase 5"></div>

## Phase 5 — Integrate, scale, and measure

### The full mission run

Run the complete mission. The mission has five stages:

1. Both robots search the arena.
2. One robot finds the target.
3. That robot calls the other robot.
4. Both robots move to the target.
5. No collision occurs.

### Scaling test

Start three or four agent processes.

If the [Phase 3]({{ '/phase-3/' | relative_url }}) design is clean, this works without
new code. That result proves that decentralization scales.

### The communication volume problem

The communication volume increases as O(n²). Every agent sends a message to every other
agent.

**Correction.** Add range-limited messaging. An agent then sends messages only to agents
that are within a given distance.

### Measurements

Measure three values:

| Metric | Meaning |
|---|---|
| Time to find | How long the swarm needs to detect the target |
| Collisions | The number of contacts between robots, or between a robot and an obstacle |
| Messages per second | The communication load on the network |

### Concepts

This phase makes two concepts concrete: emergence, and the trade-offs of scale.

---

<div class="tabmark" data-tab="Phase 6"></div>

## Phase 6 — The hardware test (optional)

### Components

| Component | Approximate cost |
|---|---|
| One ESP32 microcontroller | $8 |
| One infrared sensor | $2 |
| One breadboard | — |

### Procedure

1. Connect the components on the breadboard.
2. Read the sensor value in code on the ESP32.
3. Make the ESP32 send a real UDP message into the simulated swarm.

### Why this matters

This is a simulation-to-reality bridge. A physical sensor changes the behaviour of the
virtual agents.

This gives the experience of building the circuit from components. It does not require a
large budget or a long schedule.

The method has a formal name: hardware-in-the-loop. It is a real engineering concept, not
a demonstration.

---

<div class="tabmark" data-tab="Open items"></div>

## Open items from earlier phases

These items stay unresolved. Record them here so that they are not lost.

### The balance problem

The robot cannot balance on a free-floating base. The system uses `useFixedBase=True` in
[Phase 1B]({{ '/phase-1/' | relative_url }}). It uses a kinematic base in
[Phase 4]({{ '/phase-4/' | relative_url }}).

Free-floating balance is a large problem. Boston Dynamics used several years to solve it.

### The waypoint and pheromone grid conflict

The robots currently navigate between fixed waypoints. But the system also maintains a
pheromone grid.

If the robots use fixed waypoints, the pheromone grid has no navigation function. In a
truly unexplored environment, no waypoints exist.

The robots must select their path from the pheromone grid. The `FrontierNavigator` class
was the attempt at this. It failed for the reasons in
[Phase 4]({{ '/phase-4/' | relative_url }}). The frontier method needs a correction that
limits the frontier candidates to reachable cells inside the arena.

### Idea 3 — Byzantine-resilient coordination

The project selected Idea 1 and Idea 2 from
[Phase 3]({{ '/phase-3/' | relative_url }}). Idea 3 stays available as future work.

If the sensors of one robot fail, that robot broadcasts incorrect target positions. The
other robot currently accepts those positions without any check.

A possible solution is a reputation system. The system weights peer estimates by their
historical accuracy. You can then measure how quickly the swarm recovers from a faulty
agent.

### ORCA

[Phase 4]({{ '/phase-4/' | relative_url }}) uses artificial potential fields. Potential
fields have local minima.

Velocity Obstacles and ORCA are the standard corrections for that limitation. The
original plan included ORCA if time was available.
