# WALL-E and EVE's Signal

An autonomous search-agent simulation in which WALL-E explores a configurable grid world, discovers clues through interaction, maintains a probabilistic belief about EVE's location, manages battery/time resources, logs actions and observations, and generates a mission report and browser visualization.

## Features
- Configurable JSON map, random seed, obstacles, charging stations, sensor noise, hidden EVE position, clue placement, battery and time budgets.
- Environment returns observations for actions; the agent does not use hidden coordinates for planning.
- Autonomous movement, scanning, recharge decisions, return-to-base policy, and shortest-safe-path routing (BFS).
- Belief updates from signal-strength readings, clue bearings, blocked movement, and direct detections.
- Full trace: action, position, observation, decision rationale, battery and elapsed time.
- Mission report with status, evidence, inferred location, confidence, resources, and next-step recommendation.
- Self-contained HTML visualization of the map and visited path.
- Standard-library Python only; tests included.

## Run
Requires Python 3.10+.

```bash
python mission.py
```

Custom run:

```bash
python mission.py --config configs/sample_mission.json --out outputs --seed 22
```

Open `outputs/visualization.html`; inspect `outputs/mission_report.md`, `outputs/report.json`, and `outputs/trace.json`.

## Configuration
Coordinates use `[x,y]`, origin at top-left. Edit `configs/sample_mission.json` to change map, obstacles, chargers, resource budget, sensor radius/noise, hidden EVE location, and clues. `hidden_world` is ground truth owned by the environment; it is not exposed to the agent's policy.

## Agent strategy
1. Start with a uniform belief distribution over traversable cells.
2. Scan when affordable and update beliefs using a distance-decay signal model.
3. Update beliefs from clue bearings and blocked paths.
4. Use BFS for safe routes and explore promising unvisited cells.
5. Recharge or return when the battery reserve or time budget is low.
6. On detection, return to base and produce the mission report.

## Evaluation and limitations
This is a reproducible educational grid-world prototype, not a physical robotics controller. The sensor model is simplified and the reported maximum cell probability is not a calibrated probability of correctness. Resource limits and obstacle layouts may cause a mission to return without confirmation. The HTML visualization intentionally shows ground-truth markers for evaluation; those markers are not supplied to the agent.

Useful evaluation metrics include detection rate, steps to detection, clues discovered, visited-cell coverage, battery remaining, and inferred-location accuracy. For stronger evaluation, run multiple seeds and aggregate the generated `report.json` files.

## Bonus extensions
Try decoy/spoofed signals, sensor dropouts, multi-agent coordination, strategy comparisons (greedy vs frontier exploration), multi-seed benchmark plots, or a Gazebo/Webots adapter.

## Deliverables
- `mission.py`: environment, agent, planning, belief updates, trace and report generation.
- `configs/sample_mission.json`: complete example mission configuration.
- `examples/`: sample output artifacts generated from the included config.
- `tests/test_mission.py`: basic automated tests.
- `outputs/`: created by each run.
