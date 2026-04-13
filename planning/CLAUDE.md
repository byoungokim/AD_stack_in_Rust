# Planning Process (Process 2)

Full native Rust process for autonomous driving decision-making.

## Agent Working Here
- **Planning Agent**: behavior planning, global/local planning, E2E, arbitration

## Architecture
```
WorldState (CH1) ──→ Behavior Planner (5Hz)
                  ──→ Hybrid A* Global Planner (1Hz)
                  ──→ DWA + MPC Local Planner (10Hz)
                  ──→ E2E Inference (15Hz, optional)
                  ──→ Pipeline Arbitrator + Safety Envelope
                  ──→ ControlCommand (CH2)

ScenarioCommand (CH8) → goal injection → Behavior Planner
ScenarioStatus (CH9) ← status feedback
PlannedPath (CH10) ← visualization data
```

## Key Files
- `src/main.rs` — process entry, ZMQ wiring, main loop
- `src/behavior/` — driving state machine
- `src/global_planner/` — Hybrid A* with Ackermann motion primitives
- `src/local_planner/dwa.rs` — Dynamic Window Approach
- `src/local_planner/mpc.rs` — simplified Model Predictive Control
- `src/arbitrator/` — pipeline selection + safety envelope
- `src/e2e/` — end-to-end inference stub

## Build & Test
```bash
cargo check -p limo-planning
cargo test -p limo-planning    # 18 tests
```
