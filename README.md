# 1507Labs

**1507Labs** is the experimental robot codebase for [Team 1507 – Warlocks](https://warlocks1507.com): a sandbox for proving new systems before they move into the production [1507Base](https://github.com/Team1507/1507Base) skeleton.

## This branch: `SystemCore` (WPILib 2027)

This branch starts as an exact copy of **1507Base's `SystemCore` branch** (commit `c52be20`, 2026-10-03): WPILib 2027 alpha 7, Commands v3, the 2027 swerve, logging and auto route framework. Experiments are added on top of it.

- Open it with **WPILib 2027 VS Code** (alpha 7). The 2026 VS Code can't build it.
- The 2026 experiments (Policy Assist over NetworkTables, the Shooter and ShooterModel, PathPlanner, AdvantageKit) are on the `main` and `docs/add-readme` branches. They use 2026 APIs and need porting before they run here.
- See the 1507Base wiki for how the base works, and its **Future Plans** page for the ideas queued for Labs.

## Planned experiments

| Experiment | Notes |
|---|---|
| **Policy driver** | Run the trained path policy (Swerve-Policy-Playground) in Java on SystemCore, for autos written with `new AutoSequence(Driver.POLICY)`. Plan: [docs/2027-auto-plan.md](docs/2027-auto-plan.md), sections 5 and 7 |
| **Auto path tester** | Run every auto in the simulator and fail on a collision, leaving the field, and similar problems |
| **Better simulation** | CTRE motor simulation plus a simulated battery (current draw, voltage sag) |

When an experiment is proven, it moves into 1507Base, and fixes found in either repo are applied to both.

## Building

```bash
./gradlew build          # compiles and runs every test
./gradlew simulateJava   # robot simulator (use the Sim GUI; don't also run the Driver Station app)
```

## Background

[EXPERIMENTS_OVERVIEW.txt](EXPERIMENTS_OVERVIEW.txt) catalogues the 2026 experiments.
