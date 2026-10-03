# 1507Labs

**1507Labs** is the experimental robot codebase for [Team 1507 – Warlocks](https://warlocks1507.com): a sandbox for proving new systems before they move into the production [1507Base](https://github.com/Team1507/1507Base) skeleton.

## How this repo works

- **`main` is an exact copy of [1507Base](https://github.com/Team1507/1507Base)'s `SystemCore` branch** (WPILib 2027 alpha 7, Commands v3, the 2027 swerve, logging and auto route framework). Only Labs' own files differ: this README, `EXPERIMENTS_OVERVIEW.txt` and `.claude/settings.local.json`. Nobody experiments on `main`.
- **Each experiment gets its own branch** made from `main`, named `exp/<name>` (for example `exp/policy-driver`). CI builds and tests every branch on every push.
- Open the repo with **WPILib 2027 VS Code** (alpha 7). The 2026 VS Code can't build it.

### Starting an experiment

```bash
git switch main
git pull
git switch -c exp/<name>
git push -u origin exp/<name>
```

### When 1507Base changes

1. Copy 1507Base's files into Labs `main` (everything except `.git` and the Labs-only files listed above).
2. Check that the Labs-only files are the only differences from 1507Base, build, commit "Sync with 1507Base <commit>", and push.
3. Merge `main` into each active experiment branch (`git switch exp/<name>`, then `git merge main`).

### When an experiment is proven

Because `main` matches 1507Base, the experiment's changes are just `git diff main..exp/<name>`. Apply that diff to 1507Base (as a patch, or by hand), then sync Labs `main` as above. Fixes found in either repo are applied to both.

### The 2026 experiments

The 2026 (RoboRIO) experiments are archived at tag **`labs-2026-final`** (branch `archive/2026-experiments`): Policy Assist over NetworkTables, the Shooter and ShooterModel, PathPlanner, AdvantageKit. They use 2026 APIs, so read them for ideas and redo them on a new `exp/` branch. [EXPERIMENTS_OVERVIEW.txt](EXPERIMENTS_OVERVIEW.txt) catalogues them.

## Planned experiments

| Experiment | Notes |
|---|---|
| **Policy driver** | Run the trained path policy (Swerve-Policy-Playground) in Java on SystemCore, for autos written with `new AutoSequence(Driver.POLICY)`. Plan: [docs/2027-auto-plan.md](docs/2027-auto-plan.md), sections 5 and 7 |
| **Auto path tester** | Run every auto in the simulator and fail on a collision, leaving the field, and similar problems |
| **Better simulation** | CTRE motor simulation plus a simulated battery (current draw, voltage sag) |

See the 1507Base wiki for how the base works, and its **Future Plans** page for the ideas queued for Labs.

## Building

```bash
./gradlew build          # compiles and runs every test
./gradlew simulateJava   # robot simulator (use the Sim GUI; don't also run the Driver Station app)
```
