# 1507Labs

**1507Labs** is the experimental robot codebase for [Team 1507 – Warlocks](https://warlocks1507.com). It is a sandbox for prototyping new systems before they graduate into the production [1507Base](https://github.com/Team1507/1507Base) codebase.

See [EXPERIMENTS_OVERVIEW.txt](EXPERIMENTS_OVERVIEW.txt) for a full catalogue of what has been built here and the prioritization plan for moving features into production.

## Active Experiments

| System | Subsystem/Package | Description |
|---|---|---|
| Policy Assist | `subsystems/PolicyAssist.java` | RL teleop-assist policy via NetworkTables bridge |
| Shooter | `subsystems/Shooter.java` | Flywheel shooter with PID tuner integration |
| Swerve | `subsystems/Swerve.java` | Swerve drivetrain with vision fusion support |

## Structure

```
src/main/java/org/team1507/
  robot/
    Constants.java        Tunable parameters
    Robot.java            WPILib Robot entry point
    RobotBehaviors.java   Input bindings
    auto/                 Autonomous routines
    subsystems/           Active experiment subsystems
  lib/
    core/                 Shared utilities shared with 1507Base
```

## Setup

1. Install [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
2. Open in VS Code with the WPILib extension
3. Simulate: `./gradlew simulateJava`
4. Build: `./gradlew build`

## License

Uses WPILib components — see [WPILib-License.md](WPILib-License.md).
