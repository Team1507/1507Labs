//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.team1507.lib.core.framework.LoggedRobot;
import org.team1507.lib.core.vision.QuestNavSubsystem;
import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.auto.nodes.Nodes;
import org.team1507.robot.auto.routines.*;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.Constants.kQuest;
import org.team1507.robot.Constants.kSwerve;
import org.team1507.robot.subsystems.*;

public final class Robot extends LoggedRobot {

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------

    public final Swerve            swerve;
    public final QuestNavSubsystem questNav;
    public final PolicyAssist      policy;

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    private final CommandXboxController driver;

    // -------------------------------------------------------------------------
    // Autonomous
    // -------------------------------------------------------------------------

    private Command m_autoCommand = null;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // =========================================================================
    // Constructor
    // =========================================================================

    public Robot() {

        // Subsystems — swerve first; questNav takes method references from it.
        swerve    = new Swerve();
        policy    = new PolicyAssist();
        questNav  = new QuestNavSubsystem(
            swerve::addVisionMeasurement,
            swerve::resetPose,
            kQuest.ROBOT_TO_QUEST
        );

        // Pre-match pose preset buttons (visible in Elastic while disabled).
        // Place the robot at the known starting position and press the matching
        // button. The command waits for Quest to confirm before snapping odometry.
        questNav.setKnownPoseCommand(Nodes.Robot.Start.LEFT)
            .named("Set Pose Left")
            .publishToDashboard();
        questNav.setKnownPoseCommand(Nodes.Robot.Start.CENTER)
            .named("Set Pose Start")
            .publishToDashboard();
        questNav.setKnownPoseCommand(Nodes.Robot.Start.RIGHT)
            .named("Set Pose Right")
            .publishToDashboard();

        // Autonomous chooser
        AutoBuilder.init(swerve);
        autoChooser.setDefaultOption("Drive Forward", DriveForwardAuto.build());
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Controllers and bindings
        driver = new CommandXboxController(RobotMap.DRIVER_CONTROLLER);
        configureBindings();
        configureDefaultBindings();
    }

    // =========================================================================
    // Bindings
    // =========================================================================

    private void configureBindings() {

        // ── Driver — swerve utilities ──────────────────────────────────────

        // Swerve brake — hold Start to lock wheels in X pattern, release to resume driving
        driver.start().whileTrue(swerve.brakeCommand());

        // Policy assist toggle — right bumper enables/disables the Python bridge policy.
        // The robot falls back to raw joystick automatically if the bridge goes silent.
        driver.rightBumper().onTrue(Commands.runOnce(() -> policy.setEnabled(!policy.isEnabled())));

        // Failsafe — cancels all running commands (see RobotBehaviors for details)
        driver.back().onTrue(RobotBehaviors.failsafe());

        // Point the robot toward the opposing alliance wall, then press A
        // to zero the gyro. Do this after any hot code deploy without a power cycle.
        driver.a().onTrue(swerve.zeroHeadingCommand());
    }

    private void configureDefaultBindings() {

        swerve.setDefaultCommand(
            swerve.driveCommand(() -> {
                // Raw values — no deadband — sent to the policy.
                // The policy was trained on raw stick input with injected drift
                // and learns to threshold internally.
                double rawY   = -driver.getLeftY();
                double rawX   = -driver.getLeftX();
                double rawRot = -driver.getRightX();

                policy.update(
                    rawY, rawX, rawRot,
                    swerve.getPose(),
                    swerve.getChassisSpeeds(),
                    swerve.getHeading().getRadians()
                );

                if (policy.isActive()) {
                    ChassisSpeeds ps = policy.getPolicySpeeds();
                    // Policy handles translation only (phase 1 — omega not trained).
                    // Rotation comes directly from the driver so the right stick always works.
                    double omega = MathUtil.applyDeadband(rawRot, 0.12) * Math.PI;
                    return new ChassisSpeeds(ps.vxMetersPerSecond, ps.vyMetersPerSecond, omega);
                }

                // Fallback: apply deadband only on the raw joystick path.
                double joyY   = MathUtil.applyDeadband(rawY,   0.12);
                double joyX   = MathUtil.applyDeadband(rawX,   0.12);
                double joyRot = MathUtil.applyDeadband(rawRot, 0.12);
                return ChassisSpeeds.fromFieldRelativeSpeeds(
                    joyY   * kSwerve.MAX_SPEED,
                    joyX   * kSwerve.MAX_SPEED,
                    joyRot * Math.PI,
                    swerve.getHeading()
                );
            })
        );
    }

    // =========================================================================
    // Mode callbacks
    // =========================================================================

    @Override
    public void autonomousInit() {
        m_autoCommand = autoChooser.getSelected();
        if (m_autoCommand != null) {
            CommandScheduler.getInstance().schedule(m_autoCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (m_autoCommand != null) {
            m_autoCommand.cancel();
        }
    }
}
