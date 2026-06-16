//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.lib.core.math.geometry.VisionMeasurement;
import frc.lib.io.swerve.SwerveIO;
import frc.lib.io.swerve.SwerveInputs;
import frc.robot.localization.vision.Vision;

/**
 * Thin, IO-based swerve subsystem.
 *
 * <p>This subsystem contains no vendor-specific code. All hardware
 * interaction is delegated to the SwerveIO layer (SwerveIOReal or
 * SwerveIOSim). This keeps the subsystem clean, testable, and
 * consistent with the architecture used for the shooter.
 */
public class SwerveSubsystem extends SubsystemBase {

    private final SwerveIO io;
    private final SwerveInputs inputs = new SwerveInputs();

    private Vision vision;

    public SwerveSubsystem(SwerveIO io) {
        this.io = io;
    }

    public void setVision(Vision vision) {
        this.vision = vision;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        // Log fused robot pose in 3D (flat on ground, Z = 0)
        Logger.recordOutput("Vision/RobotPose3d", new Pose3d(inputs.pose));

        if (vision != null) {
            vision.addHeadingData(Timer.getFPGATimestamp(), io.getHeading());

            VisionMeasurement[] measurements = vision.getUnreadResults();
            Pose3d[] photonPoses = new Pose3d[measurements.length];
            for (int i = 0; i < measurements.length; i++) {
                VisionMeasurement m = measurements[i];
                io.addVisionMeasurement(m.pose(), m.timestamp(), m.stdDevs());
                photonPoses[i] = new Pose3d(m.pose());
            }
            Logger.recordOutput("Vision/PhotonVision/AcceptedPoses3d", photonPoses);
        }
    }

    // ==========================================================
    // Control API
    // ==========================================================

    /** Drive the robot using chassis speeds (vx, vy, omega). */
    public void drive(ChassisSpeeds speeds) {
        io.drive(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        io.driveRobotRelative(speeds);
    }

    /** Changes the position of the swerve modules to lock the robot in place */
    public void lock() {
        SwerveModuleState[] lockStates = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };

        io.setModuleStates(lockStates);
    }

    /** Directly set module states (used by autos / PathPlanner). */
    public void setModuleStates(SwerveModuleState[] states) {
        io.setModuleStates(states);
    }

    /** Stop all motion. */
    public void stop() {
        io.stop();
    }

    // ==========================================================
    // Pose + Odometry
    // ==========================================================

    /** Get the current estimated robot pose. */
    public Pose2d getPose() {
        return inputs.pose;
    }

    /** Get the current heading of the gyro */
    public Rotation2d getHeading() {
        return io.getHeading();
    }

    /** Reset the robot pose (odometry only). */
    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    /** Resets the pose estimator and vision heading history. */
    public void resetLocalization(Pose2d pose) {
        io.resetPose(pose);
        if (vision != null) {
            vision.resetHeadingData(Timer.getFPGATimestamp(), pose.getRotation());
        }
    }

    public void setTemporaryTargetPose(Pose2d tempPose) {
        inputs.tempPose = tempPose;
    }

    public Pose2d getTemporaryTargetPose() {
        return inputs.tempPose;
    }

    /** Adds a vision measurement (delegates to IO). */
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        io.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void setPoseFromVision(Pose2d visionPose) {
        // Align gyro to match the vision heading
        io.alignGyro(visionPose.getRotation());

        // Reset pose estimator
        io.resetPose(visionPose);
    }

    // ==========================================================
    // Access to raw inputs (optional)
    // ==========================================================

    public SwerveInputs getInputs() {
        return inputs;
    }
}
