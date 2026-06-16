//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Hardware abstraction layer for the swerve drivetrain.
 *
 * <p>This interface defines all hardware-level operations that the
 * SwerveSubsystem may require. It allows the robot code to remain
 * completely independent of vendor-specific implementations such as
 * CTRE Phoenix, WPILib, or custom simulation models.
 *
 * <p>Implementations:
 * <ul>
 *   <li>{@code SwerveIOReal} wraps CTRE's CommandSwerveDrivetrain</li>
 *   <li>{@code SwerveIOSim} provides physics-based simulation</li>
 *   <li>{@code SwerveIOReplay} (optional) for log replay</li>
 * </ul>
 */
public interface SwerveIO {

    /**
     * Updates the provided inputs structure with the latest sensor data.
     *
     * @param inputs container to populate with drivetrain state
     */
    void updateInputs(SwerveInputs inputs);

    /**
     * Drives the robot using chassis speeds (vx, vy, omega).
     *
     * <p>This is the primary teleop control method.
     *
     * @param speeds desired chassis speeds in robot-relative coordinates
     */
    void drive(ChassisSpeeds speeds);

    /** Drives the robot using robot-relative chassis speeds (vx, vy, omega). */
    void driveRobotRelative(ChassisSpeeds speeds);

    /**
     * Directly sets the module states.
     *
     * <p>This is required for trajectory following (PathPlanner, WPILib
     * HolonomicDriveController, etc.).
     *
     * @param states desired module states
     */
    void setModuleStates(SwerveModuleState[] states);

    /**
     * Returns the current estimated pose of the robot.
     *
     * @return the drivetrain pose
     */
    Pose2d getPose();

    /**
     * Resets the drivetrain pose estimator to the given pose.
     *
     * @param pose new pose to apply
     */
    void resetPose(Pose2d pose);

    /**
     * Seeds the pose estimator using a vision measurement.
     *
     * @param pose pose from vision system
     * @param timestamp
     * @param stdDevs
     */
    void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);

    /**
     * This will reset the current heading of the drivetrain and pigeon
     * 
     * @param heading the heading to reset to
     */
    void alignGyro(Rotation2d heading);

    /**
     * Returns the Pigeon Yaw
     */
    Rotation2d getHeading();

    /**
     * Stops all module motion.
     */
    void stop();
}

