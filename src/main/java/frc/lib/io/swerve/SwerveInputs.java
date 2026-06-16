//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Container for all swerve drivetrain sensor inputs.
 *
 * <p>This is populated by SwerveIOReal or SwerveIOSim and consumed
 * by SwerveSubsystem. It contains no vendor-specific types.
 */
public class SwerveInputs {

    /** Current estimated robot pose */
    public Pose2d pose = new Pose2d();
    public Pose2d tempPose = new Pose2d();

    /** Current robot-relative chassis speeds */
    public ChassisSpeeds speeds = new ChassisSpeeds();

    /** Current module states (drive velocity + steer angle) */
    public SwerveModuleState[] moduleStates = new SwerveModuleState[0];
}
