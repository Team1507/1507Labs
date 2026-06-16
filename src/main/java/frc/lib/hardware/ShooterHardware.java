//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.hardware;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.core.math.GearRatio;

/**
 * ShooterHardware defines all physical shooter-related hardware information.
 * 
 * This includes:
 *  - CAN IDs
 *  - Gear ratios
 *  - Physical transforms (robot → shooter)
 * 
 * No tuning values (PID, FF, voltage limits) belong here.
 * No behavior or logic belongs here.
 * No simulation or IO code belongs here.
 */
public final class ShooterHardware {

    // ------------------------------------------------------------
    // CAN IDs
    // ------------------------------------------------------------
    public static final int BLU_ID = 19;
    public static final int YEL_ID = 17;

    // ------------------------------------------------------------
    // Sensor DIO
    // ------------------------------------------------------------
    public static final int BLU_DIO = 6;
    public static final int YEL_DIO = 7;
    

    // ------------------------------------------------------------
    // Gear Ratio (motor → wheel)
    // ------------------------------------------------------------
    public static final GearRatio BLU_RATIO = GearRatio.gearBox(1, 1);
    public static final GearRatio YEL_RATIO = GearRatio.gearBox(1, 1);

    // ------------------------------------------------------------
    // Physical transform: robot origin → shooter mechanism
    // (Used for pose-based targeting and model-driven shooting)
    // ------------------------------------------------------------
    public static final Transform2d ROBOT_TO_BLU_SHOOTER =
        new Transform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));
    public static final Transform2d ROBOT_TO_YEL_SHOOTER =
        new Transform2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0));


    // Prevent instantiation
    private ShooterHardware() {}
}
