//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.hardware;

import edu.wpi.first.math.geometry.Transform2d;

/**
 * Hardware definition for the agitator motor.
 *
 * <p>This class contains ONLY physical hardware information:
 * <ul>
 *   <li>CAN IDs</li>
 *   <li>Gear ratios (if any)</li>
 *   <li>Robot-to-mechanism transforms (if needed)</li>
 * </ul>
 *
 * <p>No tuning values, no vendor-specific code, no logic.
 */
public final class AgitatorHardware {

    /** CAN ID for the agitator motor. */
    public static final int AGITATOR_ID = 15;

    /** Gear ratio (if the agitator is direct drive, this stays 1:1). */
    public static final double AGITATOR_RATIO = 1.0;

    /** Transform from robot center to agitator (optional, leave identity). */
    public static final Transform2d ROBOT_TO_AGITATOR = new Transform2d();

    private AgitatorHardware() {}
}
