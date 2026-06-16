//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.hardware;

import frc.lib.core.math.GearRatio;

/**
 * Hardware definition for the intake arm motors.
 */
public final class IntakeArmHardware {

    /** CAN ID for the BLUE intake arm motor. */
    public static final int BLU_ID = 13;

    /** CAN ID for the YELLOW intake arm motor. */
    public static final int YEL_ID = 14;

    /** Shared gear ratio for both arms. */
    public static final GearRatio RATIO = 
        GearRatio
            .gearBox(100, 1)
            .withScaling(0.0, 63.0, 135.0);

    private IntakeArmHardware() {}
}
