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
 * Hardware definition for the feeder motors.
 */
public final class FeederHardware {

    /** CAN ID for the BLUE feeder motor. */
    public static final int BLU_ID = 20;

    /** CAN ID for the YELLOW feeder motor. */
    public static final int YEL_ID = 18;

    /** Gear ratio for both feeders (1:1 unless changed). */
    public static final GearRatio RATIO = GearRatio.gearBox(1, 1);

    private FeederHardware() {}
}
