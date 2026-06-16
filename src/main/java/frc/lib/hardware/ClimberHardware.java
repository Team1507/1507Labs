//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.hardware;

import edu.wpi.first.math.geometry.Transform2d;
import frc.lib.core.math.GearRatio;

/**
 * Hardware definition for the climber.
 */
public final class ClimberHardware {

    /** CAN ID for the climber motor. */
    public static final int CLIMBER_MOTOR_ID = 23;

    /** PWM port for the ratchet servo. */
    public static final int SERVO_PORT = 0;

    /** Limit switch DIO port (if used). */
    public static final int LIMIT_SWITCH_PORT = 9; // example, update as needed

    /** Gear ratio for the climber gearbox. */
    public static final GearRatio RATIO = 
        GearRatio
            .gearBox(64, 1)
            .withScaling(0.0, 23.0, 1.0, 27.0);

    /** Transform from robot center to climber (optional). */
    public static final Transform2d ROBOT_TO_CLIMBER = new Transform2d();

    private ClimberHardware() {}
}
