//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.framework.io;

/**
 * Standardized motor telemetry container.
 * 
 * This class represents the minimum set of data that every motor
 * on the robot is expected to provide, regardless of subsystem.
 *
 * All values are raw motor-space measurements.
 */
public class MotorInputs1507 {

    /** Motor position in rotations. */
    public double motorRot = 0.0;

    /** Motor velocity in rotations per second. */
    public double velocity = 0.0;

    /** Applied motor voltage. */
    public double appliedVolts = 0.0;

    /** Motor stator current in amps. */
    public double currentA = 0.0;

    /** Motor temperature in Celsius. */
    public double temperatureC = 0.0;
}
