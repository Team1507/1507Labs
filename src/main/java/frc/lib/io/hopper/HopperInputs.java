//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.hopper;

/**
 * Container for hopper motor sensor inputs.
 */
public class HopperInputs {

    /** Hopper position in degrees. */
    public double positionDeg;

    public double position = 0.0;

    public boolean magSensor = false;

    /** Raw motor rotations. */
    public double motorRot = 0.0;

    /** Motor current (amps). */
    public double currentA = 0.0;

    /** Motor temperature (Celsius). */
    public double temperatureC = 0.0;

    public boolean hopperRetracted = false;

    public boolean hopperExtended = false;

    public boolean reverseLimit = false;
}
