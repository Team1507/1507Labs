//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.climber;

/**
 * Container for climber motor + servo sensor inputs.
 */
public class ClimberInputs {

    /** Current climber output position (after gear ratio), in mechanism units. */
    public double position = 0.0;

    /** Raw motor position (motor rotations). */
    public double motorPosition = 0.0;

    /** Motor current (amps). */
    public double currentA = 0.0;

    /** Motor temperature (Celsius). */
    public double temperatureC = 0.0;

    /** Whether the limit switch is pressed. */
    public boolean limitSwitch = false;

    /** Current servo position (0–1). */
    public double servoPosition = 0.0;
}
