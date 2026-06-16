//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.framework.io;

/**
 * Base IO implementation for a single motor.
 *
 * Owns the motor hardware reference and standard telemetry.
 * Subclasses add specialized control behavior.
 */
public abstract class IOReal1507<T> {

    protected final T motor;
    protected final CtreMotorIO telemetry;

    protected IOReal1507(T motor, CtreMotorIO telemetry) {
        this.motor = motor;
        this.telemetry = telemetry;
    }

    /** Populate standard motor telemetry. */
    public void updateInputs(MotorInputs1507 inputs) {
        telemetry.updateInputs(inputs);
    }

    /** Default open-loop control (optional override). */
    public abstract void run(double duty);

    /** Standard stop behavior. */
    public abstract void stop();
}
