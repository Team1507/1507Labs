//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.feeder;

/**
 * Hardware abstraction for the feeder motor.
 */
public interface FeederIO {

    /** Populate the inputs struct with the latest sensor data. */
    void updateInputs(FeederInputs inputs);

    /** Run the feeder at a target RPM (mechanism units). */
    void runRPM(double rpm);

    /** Run the feeder in open-loop duty cycle mode. */
    void runDuty(double duty);

    /** Apply a direct voltage to the motor. */
    void setVoltage(double voltage);

    /** Stop the motor. */
    void stop();
}
