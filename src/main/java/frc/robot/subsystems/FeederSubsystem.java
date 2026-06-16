//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.feeder.FeederIO;
import frc.lib.io.feeder.FeederInputs;

/**
 * Thin, IO-based feeder subsystem.
 */
public class FeederSubsystem extends SubsystemBase {

    private final FeederIO io;
    private final FeederInputs inputs = new FeederInputs();

    public FeederSubsystem(FeederIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void runRPM(double rpm) {
        io.runRPM(rpm);
    }

    public void runDuty(double duty) {
        io.runDuty(duty);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.stop();
    }

    public FeederInputs getInputs() {
        return inputs;
    }

    public double getVelocityRPM() {
        return inputs.velocityRPM;
    }
}
