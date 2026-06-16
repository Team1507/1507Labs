//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.intakeroller.IntakeRollerIO;
import frc.lib.io.intakeroller.IntakeRollerInputs;

/**
 * Thin, IO-based intake roller subsystem.
 */
public class IntakeRollerSubsystem extends SubsystemBase {

    private final IntakeRollerIO io;
    private final IntakeRollerInputs inputs = new IntakeRollerInputs();

    public IntakeRollerSubsystem(IntakeRollerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void run() {
        io.runDuty(inputs.cmdDutyCycle);
    }

    public void run(double duty) {
        io.runDuty(duty);
    }

    public void runPower(double power) {
        io.runPower(power);
    }

    public void stop() {
        io.stop();
    }

    public IntakeRollerInputs getInputs() {
        return inputs;
    }

    public void setDutyCycle(double duty) {
        inputs.cmdDutyCycle = duty;
    }

    public double getDutyCycle() {
        return inputs.dutyCycle;
    }

    public double increaseRollerSpeed(double rollerSpeed) {
        io.increaseSpeed(rollerSpeed);
        return rollerSpeed;
    }
}
