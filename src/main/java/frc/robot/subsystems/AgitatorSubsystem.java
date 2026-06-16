//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.core.framework.motor.Motor1507;
import frc.lib.core.framework.motor.Motor1507.Type;
import frc.lib.hardware.AgitatorHardware;
import frc.lib.core.framework.io.MotorInputs1507;
import frc.lib.io.agitator.AgitatorInputs;
import frc.robot.Constants.kAgitator;

/**
 * Agitator subsystem.
 *
 * Owns a single open-loop motor used to feed game pieces.
 */
public class AgitatorSubsystem extends SubsystemBase {

    private final Motor1507 motor;
    private final MotorInputs1507 motorInputs = new MotorInputs1507();
    private final AgitatorInputs inputs = new AgitatorInputs();

    public AgitatorSubsystem() {
        motor = new Motor1507(
            Type.FX,
            AgitatorHardware.AGITATOR_ID,
            kAgitator.CONFIG
        );

        motor.applyConfig();
    }

    @Override
    public void periodic() {
        motor.update();

        inputs.dutyCycle    = motorInputs.appliedVolts / 12.0;
        inputs.currentA     = motorInputs.currentA;
        inputs.temperatureC = motorInputs.temperatureC;
    }

    /** Run the agitator at the given duty cycle (-1 to 1). */
    public void run(double dutyCycle) {
        motor.runDuty(dutyCycle);
        inputs.feedingEnabled = dutyCycle != 0.0;
    }

    /** Stop the agitator motor immediately. */
    public void stop() {
        motor.stop();
        inputs.feedingEnabled = false;
    }

    public AgitatorInputs getInputs() {
        return inputs;
    }

    public double getDutyCycle() {
        return inputs.dutyCycle;
    }
}
