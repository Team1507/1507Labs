//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakeroller;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.core.framework.base.Subsystems1507;
import frc.lib.core.util.MotorConfig;

/**
 * Real hardware implementation of IntakeRollerIO using a TalonFX.
 */
public class IntakeRollerIOReal extends Subsystems1507 implements IntakeRollerIO {

    private final TalonFX motor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    public IntakeRollerIOReal(int canID, MotorConfig config) {
        this.motor = new TalonFX(canID);
        configureFXMotor(motor, config);
    }

    @Override
    public void updateInputs(IntakeRollerInputs inputs) {
        inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();
        inputs.currentA = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureC = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void runDuty(double duty) {
        motor.setControl(dutyRequest.withOutput(duty));
    }

    @Override
    public void runPower(double rollerSpeed) {
        motor.set(rollerSpeed);
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    public void increaseSpeed(double rollerSpeed) {
        rollerSpeed += 0.1;
    }

    public void resetSpeed(double rollerSpeed) {
        rollerSpeed = 0.3;
    }
}
