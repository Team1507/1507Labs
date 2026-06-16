//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.feeder;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.core.framework.base.Subsystems1507;
import frc.lib.core.math.GearRatio;
import frc.lib.core.util.MotorConfig;
import frc.lib.hardware.FeederHardware;

/**
 * Real hardware implementation of FeederIO using a TalonFX.
 */
public class FeederIOReal extends Subsystems1507 implements FeederIO {

    private final TalonFX motor;
    private final GearRatio ratio = FeederHardware.RATIO;

    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0).withSlot(0);

    public FeederIOReal(int canID, MotorConfig config) {
        this.motor = new TalonFX(canID);
        
        configureFXMotor(motor, config);
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        inputs.motorRPS = motor.getVelocity().getValueAsDouble();
        double motorRPM = inputs.motorRPS * 60.0;
        inputs.velocityRPM = ratio.toOutput(motorRPM);

        inputs.currentA = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureC = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void runRPM(double rpm) {
        double motorRPM = ratio.toMotor(rpm);
        double motorRPS = motorRPM / 60.0;
        motor.setControl(velocityRequest.withVelocity(motorRPS));
    }

    @Override
    public void runDuty(double duty) {
        motor.set(duty);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.set(0);
    }
}
