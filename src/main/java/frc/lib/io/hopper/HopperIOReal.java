//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.hopper;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;

import frc.lib.core.framework.base.Subsystems1507;
import frc.lib.core.math.GearRatio;
import frc.lib.core.util.MotorConfig;
import frc.robot.Constants.kHopper;

/**
 * Real hardware implementation of HopperIO using a TalonFXS.
 */
public class HopperIOReal extends Subsystems1507 implements HopperIO {

    private final TalonFXS motor;
    private final GearRatio ratio;
    private final DigitalInput halSensor;

    private final PositionDutyCycle positionRequest =
        new PositionDutyCycle(0).withSlot(0);

    public HopperIOReal(int canID, MotorConfig config, GearRatio ratio, int sensorDIO) {
        this.motor = new TalonFXS(canID);
        this.ratio = ratio;
        this.halSensor = new DigitalInput(sensorDIO);

        configureFXSMotor(motor, config);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        // Raw sensor units from TalonFXS
        inputs.motorRot = motor.getPosition().getValueAsDouble();

        // Convert sensor units → real-world inches using scaling
        inputs.position = ratio.sensorToReal(inputs.motorRot);

        inputs.currentA = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureC = motor.getDeviceTemp().getValueAsDouble();
        inputs.reverseLimit = !halSensor.get();
        if(inputs.reverseLimit){
            motor.setPosition(0);
        }

        // Hopper is extended if real-world inches exceed safe threshold
        inputs.hopperExtended = inputs.position > kHopper.SAFE_EXTENDED;
    }

    @Override
    public void setPosition(double position) {
        // Hopper is linear, not rotational — convert inches instead
        double safePosition = position; // if commands still use degrees, rename later
        if(!halSensor.get() && position < 0) {
            safePosition = 0.0;
        }
        double sensorUnits = ratio.realToSensor(safePosition);

        
        motor.setControl(positionRequest.withPosition(sensorUnits));
    }

    @Override
    public boolean getMagSensor() {
        return !halSensor.get();
    }

    @Override
    public void runPower(double power) {
        boolean atRev = !halSensor.get();
        double safePower =power;
        if(atRev && power < 0) {
            safePower = 0;
        }
        motor.set(safePower);
    }

    @Override
    public void hopperStop() {
        if (getMagSensor()) {
            motor.set(0);
        }
    }
}
