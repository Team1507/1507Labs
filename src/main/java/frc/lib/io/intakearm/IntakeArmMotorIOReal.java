//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakearm;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import frc.lib.core.framework.io.CtreMotorIO;
import frc.lib.core.framework.io.IOReal1507;
import frc.lib.core.math.GearRatio;
import frc.lib.core.util.MotorConfig;

/**
 * Real hardware IO for a single intake arm motor.
 */
public class IntakeArmMotorIOReal extends IOReal1507<TalonFXS> {

    private final GearRatio ratio;
    private final MotorConfig config;

    public IntakeArmMotorIOReal(
        int canId,
        MotorConfig config,
        GearRatio ratio
    ) {
        super(
            createMotor(canId),
            createTelemetry(createMotor(canId))
        );

        this.config = config;
        this.ratio = ratio;
    }

    private static TalonFXS createMotor(int canId) {
        return new TalonFXS(canId);
    }

    private static CtreMotorIO createTelemetry(TalonFXS motor) {
        return new CtreMotorIO(
            motor.getPosition(),
            motor.getVelocity(),
            motor.getMotorVoltage(),
            motor.getStatorCurrent(),
            motor.getDeviceTemp(),
            100
        );
    }

    public void updateInputs(IntakeArmMotorInputs inputs) {
        super.updateInputs(inputs.motor);

        inputs.positionDeg =
            ratio.sensorToReal(inputs.motor.motorRot);

        inputs.reverseLimit =
            motor.getReverseLimit().getValue()
                == ReverseLimitValue.ClosedToGround;
    }

    public void setAngle(double degrees, double ffVolts) {
        double motorRot = ratio.realToSensor(degrees);

        motor.setControl(
            new PositionVoltage(motorRot)
                .withSlot(0)
                .withFeedForward(ffVolts)
        );
    }

    @Override
    public void run(double duty) {
        motor.set(duty);
    }

    @Override
    public void stop() {
        motor.set(0.0);
    }
}
