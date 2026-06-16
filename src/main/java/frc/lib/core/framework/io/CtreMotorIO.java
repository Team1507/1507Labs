//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.framework.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * CTRE motor observation wrapper.
 *
 * Owns all standard CTRE status signals for a single motor and
 * populates MotorInputs1507 in a consistent, efficient way.
 */
public final class CtreMotorIO {

    private static final double DEFAULT_UPDATE_HZ = 100.0;

    private final StatusSignal<Angle> positionSig;
    private final StatusSignal<AngularVelocity> velocitySig;
    private final StatusSignal<Voltage> voltageSig;
    private final StatusSignal<Current> currentSig;
    private final StatusSignal<Temperature> tempSig;

    private CtreMotorIO(
        StatusSignal<Angle> position,
        StatusSignal<AngularVelocity> velocity,
        StatusSignal<Voltage> voltage,
        StatusSignal<Current> current,
        StatusSignal<Temperature> temp
    ) {
        this.positionSig = position;
        this.velocitySig = velocity;
        this.voltageSig  = voltage;
        this.currentSig  = current;
        this.tempSig     = temp;

        BaseStatusSignal.setUpdateFrequencyForAll(
            DEFAULT_UPDATE_HZ,
            positionSig,
            velocitySig,
            voltageSig,
            currentSig,
            tempSig
        );
    }

    // ------------------------------------------------
    // Factory
    // ------------------------------------------------

    public static CtreMotorIO fromMotor(Object motor) {
        if (motor instanceof TalonFX fx) {
            return new CtreMotorIO(
                fx.getPosition(),
                fx.getVelocity(),
                fx.getMotorVoltage(),
                fx.getStatorCurrent(),
                fx.getDeviceTemp()
            );
        }

        if (motor instanceof TalonFXS fxs) {
            return new CtreMotorIO(
                fxs.getPosition(),
                fxs.getVelocity(),
                fxs.getMotorVoltage(),
                fxs.getStatorCurrent(),
                fxs.getDeviceTemp()
            );
        }

        throw new IllegalArgumentException("Unsupported motor type");
    }

    // ------------------------------------------------
    // Update
    // ------------------------------------------------

    public void updateInputs(MotorInputs1507 inputs) {
        BaseStatusSignal.refreshAll(
            positionSig,
            velocitySig,
            voltageSig,
            currentSig,
            tempSig
        );

        inputs.motorRot     = positionSig.getValueAsDouble();
        inputs.velocity     = velocitySig.getValueAsDouble();
        inputs.appliedVolts = voltageSig.getValueAsDouble();
        inputs.currentA     = currentSig.getValueAsDouble();
        inputs.tempC        = tempSig.getValueAsDouble();
    }
}
