//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.lib.core.framework.base.Subsystems1507;
import frc.lib.core.util.MotorConfig;

/**
 * ShooterIOReal provides the real hardware implementation of the shooter IO layer.
 *
 * <p>This class is responsible for:
 * <ul>
 *   <li>Creating and configuring the TalonFX motor</li>
 *   <li>Applying PID/FF/voltage limits from {@link MotorConfig}</li>
 *   <li>Sending velocity commands to the motor</li>
 *   <li>Reading sensor values into {@link ShooterInputs}</li>
 * </ul>
 *
 * <p>No shooter logic, kinematics, or simulation belongs here — only hardware access.
 */
public class ShooterIOReal extends Subsystems1507 implements ShooterIO {

    /** Underlying TalonFX motor controller for the shooter. */
    private final TalonFX motor;

    /** Configuration for this motor */
    private final MotorConfig config;

    /** Digital IO for the shooter ball sensor */
    private final DigitalInput ballSensor;

    /** Phoenix 6 velocity control request reused each cycle. */
    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0).withSlot(0);

    /**
     * Creates a real shooter IO implementation.
     *
     * @param canID  CAN ID of the shooter motor
     * @param config Motor configuration containing PID/FF/voltage limits
     */
    public ShooterIOReal(int canID, MotorConfig config, int sensorDIO) {
        this.motor = new TalonFX(canID);
        this.config = config;

        this.ballSensor = new DigitalInput(sensorDIO);

        // Apply PID/FF/voltage limits to the motor
        configureFXMotor(motor, config);
    }

    /**
     * Populates the {@link ShooterInputs} structure with real sensor data.
     *
     * @param inputs container for shooter sensor readings
     */
    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.motorRPS = motor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();

        inputs.ballFired = getBallSensor();
    }

    /**
     * Sends a velocity command to the shooter motor.
     *
     * @param motorRPS target motor velocity in RPS (Phoenix native units)
     */
    @Override
    public void setTargetRPS(double motorRPS) {
        motor.setControl(velocityRequest.withVelocity(motorRPS));
    }

    /**
     * 
     */
    @Override
    public boolean getBallSensor() {
        return !ballSensor.get(); // depending on wiring
    }

    /**
     * Stops the shooter motor by commanding zero velocity.
     */
    @Override
    public void stop() {
        motor.setControl(velocityRequest.withVelocity(0));
    }

    @Override
    public double getKV() { return config.kV(); }

    @Override
    public double getKS() { return config.kS(); }

    @Override
    public double getKP() { return config.kP(); }

    @Override
    public double getKI() { return config.kI(); }

    @Override
    public double getKD() { return config.kD(); }
}
