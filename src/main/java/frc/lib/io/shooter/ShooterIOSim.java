//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.shooter;

import frc.lib.core.math.FlywheelModel;
import frc.lib.core.math.GearRatio;
import frc.lib.core.util.MotorConfig;
import frc.robot.Constants.kShooter;

/**
 * ShooterIOSim implements a physics-based simulation of the shooter flywheel.
 *
 * <p>This class mirrors the behavior of ShooterIOReal but without any hardware.
 * It uses:
 *  - FlywheelModel for physics
 *  - MotorConfig for PID + feedforward tuning
 *  - GearRatio (passed per-shooter) for motor↔wheel conversion
 */
public class ShooterIOSim implements ShooterIO {

    // ------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------
    private final MotorConfig config;
    private final FlywheelModel flywheel;
    private final GearRatio ratio;

    // ------------------------------------------------------------
    // Simulation state
    // ------------------------------------------------------------
    private double simWheelRPM = 0.0;
    private double simVoltage = 0.0;
    private double simMotorRpsMeasured = 0.0;
    private double simMotorRpsCommanded = 0.0;

    // Target motor RPS (Phoenix native units)
    private double targetMotorRPS = 0.0;

    public ShooterIOSim(MotorConfig config, FlywheelModel flywheel, GearRatio ratio) {
        this.config = config;
        this.flywheel = flywheel;
        this.ratio = ratio;
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        // Convert wheel RPM → motor RPS
        double wheelRPS = simWheelRPM / 60.0;
        double motorRPS = ratio.toMotor(wheelRPS);

        inputs.motorRPS = motorRPS;
        inputs.appliedVolts = simVoltage;

        // Simple current model
        inputs.statorCurrent = 5.0 + Math.abs(simWheelRPM) / 1000.0;
        inputs.supplyCurrent = inputs.statorCurrent;
    }

    @Override
    public void setTargetRPS(double motorRPS) {
        this.targetMotorRPS = motorRPS;
    }

    @Override
    public boolean getBallSensor() {
        return false; // or simulate later
    }

    @Override
    public void stop() {
        this.targetMotorRPS = 0.0;
    }

    // ------------------------------------------------------------
    // Simulation step (20ms)
    // ------------------------------------------------------------
    public void simulate(double dtSeconds) {

        // Convert wheel RPM → motor RPS
        double wheelRPS = simWheelRPM / 60.0;
        double motorRPS = ratio.toMotor(wheelRPS);

        // 1. Sensor filtering
        double alphaSensor = dtSeconds / (kShooter.kSim.SENSOR_FILTER_TIME_CONSTANT + dtSeconds);
        simMotorRpsMeasured += alphaSensor * (motorRPS - simMotorRpsMeasured);

        // 2. Command filtering
        double alphaCommand = dtSeconds / (kShooter.kSim.COMMAND_FILTER_TIME_CONSTANT + dtSeconds);
        simMotorRpsCommanded += alphaCommand * (targetMotorRPS - simMotorRpsCommanded);

        // 3. Phoenix-like control law
        double errorRPS = simMotorRpsCommanded - simMotorRpsMeasured;

        double ffVolts = config.kV() * simMotorRpsCommanded;
        double ksVolts = config.kS() * Math.signum(simMotorRpsCommanded);
        double fbVolts = config.kP() * errorRPS;

        double desiredVolts = ffVolts + ksVolts + fbVolts;

        // 4. Voltage slew rate limiting
        double maxStep = kShooter.kSim.VOLTAGE_SLEW_RATE * dtSeconds;
        double delta = desiredVolts - simVoltage;

        if (delta > maxStep) delta = maxStep;
        if (delta < -maxStep) delta = -maxStep;

        simVoltage += delta;

        // Clamp to battery
        simVoltage = Math.max(-kShooter.kSim.MAX_VOLTAGE,
                              Math.min(kShooter.kSim.MAX_VOLTAGE, simVoltage));

        // 5. Step flywheel physics
        if (flywheel != null)
            simWheelRPM = flywheel.stepRPM(simWheelRPM, simVoltage, dtSeconds);
    }

    /** Resets all simulation state variables to zero. */
    public void reset() {
        simWheelRPM = 0;
        simMotorRpsMeasured = 0;
        simMotorRpsCommanded = 0;
        simVoltage = 0;
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
