package frc.robot.mechanics;

/**
 * Physics-based flywheel model using GearRatio.
 *
 * Public API uses RPM for convenience.
 * Internal math uses RPS for correctness.
 */
public class FlywheelModel {

    private final double inertia;        // kg·m² (wheel inertia)
    private final double motorKv;        // rad/s per volt
    private final double motorKt;        // N·m per amp
    private final double motorResistance;// ohms
    private final double frictionTorque; // N·m
    private final GearRatio ratio;       // motor-to-wheel ratio

    public FlywheelModel(
        double inertia,
        double motorKv,
        double motorKt,
        double motorResistance,
        double frictionTorque,
        GearRatio ratio
    ) {
        this.inertia = inertia;
        this.motorKv = motorKv;
        this.motorKt = motorKt;
        this.motorResistance = motorResistance;
        this.frictionTorque = frictionTorque;
        this.ratio = ratio;
    }

    // -----------------------------
    // Helpers: RPM ↔ RPS
    // -----------------------------
    private static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    private static double rpsToRpm(double rps) {
        return rps * 60.0;
    }

    // -----------------------------
    // Core physics
    // -----------------------------

    /** Compute wheel acceleration (RPS²) from wheel speed (RPM) and voltage. */
    public double computeAccelerationRPM(double wheelRPM, double voltage) {
        double wheelRPS = rpmToRps(wheelRPM);

        // Convert wheel speed → motor speed
        double motorRPS = ratio.toMotor(wheelRPS);

        // Convert motor speed to rad/s for physics
        double motorRadPerSec = motorRPS * 2 * Math.PI;

        // Back-EMF
        double backEmf = motorRadPerSec / motorKv;

        // Motor current
        double current = (voltage - backEmf) / motorResistance;

        // Motor torque
        double motorTorque = current * motorKt;

        // Wheel torque after gearbox
        double wheelTorque = motorTorque * ratio.getRatio();

        // Subtract friction
        wheelTorque -= frictionTorque * Math.signum(wheelRPS);

        // Angular acceleration (rad/s²)
        double accelRad = wheelTorque / inertia;

        // Convert rad/s² → RPS²
        double accelRPS = accelRad / (2 * Math.PI);

        return accelRPS * 60.0; // return acceleration in RPM/s
    }

    /** Predict steady-state wheel RPM for a given voltage. */
    public double computeSteadyStateRPM(double voltage) {
        // Solve for motor speed where torque = friction
        double motorTorque = frictionTorque / ratio.getRatio();
        double current = motorTorque / motorKt;
        double backEmf = voltage - current * motorResistance;
        double motorRadPerSec = backEmf * motorKv;

        double motorRPS = motorRadPerSec / (2 * Math.PI);
        double wheelRPS = ratio.toOutput(motorRPS);

        return rpsToRpm(wheelRPS);
    }

    /** Simulate one timestep of flywheel motion. */
    public double stepRPM(double wheelRPM, double voltage, double dt) {
        double accelRPM = computeAccelerationRPM(wheelRPM, voltage);
        return wheelRPM + accelRPM * dt;
    }
}
