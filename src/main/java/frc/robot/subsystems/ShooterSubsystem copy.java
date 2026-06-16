//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.core.logging.Telemetry;
import frc.lib.core.math.GearRatio;
import frc.lib.core.shooterML.ShooterTelemetryProvider;
import frc.lib.core.shooterML.data.ShotRecord;
import frc.lib.core.shooterML.data.ShotTrainer;
import frc.lib.core.shooterML.model.ShooterModel;
import frc.lib.io.shooter.ShooterIO;
import frc.lib.io.shooter.ShooterIOSim;
import frc.lib.io.shooter.ShooterInputs;
import frc.robot.framework.base.Subsystems1507;

/**
 * High‑level shooter subsystem using the IO architecture.
 *
 * <p>This class owns:
 * <ul>
 *   <li>Target RPM logic</li>
 *   <li>Model‑driven shooter behavior</li>
 *   <li>Pose‑based kinematics</li>
 *   <li>Telemetry logging</li>
 * </ul>
 *
 * <p>All hardware interaction is delegated to {@link ShooterIO}.
 * Simulation is handled by {@link ShooterIOSim}.
 */
public class ShooterSubsystem extends Subsystems1507 implements ShooterTelemetryProvider {

    private final ShooterIO io;
    private final ShooterInputs inputs = new ShooterInputs();

    private final GearRatio ratio;
    private final ShooterModel model;

    private final Supplier<Pose2d> poseSupplier;
    private Pose2d targetPose;
    private final ShooterKinematics kinematics;

    private double targetMotorRPS = 0.0;

    private final Telemetry telemetry;
    private final String shooterKey;

    private ShotTrainer shotTrainer;
    private boolean lastBallSensor = false;
    private double lastShotTime;
    // ------------------------------------------------------------
    // Constructors
    // ------------------------------------------------------------

    /**
     * Creates a basic shooter with no model‑driven behavior.
     *
     * @param io         shooter IO implementation (real or sim)
     * @param telemetry  telemetry logger
     * @param shooterKey logging key for this shooter
     */
    public ShooterSubsystem(
        ShooterIO io,
        GearRatio ratio,
        Transform2d shooterOffset,
        Telemetry telemetry,
        String shooterKey
    ) {
        this(io, ratio, shooterOffset, null, () -> new Pose2d(), new Pose2d(), telemetry, shooterKey);
    }

    /**
     * Creates a fully featured shooter subsystem.
     *
     * @param io           shooter IO implementation
     * @param model        optional shooter model for predictive RPM
     * @param poseSupplier supplier for robot pose
     * @param targetPose   target pose used by the shooter model
     * @param telemetry    telemetry logger
     * @param shooterKey   logging key for this shooter
     */
    public ShooterSubsystem(
        ShooterIO io,
        GearRatio ratio,
        Transform2d shooterOffset,
        ShooterModel model,
        Supplier<Pose2d> poseSupplier,
        Pose2d targetPose,
        Telemetry telemetry,
        String shooterKey
    ) {
        this.io = io;
        this.ratio = ratio;
        this.model = model;
        this.poseSupplier = poseSupplier;
        this.targetPose = targetPose;

        this.kinematics = new ShooterKinematics(shooterOffset);

        this.telemetry = telemetry;
        this.shooterKey = shooterKey;

        telemetry.registerShooterSource(shooterKey);
    }

    // ------------------------------------------------------------
    // ShotTrainer injection
    // ------------------------------------------------------------
    public void setShotTrainer(ShotTrainer trainer) { 
        this.shotTrainer = trainer; 
    }

    // ------------------------------------------------------------
    // Shooter control API
    // ------------------------------------------------------------

    /** Sets the desired wheel RPM for the shooter. */
    public void setTargetRPM(double wheelRPM) {
        double wheelRPS = wheelRPM / 60.0;
        targetMotorRPS = ratio.toMotor(wheelRPS);
    }

    /** Stops the shooter motor. */
    public void stop() {
        targetMotorRPS = 0.0;
    }

    /** @return current target wheel RPM */
    public double getTargetRPM() {
        return ratio.toOutput(targetMotorRPS) * 60.0;
    }

    // ------------------------------------------------------------
    // Model‑driven shooter
    // ------------------------------------------------------------

    /** Builds a telemetry snapshot used by the ShooterModel. */
    private ShotRecord buildTelemetry() {
        Pose2d robotPose = poseSupplier.get();
        Pose2d shooterPose = kinematics.shooterPose(robotPose);

        inputs.distanceToTarget = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

        return new ShotRecord(
            getShooterRPM(),
            inputs.appliedVolts,
            inputs.statorCurrent,
            inputs.supplyCurrent,
            getClosedLoopError(),
            shooterPose,
            inputs.distanceToTarget
        );
    }

    /** Updates the shooter target RPM using the ShooterModel. */
    public void updateShooterFromModel() {
        if (model == null) return;

        ShotRecord t = buildTelemetry();
        double rpm = model.getRPM(t);
        setTargetRPM(rpm);
    }

    // ------------------------------------------------------------
    // Derived telemetry values
    // ------------------------------------------------------------

    /** @return current wheel RPM (converted from motor RPS) */
    public double getShooterRPM() {
        return ratio.toOutput(inputs.motorRPS) * 60.0;
    }

    /** @return pose of the shooter mechanism on the field */
    public Pose2d getShooterPose() {
        return kinematics.shooterPose(poseSupplier.get());
    }

    /** @return distance from shooter to target pose */
    public double getDistanceToTarget() {
        return getShooterPose().getTranslation().getDistance(targetPose.getTranslation());
    }

    /** @return current targetPose */
    public Pose2d getTargetPose() {
        return targetPose;
    }

    public ShooterInputs getInputs() {
        return inputs;
    }

    /** @return applied voltage (real or simulated) */
    public double getShooterVoltage() {
        return inputs.appliedVolts;
    }

    /** @return stator current (real or simulated) */
    public double getStatorCurrent() {
        return inputs.statorCurrent;
    }

    /** @return supply current (real or simulated) */
    public double getSupplyCurrent() {
        return inputs.supplyCurrent;
    }

    /** @return closed‑loop error in wheel RPM */
    public double getClosedLoopError() {
        return getTargetRPM() - getShooterRPM();
    }

    public double getKV() { return io.getKV(); }
    public double getKS() { return io.getKS(); }
    public double getKP() { return io.getKP(); }
    public double getKI() { return io.getKI(); }
    public double getKD() { return io.getKD(); }

    public boolean getSensor() {return inputs.ballFired;}


     public boolean atVelocity() {
        return Math.abs(getClosedLoopError()) <= 100.0;
     }
    // ------------------------------------------------------------
    // Periodic
    // ------------------------------------------------------------

    @Override
    public void periodic() {

        io.updateInputs(inputs);

        if (RobotBase.isSimulation() && io instanceof ShooterIOSim sim) {
            sim.simulate(0.02);
        }

        io.setTargetRPS(targetMotorRPS);

        // -----------------------------
        // Shot sensor edge detection
        // -----------------------------
        boolean current = inputs.ballFired;
        boolean fallingEdge = lastBallSensor && !current;

        double now = Timer.getFPGATimestamp();

        boolean debounced = (now - lastShotTime) > 0.1;

        if (shotTrainer != null && fallingEdge && debounced) {
            lastShotTime = now;
            shotTrainer.notifyShotFired(Timer.getFPGATimestamp());
        }

        lastBallSensor = current;

        if (shotTrainer !=null){
            shotTrainer.update();
        }
        // -----------------------------
        // Telemetry logging
        // -----------------------------
        telemetry.logShooter(
            shooterKey,
            getShooterRPM(),
            getTargetRPM(),
            inputs.appliedVolts,
            inputs.statorCurrent,
            inputs.supplyCurrent,
            getClosedLoopError(),
            getShooterPose(),
            getShooterPose().getTranslation().getDistance(targetPose.getTranslation())
        );
    }

    // ------------------------------------------------------------
    // Internal helper
    // ------------------------------------------------------------

    /** Computes shooter pose relative to robot pose using a fixed transform. */
    private static class ShooterKinematics {
        private final Transform2d offset;

        ShooterKinematics(Transform2d offset) {
            this.offset = offset;
        }

        Pose2d shooterPose(Pose2d robotPose) {
            return robotPose.transformBy(offset);
        }
    }
}
