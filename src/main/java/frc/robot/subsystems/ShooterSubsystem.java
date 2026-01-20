package frc.robot.subsystems;

// CTRE Libraries
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

// WPI Libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Mechanics
import frc.robot.mechanics.GearRatio;
import frc.robot.mechanics.FlywheelModel;

// Shooter Model
import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotRecord;
import frc.robot.shooter.model.ShooterModel;

// Constants
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.Gains;

/**
 * ShooterSubsystem provides a unified interface for controlling a flywheel‑based shooter.
 * <p>
 * This implementation supports four capability tiers:
 * <ul>
 *     <li><b>Basic shooter</b> — closed‑loop velocity control only</li>
 *     <li><b>Model‑driven shooter</b> — uses a ShooterModel to compute RPM from robot pose</li>
 *     <li><b>GearRatio‑aware shooter</b> — correct motor↔wheel conversion</li>
 *     <li><b>Full physics simulation</b> — FlywheelModel for realistic acceleration and steady‑state behavior</li>
 * </ul>
 *
 * All constructors chain into the full constructor, ensuring consistent initialization.
 * Simulation behavior is automatically enabled when running in WPILib simulation mode.
 */
public class ShooterSubsystem extends SubsystemBase {

    // ------------------------------------------------------------
    // Default objects for optional constructors
    // ------------------------------------------------------------

    /** Default 1:1 gearbox for basic shooters. */
    private static final GearRatio DEFAULT_RATIO = GearRatio.gearBox(1, 1);

    /** Default flywheel model used when none is provided. */
    private static final FlywheelModel DEFAULT_FLYWHEEL =
        new FlywheelModel(
            /* inertia */ 0.0001,
            /* motorKv */ 533,
            /* motorKt */ 0.018,
            /* resistance */ 0.09,
            /* frictionTorque */ 0.0,
            DEFAULT_RATIO
        );

    /** Default model-driven shooter disabled by default. */
    private static final ShooterModel DEFAULT_MODEL = null;

    /** Default pose supplier returning origin. */
    private static final PoseSupplier DEFAULT_POSE_SUPPLIER = () -> new Pose2d();

    /** Default target pose for model-driven shooters. */
    private static final Pose2d DEFAULT_TARGET_POSE = new Pose2d();

    /** Default shooter pose for model-driven shooters. */
    private static final Transform2d DEFAULT_SHOOTER_OFFSET = new Transform2d(
            new Translation2d(0.0, 0.0), // X=2m, Y=0m
            Rotation2d.fromDegrees(0)
        );

    // ------------------------------------------------------------
    // Hardware
    // ------------------------------------------------------------

    private final TalonFX shooterMotor;
    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0).withSlot(0);

    // ------------------------------------------------------------
    // Mechanics
    // ------------------------------------------------------------

    private final GearRatio ratio;
    private final FlywheelModel flywheel;

    // ------------------------------------------------------------
    // Model-driven shooter fields
    // ------------------------------------------------------------

    private final ShooterModel model;
    private final PoseSupplier poseSupplier;
    private Pose2d targetPose;

    /** Phoenix 6 uses motor RPS internally. */
    private double targetMotorRPS = 0.0;

    /** Used to know where the robot is located relative to robot center */
    private final Transform2d shooterOffset;

    // ------------------------------------------------------------
    // Simulation state
    // ------------------------------------------------------------

    private double simWheelRPM = 0.0;
    private double simVoltage = 0.0;
    private double simMotorRpsMeasured = 0.0;
    private double simMotorRpsCommanded = 0.0;

    // ------------------------------------------------------------
    // Constructor Tier 1 — Basic shooter
    // ------------------------------------------------------------

    /**
     * Creates a basic shooter subsystem using default GearRatio, FlywheelModel,
     * and no model-driven behavior.
     *
     * @param shooterMotor the TalonFX controlling the shooter
     */
    public ShooterSubsystem(TalonFX shooterMotor) {
        this(
            shooterMotor,
            DEFAULT_RATIO,
            DEFAULT_FLYWHEEL,
            DEFAULT_MODEL,
            DEFAULT_POSE_SUPPLIER,
            DEFAULT_TARGET_POSE,
            DEFAULT_SHOOTER_OFFSET
        );
    }

    // ------------------------------------------------------------
    // Constructor Tier 2 — Model-driven shooter
    // ------------------------------------------------------------

    /**
     * Creates a shooter subsystem with model-driven RPM prediction.
     *
     * @param shooterMotor the TalonFX controlling the shooter
     * @param model        shooter model used to compute RPM from telemetry
     * @param poseSupplier supplier for robot pose
     * @param targetPose   target pose used by the shooter model
     */
    public ShooterSubsystem(
        TalonFX shooterMotor,
        ShooterModel model,
        PoseSupplier poseSupplier,
        Pose2d targetPose,
        Transform2d shooterOffset
    ) {
        this(
            shooterMotor,
            DEFAULT_RATIO,
            DEFAULT_FLYWHEEL,
            model,
            poseSupplier,
            targetPose,
            shooterOffset
        );
    }

    // ------------------------------------------------------------
    // Constructor Tier 3 — GearRatio + Model-driven shooter
    // ------------------------------------------------------------

    /**
     * Creates a shooter subsystem with a custom GearRatio and model-driven behavior.
     *
     * @param shooterMotor the TalonFX controlling the shooter
     * @param ratio        gearbox ratio for motor↔wheel conversion
     * @param model        shooter model used to compute RPM
     * @param poseSupplier supplier for robot pose
     * @param targetPose   target pose used by the shooter model
     */
    public ShooterSubsystem(
        TalonFX shooterMotor,
        GearRatio ratio,
        ShooterModel model,
        PoseSupplier poseSupplier,
        Pose2d targetPose,
        Transform2d shooterOffset
    ) {
        this(
            shooterMotor,
            ratio,
            DEFAULT_FLYWHEEL,
            model,
            poseSupplier,
            targetPose,
            shooterOffset
        );
    }

    // ------------------------------------------------------------
    // Constructor Tier 4 — Full physics simulation
    // ------------------------------------------------------------

    /**
     * Creates a fully featured shooter subsystem with:
     * <ul>
     *     <li>GearRatio for motor↔wheel conversion</li>
     *     <li>FlywheelModel for physics-based simulation</li>
     *     <li>ShooterModel for model-driven RPM prediction</li>
     *     <li>PoseSupplier for dynamic targeting</li>
     * </ul>
     *
     * @param shooterMotor the TalonFX controlling the shooter
     * @param ratio        gearbox ratio for motor↔wheel conversion
     * @param flywheel     physics model for simulation
     * @param model        shooter model (may be null)
     * @param poseSupplier supplier for robot pose
     * @param targetPose   target pose used by the shooter model
     */
    public ShooterSubsystem(
        TalonFX shooterMotor,
        GearRatio ratio,
        FlywheelModel flywheel,
        ShooterModel model,
        PoseSupplier poseSupplier,
        Pose2d targetPose,
        Transform2d shooterOffset
    ) {
        this.shooterMotor = shooterMotor;
        this.ratio = ratio;
        this.flywheel = flywheel;
        this.model = model;
        this.poseSupplier = poseSupplier;
        this.targetPose = targetPose;
        this.shooterOffset = shooterOffset;

        configurePID();
    }

    // ------------------------------------------------------------
    // PID Configuration
    // ------------------------------------------------------------

    /**
     * Applies PID and feedforward gains from {@link Shooter.Gains}
     * to the TalonFX Slot0 configuration.
     */
    private void configurePID() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = Gains.KP;
        cfg.Slot0.kI = Gains.KI;
        cfg.Slot0.kD = Gains.KD;

        cfg.Slot0.kV = Gains.KV;
        cfg.Slot0.kS = Gains.KS;
        cfg.Slot0.kA = Gains.KA;

        shooterMotor.getConfigurator().apply(cfg);
    }

    /**
     * @return the underlying TalonFX motor controller
     */
    public TalonFX getShooterMotor() {
        return shooterMotor;
    }

    // ------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------

    /**
     * Returns the current wheel RPM.
     * <p>
     * In simulation, this returns the simulated wheel RPM.
     * On real hardware, this converts motor RPS → wheel RPM using the GearRatio.
     *
     * @return current wheel RPM
     */
    public double getShooterRPM() {
        if (RobotBase.isSimulation()) return simWheelRPM;

        double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
        return ratio.toOutput(motorRPS) * 60.0;
    }

    /**
     * @return applied voltage (real or simulated)
     */
    public double getShooterVoltage() {
        return RobotBase.isSimulation()
            ? simVoltage
            : shooterMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * @return stator current (real or simulated)
     */
    public double getStatorCurrent() {
        return RobotBase.isSimulation()
            ? 5.0 + Math.abs(simWheelRPM) / 1000.0
            : shooterMotor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * @return supply current (real or simulated)
     */
    public double getSupplyCurrent() {
        return RobotBase.isSimulation()
            ? getStatorCurrent()
            : shooterMotor.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * @return closed-loop error in wheel RPM
     */
    public double getClosedLoopError() {
        return getTargetRPM() - getShooterRPM();
    }

    /**
     * @return pose of the shooter
     */
    public Pose2d getShooterPose() {
        return poseSupplier.getPose().transformBy(shooterOffset);
    }

    // ------------------------------------------------------------
    // Model-driven shooter update
    // ------------------------------------------------------------

    /**
     * Builds a {@link ShotRecord} containing telemetry used by the ShooterModel.
     *
     * @return telemetry snapshot
     */
    private ShotRecord buildTelemetry() {
        Pose2d robotPose = poseSupplier.getPose();
        Pose2d shooterPose = robotPose.transformBy(shooterOffset);

        double distance = shooterPose.getTranslation().getDistance(targetPose.getTranslation());

        return new ShotRecord(
            getShooterRPM(),
            getShooterVoltage(),
            getStatorCurrent(),
            getSupplyCurrent(),
            getClosedLoopError(),
            shooterPose,
            distance
        );
    }

    /**
     * Updates the shooter target RPM using the ShooterModel.
     * <p>
     * If no model is provided, this method does nothing.
     */
    public void updateShooterFromModel() {
        if (model == null) return;

        ShotRecord telemetry = buildTelemetry();
        double rpm = model.getRPM(telemetry);
        setTargetRPM(rpm);
    }

    // ------------------------------------------------------------
    // Shooter control API
    // ------------------------------------------------------------

    /**
     * Sets the desired wheel RPM for the shooter.
     * <p>
     * Internally converts wheel RPM → motor RPS using the GearRatio.
     *
     * @param wheelRPM desired wheel RPM
     */
    public void setTargetRPM(double wheelRPM) {
        double wheelRPS = wheelRPM / 60.0;
        targetMotorRPS = ratio.toMotor(wheelRPS);
    }

    /**
     * @return current target wheel RPM
     */
    public double getTargetRPM() {
        return ratio.toOutput(targetMotorRPS) * 60.0;
    }

    /**
     * Sets the target pose used by the ShooterModel.
     *
     * @param newTarget new target pose
     */
    public void setTargetPose(Pose2d newTarget) {
        this.targetPose = newTarget;
    }

    // ------------------------------------------------------------
    // Simulation reset
    // ------------------------------------------------------------

    /**
     * Resets all simulation state variables to zero.
     * <p>
     * Called automatically by the PID tuner before each test run.
     */
    public void resetSimulationState() {
        simWheelRPM = 0;
        simMotorRpsMeasured = 0;
        simMotorRpsCommanded = 0;
        simVoltage = 0;
    }

    // ------------------------------------------------------------
    // Periodic
    // ------------------------------------------------------------

    /**
     * Periodic update loop.
     * <p>
     * On real hardware, this sends the VelocityVoltage command to the TalonFX.
     * In simulation, this executes a physics-based simulation loop:
     * <ol>
     *     <li>Convert wheel RPM → motor RPS</li>
     *     <li>Apply sensor filtering</li>
     *     <li>Apply command filtering</li>
     *     <li>Compute PID + FF voltage</li>
     *     <li>Apply voltage slew rate</li>
     *     <li>Step the FlywheelModel</li>
     * </ol>
     */
    @Override
    public void periodic() {

        if (RobotBase.isReal()) {
            shooterMotor.setControl(velocityRequest.withVelocity(targetMotorRPS));
            return;
        }

        // -----------------------------
        // Simulation loop (20ms)
        // -----------------------------
        double dt = 0.02;

        double wheelRPS = simWheelRPM / 60.0;
        double motorRPS = ratio.toMotor(wheelRPS);

        // 1. Sensor filtering
        double alphaSensor = dt / (Shooter.Sim.SENSOR_FILTER_TIME_CONSTANT + dt);
        simMotorRpsMeasured += alphaSensor * (motorRPS - simMotorRpsMeasured);

        // 2. Command filtering
        double alphaCommand = dt / (Shooter.Sim.COMMAND_FILTER_TIME_CONSTANT + dt);
        simMotorRpsCommanded += alphaCommand * (targetMotorRPS - simMotorRpsCommanded);

        // 3. Phoenix-like control law
        double errorRPS = simMotorRpsCommanded - simMotorRpsMeasured;

        double ffVolts = Gains.KV * simMotorRpsCommanded;
        double ksVolts = Gains.KS * Math.signum(simMotorRpsCommanded);
        double fbVolts = Gains.KP * errorRPS;

        double desiredVolts = ffVolts + ksVolts + fbVolts;

        // 4. Voltage slew rate limiting
        double maxStep = Shooter.Sim.VOLTAGE_SLEW_RATE * dt;
        double delta = desiredVolts - simVoltage;

        if (delta > maxStep) delta = maxStep;
        if (delta < -maxStep) delta = -maxStep;

        simVoltage += delta;

        // Clamp to battery
        simVoltage = Math.max(-Shooter.Sim.MAX_VOLTAGE,
                              Math.min(Shooter.Sim.MAX_VOLTAGE, simVoltage));

        // 5. Step flywheel physics
        if (flywheel != null)
            simWheelRPM = flywheel.stepRPM(simWheelRPM, simVoltage, dt);
    }
}
