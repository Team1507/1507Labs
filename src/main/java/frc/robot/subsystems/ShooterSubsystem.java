package frc.robot.subsystems;

// CTRE Libraries
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

// WPI Libraries
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Mechanics
import frc.robot.mechanics.GearRatio;
import frc.robot.mechanics.FlywheelModel;

// Shooter Model
import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotRecord;
import frc.robot.shooter.model.ShooterModel;

// Constants
import static frc.robot.Constants.Shooter;

import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.Gains;

public class ShooterSubsystem extends SubsystemBase {

    // ------------------------------------------------------------ 
    // Hardware 
    // ------------------------------------------------------------ 
    private final TalonFX shooterMotor;
    private final VelocityVoltage velocityRequest = 
        new VelocityVoltage(0).withSlot(0);

    private final GearRatio ratio;
    private final FlywheelModel flywheel;

    // ------------------------------------------------------------ 
    // Model-driven shooter fields 
    // ------------------------------------------------------------ 
    private final ShooterModel model; 
    private final PoseSupplier poseSupplier; 
    private Pose2d targetPose; 
    
    // Phoenix 6 uses MOTOR RPS internally 
    private double targetMotorRPS = 0.0;

    // ------------------------------------------------------------ 
    // Simulation state 
    // ------------------------------------------------------------     
    // Wheel RPM state 
    private double simWheelRPM = 0.0; 
    
    // Voltage applied in sim 
    private double simVoltage = 0.0; 
    
    // Phoenix-style smoothing state 
    private double simMotorRpsMeasured = 0.0; 
    private double simMotorRpsCommanded = 0.0;

    public ShooterSubsystem(
        TalonFX shooterMotor,
        GearRatio ratio,
        FlywheelModel flywheel,
        ShooterModel model,
        PoseSupplier poseSupplier,
        Pose2d targetPose
    ) {
        this.shooterMotor = shooterMotor;
        this.ratio = ratio;
        this.flywheel = flywheel;
        this.model = model;
        this.poseSupplier = poseSupplier;
        this.targetPose = targetPose;

        configurePID();
    }

    // ------------------------------------------------------------
    // PID Configuration
    // ------------------------------------------------------------
    private void configurePID() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Slot0 PID values
        cfg.Slot0.kP = Gains.KP;
        cfg.Slot0.kI = Gains.KI;
        cfg.Slot0.kD = Gains.KD;

        // Slot0 Feedforward values
        cfg.Slot0.kV = Gains.KV;
        cfg.Slot0.kS = Gains.KS;
        cfg.Slot0.kA = Gains.KA;

        shooterMotor.getConfigurator().apply(cfg);
    }

    public TalonFX getShooterMotor() {
        return shooterMotor;
    }

    // ------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------
    public double getShooterRPM() {
        if (RobotBase.isSimulation()) return simWheelRPM;
        
        double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
        return ratio.toOutput(motorRPS) * 60.0;
    }
    
    public double getShooterVoltage() {
        return RobotBase.isSimulation() 
            ? simVoltage 
            : shooterMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return RobotBase.isSimulation() 
            ? 5.0 + Math.abs(simWheelRPM) / 1000.0 
            : shooterMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return RobotBase.isSimulation()
            ? getStatorCurrent()
            : shooterMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getClosedLoopError() {
        return getTargetRPM() - getShooterRPM();
    }

    // ------------------------------------------------------------
    // Model-driven shooter update
    // ------------------------------------------------------------

    private ShotRecord buildTelemetry() {
        Pose2d pose = poseSupplier.getPose();
        double distance = pose.getTranslation().getDistance(targetPose.getTranslation());

        return new ShotRecord(
            getShooterRPM(),
            getShooterVoltage(),
            getStatorCurrent(),
            getSupplyCurrent(),
            getClosedLoopError(),
            pose,
            distance
        );
    }

    public void updateShooterFromModel() {
        ShotRecord telemetry = buildTelemetry();
        double rpm = model.getRPM(telemetry);
        setTargetRPM(rpm);
    }
    
    // ------------------------------------------------------------
    // Shooter control API (wheel RPM)
    // ------------------------------------------------------------
    public void setTargetRPM(double wheelRPM) { 
        double wheelRPS = wheelRPM / 60.0;
        targetMotorRPS = ratio.toMotor(wheelRPS); // convert to motor RPS
    }

    public double getTargetRPM() {        
        return ratio.toOutput(targetMotorRPS) * 60.0;
    }

    public void setTargetPose(Pose2d newTarget) {
        this.targetPose = newTarget;
    }

    // ------------------------------------------------------------
    // Reset Simulation Values to 0
    // ------------------------------------------------------------
    public void resetSimulationState() {
        simWheelRPM = 0;
        simMotorRpsMeasured = 0;
        simMotorRpsCommanded = 0;
        simVoltage = 0;
    }

    // ------------------------------------------------------------
    // Periodic
    // ------------------------------------------------------------
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

        // Convert wheel RPM → motor RPS
        double wheelRPS = simWheelRPM / 60.0;
        double motorRPS = ratio.toMotor(wheelRPS);

        // -----------------------------
        // 1. Sensor filtering
        // -----------------------------
        double alphaSensor = dt / (Shooter.Sim.SENSOR_FILTER_TIME_CONSTANT + dt);
        simMotorRpsMeasured += alphaSensor * (motorRPS - simMotorRpsMeasured);

        // -----------------------------
        // 2. Command filtering
        // -----------------------------
        double alphaCommand = dt / (Shooter.Sim.COMMAND_FILTER_TIME_CONSTANT + dt);
        simMotorRpsCommanded += alphaCommand * (targetMotorRPS - simMotorRpsCommanded);

        // -----------------------------
        // 3. Phoenix-like control law
        // -----------------------------
        double errorRPS = simMotorRpsCommanded - simMotorRpsMeasured;

        double ffVolts = Shooter.Gains.KV * simMotorRpsCommanded;
        double ksVolts = Shooter.Gains.KS * Math.signum(simMotorRpsCommanded);
        double fbVolts = Shooter.Gains.KP * errorRPS;

        double desiredVolts = ffVolts + ksVolts + fbVolts;

        // -----------------------------
        // 4. Voltage slew rate limiting
        // -----------------------------
        double maxStep = Shooter.Sim.VOLTAGE_SLEW_RATE * dt;
        double delta = desiredVolts - simVoltage;

        if (delta > maxStep) delta = maxStep;
        if (delta < -maxStep) delta = -maxStep;

        simVoltage += delta;

        // Clamp to battery
        simVoltage = Math.max(-Shooter.Sim.MAX_VOLTAGE,
                              Math.min(Shooter.Sim.MAX_VOLTAGE, simVoltage));

        // -----------------------------
        // 5. Step flywheel physics
        // -----------------------------
        simWheelRPM = flywheel.stepRPM(simWheelRPM, simVoltage, dt);
    }
}
