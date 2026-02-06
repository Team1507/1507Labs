package frc.robot;

// CTRE libraries
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;

// WPI libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;

// Robot Commands
import frc.robot.commands.CmdMoveRRT;
import frc.robot.commands.CmdSetShooterRPM;
import frc.robot.commands.CmdShooterPIDTuner;

// Robot Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionManagerSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

// Robot Constants
import frc.robot.Constants.kVision;
import static frc.robot.Constants.kIO.*;
import static frc.robot.Constants.kSpeed.*;
import static frc.robot.Constants.kShooter.*;
import frc.robot.Constants.kShooter.Flywheel;

// Shooter model imports
import frc.robot.shooter.data.PoseSupplier;
import frc.robot.shooter.data.ShotTrainer;
import frc.robot.shooter.model.ModelLoader;
import frc.robot.shooter.model.ShooterModel;

// Nodes
import frc.robot.navigation.Nodes.Hub;
import frc.robot.navigation.Nodes.AllianceZoneBlue;
//import frc.robot.navigation.Nodes.AllianceZoneRed;

// Autos
import frc.robot.auto.routines.OnePieceAuto;

// Robot Extra
import frc.robot.utilities.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.mechanics.FlywheelModel;
import frc.robot.mechanics.GearRatio;
import frc.robot.sim.FuelSimulator;

public class RobotContainer {

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(getMaxSpeed());
    private final CommandXboxController joystick = new CommandXboxController(JOYSTICK_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // -----------------------------
    // Shooter + Model
    // -----------------------------

    private final GearRatio ratio = GearRatio.gearBox(2, 1);

    private final FlywheelModel flywheel = new FlywheelModel(
        Flywheel.INERTIA, 
        Flywheel.MOTOR_KV,
        Flywheel.MOTOR_KT,
        Flywheel.MOTOR_RESISTANCE,
        Flywheel.FRICTION_TORQUE,
        Flywheel.GEAR_BOX
    );
    
    private final PoseSupplier poseSupplier = () -> drivetrain.getState().Pose;

    // Load model.json from deploy directory
    private final ShooterModel shooterModelConfig =
        ModelLoader.load("model.json", poseSupplier);

    public final ShooterSubsystem shooterSubsystem =
        new ShooterSubsystem(
            new TalonFX(SHOOTER_CAN_ID),
            ratio,
            flywheel,
            shooterModelConfig,
            poseSupplier,
            Hub.CENTER, // default target
            SHOOTER_OFFSET
        );

    public final ShotTrainer shotTrainer =
        new ShotTrainer(
            shooterSubsystem.getShooterMotor(),
            poseSupplier,
            Hub.CENTER.getTranslation()
        );

    private double shooterRPM;

    // -----------------------------
    // Autos
    // -----------------------------
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    
    // -----------------------------
    // Cameras
    // -----------------------------
/* 
    public final PhotonVisionManagerSubsystem photonVision =
        new PhotonVisionManagerSubsystem(
            drivetrain,
            logger,
            new PhotonVisionManagerSubsystem.CameraConfig(kVision.BLU.NANME, kVision.BLU.ROBOT_TO_CAMERA),
            new PhotonVisionManagerSubsystem.CameraConfig(kVision.YEL.NANME, kVision.YEL.ROBOT_TO_CAMERA)
        );
 */

    public final Vision PVManager =
        new Vision(
            drivetrain::addVisionMeasurement,
            drivetrain::getHeading,          // Supplier<Rotation2d>
            drivetrain::seedPoseFromVision,
            new VisionIOPhotonVision(kVision.BLU.NAME, kVision.BLU.ROBOT_TO_CAMERA),
            new VisionIOPhotonVision(kVision.YEL.NAME, kVision.YEL.ROBOT_TO_CAMERA));

    /** Robot Constructor */
    public RobotContainer() { 
        configureTelemetry();
        configureDefaultCommands();
        configureDriverControls();
        configureVision();
        configureAutos();
        configureDashboard();
    }

    /**
     * Shooter default behavior: use the trained model.json
     */
    private void configureShooterDefault() {

        shooterSubsystem.setDefaultCommand(
            Commands.run(
                () -> {
                    // Build telemetry → ask model → set RPM
                    //shooterSubsystem.updateShooterFromModel();
                    shooterSubsystem.setTargetRPM(shooterRPM);
                },
                shooterSubsystem
            )
        );
    }

    private void configureDefaultCommands() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                double xInput = applyDeadband(-joystick.getLeftY(), 0.15);
                double yInput = applyDeadband(-joystick.getLeftX(), 0.15);
                double rotInput = applyDeadband(-joystick.getRightX(), 0.15);

                return drive
                    .withDeadband(getMaxSpeed() * 0.1)
                    .withRotationalDeadband(getMaxAngularSpeed() * 0.1)
                    .withVelocityX(xInput * getTranslationScale() * getMaxSpeed())
                    .withVelocityY(yInput * getTranslationScale() * getMaxSpeed())
                    .withRotationalRate(rotInput * getRotationScale() * getMaxAngularSpeed());
            })
        );

        shooterSubsystem.setDefaultCommand(
            Commands.run(
                () -> shooterSubsystem.setTargetRPM(shooterRPM),
                shooterSubsystem
            )
        );
    }

    private void configureDriverControls() {

        // SysId
        joystick.back().and(joystick.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        joystick.start().and(joystick.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Field-centric reset
        joystick.leftBumper()
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Motion commands
        joystick.a()
            .onTrue(new CmdMoveRRT(drivetrain, Hub.APPROACH_FRONT));

        joystick.b()
            .onTrue(Commands.runOnce(CommandScheduler.getInstance()::cancelAll));

        // Shooter targeting
        joystick.rightBumper()
            .onTrue(Commands.runOnce(() ->
                shooterSubsystem.setTargetPose(Hub.APPROACH_FRONT)));

        joystick.rightTrigger()
            .onTrue(Commands.runOnce(() ->
                shooterSubsystem.setTargetPose(AllianceZoneBlue.LEFT)));

        joystick.leftTrigger()
            .onTrue(Commands.runOnce(() ->
                shooterSubsystem.setTargetPose(AllianceZoneBlue.RIGHT)));

        // Simulation visualization
        joystick.rightBumper()
            .onTrue(new FuelSimulator(
                shooterSubsystem, poseSupplier, shotTrainer,
                Hub.APPROACH_FRONT.getTranslation()));

        joystick.rightTrigger()
            .onTrue(new FuelSimulator(
                shooterSubsystem, poseSupplier, shotTrainer,
                AllianceZoneBlue.LEFT.getTranslation()));

        joystick.leftTrigger()
            .onTrue(new FuelSimulator(
                shooterSubsystem, poseSupplier, shotTrainer,
                AllianceZoneBlue.RIGHT.getTranslation()));
    }

    private void configureVision() {
        // Nothing to bind yet — vision runs autonomously
        // Future: buttons for resetVisionSeeding(), debug toggles, etc.
    }

    private void configureTelemetry() {
        logger.registerVisionPoseSource("PhotonVisionManager");
        logger.registerVisionPoseSource("Photon-BLU");
        logger.registerVisionPoseSource("Photon-YEL");

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    private void configureAutos() {

        // Default auto
        autoChooser.setDefaultOption(
            "Auto Do Nothing",
            Commands.print("Doing nothing")
        );
    
        // Example autos using your new builder
        autoChooser.addOption(
            "One Piece Auto",
            OnePieceAuto.build(drivetrain)
        );
    
        // Publish to dashboard
        SmartDashboard.putData("Auto Mode", autoChooser);
    }    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }    

    private double applyDeadband(double value, double deadband) {
        return (Math.abs(value) < deadband) ? 0.0 : value;
    }

    private void configureDashboard() {

        // Commands / buttons
        SmartDashboard.putData(
            "Run Shooter PID Tuner",
            new CmdShooterPIDTuner(shooterSubsystem, MAX_RPM)
        );

        // Operator‑editable inputs (initial value only)
        SmartDashboard.putNumber("Shooter RPM", shooterRPM);

        // Choosers, static widgets, etc.
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public void updateDashboardInputs() {

        // Read operator input
        shooterRPM = SmartDashboard.getNumber("Shooter RPM", shooterRPM);

        // Publish live telemetry
        SmartDashboard.putNumber(
            "Pigeon heading",
            drivetrain.getPigeon2().getRotation2d().getDegrees()
        );
    }
}
