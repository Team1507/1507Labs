package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CmdMaintainHeadingToTarget extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;

    private final PIDController headingController =
        new PIDController(4.0, 0.0, 0.1);

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric();

    public CmdMaintainHeadingToTarget(
        CommandSwerveDrivetrain drivetrain,
        Supplier<Pose2d> targetPoseSupplier,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier
    ) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    private Rotation2d computeHeadingToTarget(Pose2d robot, Pose2d target) {
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getState().Pose;
        Pose2d targetPose = targetPoseSupplier.get();

        Rotation2d targetHeading = computeHeadingToTarget(robotPose, targetPose);

        double rotRate = headingController.calculate(
            drivetrain.getHeading().getRadians(),
            targetHeading.getRadians()
        );

        drivetrain.setControl(
            driveRequest
                .withVelocityX(xSupplier.get())
                .withVelocityY(ySupplier.get())
                .withRotationalRate(rotRate)
        );
    }
}
