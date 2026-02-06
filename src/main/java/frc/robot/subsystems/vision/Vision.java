package frc.robot.subsystems.vision;

import static frc.robot.Constants.kVision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    /** Robot heading source (gyro). Vision never supplies rotation after startup. */
    private final Supplier<Rotation2d> headingSupplier;

    /** Startup pose seeder: resets drivetrain pose + gyro yaw. */
    private final Consumer<Pose2d> poseSeeder;

    private boolean startupPoseSeeded = false;

    // Camera indices
    private static final int CAM_BLU = 0;
    private static final int CAM_YEL = 1;

    private int activeCamera = CAM_YEL;
    private double lastYelSeenTime = Double.NEGATIVE_INFINITY;

    public Vision(
        VisionConsumer consumer,
        Supplier<Rotation2d> headingSupplier,
        Consumer<Pose2d> poseSeeder,
        VisionIO... io
    ) {
        this.consumer = consumer;
        this.io = io;
        this.headingSupplier = headingSupplier;
        this.poseSeeder = poseSeeder;

        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.disconnectedAlerts = new Alert[io.length];

        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
            disconnectedAlerts[i] =
                new Alert("Vision camera " + io[i].getName() + " is disconnected.", AlertType.kWarning);
        }
    }

    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {

        // Update camera inputs
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + io[i].getName(), inputs[i]);
        }

        Pose2d bluPose = null;
        Pose2d yelPose = null;
        double bluTimestamp = 0.0;
        double yelTimestamp = 0.0;

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Process each camera
        for (int cam = 0; cam < io.length; cam++) {

            disconnectedAlerts[cam].set(!inputs[cam].connected);

            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            for (int tagId : inputs[cam].tagIds) {
                APRILTAG_LAYOUT.getTagPose(tagId).ifPresent(tagPoses::add);
            }

            for (var obs : inputs[cam].poseObservations) {

                boolean reject =
                    obs.tagCount() == 0 ||
                    (obs.tagCount() == 1 && obs.ambiguity() > maxAmbiguity) ||
                    Math.abs(obs.pose().getZ()) > maxZError ||
                    obs.pose().getX() < 0.0 ||
                    obs.pose().getX() > APRILTAG_LAYOUT.getFieldLength() ||
                    obs.pose().getY() < 0.0 ||
                    obs.pose().getY() > APRILTAG_LAYOUT.getFieldWidth();

                robotPoses.add(obs.pose());

                if (reject) {
                    robotPosesRejected.add(obs.pose());
                    continue;
                }

                robotPosesAccepted.add(obs.pose());

                // Build translation-only pose using gyro heading
                Pose2d xyOnly = new Pose2d(
                    obs.pose().toPose2d().getTranslation(),
                    headingSupplier.get()
                );

                if (cam == CAM_BLU) {
                    bluPose = xyOnly;
                    bluTimestamp = obs.timestamp();
                } else if (cam == CAM_YEL) {
                    yelPose = xyOnly;
                    yelTimestamp = obs.timestamp();
                }
            }

            Logger.recordOutput("Vision/" + io[cam].getName() + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/" + io[cam].getName() + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/" + io[cam].getName() + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/" + io[cam].getName() + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        double now = Timer.getFPGATimestamp();

        // Track YEL availability
        if (yelPose != null) {
            lastYelSeenTime = now;
        }

        // If YEL missing but BLU present, use BLU
        if (yelPose == null && bluPose != null) {
            activeCamera = CAM_BLU;
        }

        // If YEL present, prefer YEL
        if (yelPose != null) {
            activeCamera = CAM_YEL;
        }

        // Select final pose
        Pose2d finalPose = null;
        double finalTimestamp = 0.0;

        if (activeCamera == CAM_YEL && yelPose != null) {
            finalPose = yelPose;
            finalTimestamp = yelTimestamp;
        } else if (activeCamera == CAM_BLU && bluPose != null) {
            finalPose = bluPose;
            finalTimestamp = bluTimestamp;
        }

        // If no pose, nothing to do
        if (finalPose == null) {
            return;
        }

        // --- STARTUP POSE SEEDING ---
        if (!startupPoseSeeded) {

            // PhotonVision 2026.2.1 changed coordinate frame → apply -90° correction
            Rotation2d correctedHeading =
                finalPose.getRotation().minus(Rotation2d.fromDegrees(0));

            Pose2d correctedPose = new Pose2d(
                finalPose.getTranslation(),
                correctedHeading
            );

            poseSeeder.accept(correctedPose);
            startupPoseSeeded = true;

            Logger.recordOutput("Vision/StartupPoseSeeded", correctedPose);
            return; // Do NOT fuse this frame
        }

        // --- NORMAL VISION FUSION (translation only) ---
        consumer.accept(
            finalPose,
            finalTimestamp,
            VecBuilder.fill(0.8, 0.8, Double.POSITIVE_INFINITY)
        );

        Logger.recordOutput("Vision/FusedPose", finalPose);
        Logger.recordOutput("Vision/ActiveCamera", activeCamera);
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
        );
    }
}
