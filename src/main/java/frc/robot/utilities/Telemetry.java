package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {

    private final double maxSpeed;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* ---------------- Drive State ---------------- */

    private final NetworkTable driveStateTable = inst.getTable("DriveState");

    private final StructPublisher<Pose2d> drivePose =
        driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

    private final StructPublisher<ChassisSpeeds> driveSpeeds =
        driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> driveModuleStates =
        driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
        driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
        driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

    private final DoublePublisher driveTimestamp =
        driveStateTable.getDoubleTopic("Timestamp").publish();

    private final DoublePublisher driveOdometryFrequency =
        driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* ---------------- Field Pose ---------------- */

    private final NetworkTable poseTable = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub =
        poseTable.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub =
        poseTable.getStringTopic(".type").publish();

    /* ---------------- Vision Pose Publishers ---------------- */

    private final Map<String, StructPublisher<Pose2d>> visionPosePublishers = new HashMap<>();

    /* ---------------- Photon Diagnostics ---------------- */

    private final NetworkTable visionTable = inst.getTable("Vision");

    private static class PhotonDiagPublishers {
        final BooleanPublisher hasTarget;
        final DoublePublisher tagCount;
        final DoublePublisher bestAmbiguity;
        final DoublePublisher bestTagId;
        final BooleanPublisher poseAccepted;

        PhotonDiagPublishers(NetworkTable table) {
            hasTarget = table.getBooleanTopic("HasTarget").publish();
            tagCount = table.getDoubleTopic("TagCount").publish();
            bestAmbiguity = table.getDoubleTopic("BestAmbiguity").publish();
            bestTagId = table.getDoubleTopic("BestTagID").publish();
            poseAccepted = table.getBooleanTopic("PoseAccepted").publish();
        }
    }

    private final Map<String, PhotonDiagPublishers> photonDiagPublishers = new HashMap<>();

    /* ---------------- Construction ---------------- */

    public Telemetry(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        SignalLogger.start();
    }

    /* ---------------- Vision Pose API ---------------- */

    /** Register a vision pose source (call once during robot init). */
    public void registerVisionPoseSource(String sourceKey) {
        visionPosePublishers.put(
            sourceKey,
            driveStateTable
                .getStructTopic(sourceKey + "Pose", Pose2d.struct)
                .publish()
        );
    }

    /** Publish a vision pose for a registered source. */
    public void publishVisionPose(String sourceKey, Pose2d pose) {
        StructPublisher<Pose2d> pub = visionPosePublishers.get(sourceKey);
        if (pub != null) {
            pub.set(pose);
        }
    }

    /* ---------------- Photon Diagnostics API ---------------- */

    public void publishPhotonCameraDiagnostics(
        String cameraName,
        boolean hasTarget,
        int tagCount,
        double bestAmbiguity,
        int bestTagId,
        boolean poseAccepted
    ) {
        PhotonDiagPublishers pubs =
            photonDiagPublishers.computeIfAbsent(cameraName, name -> {
                NetworkTable camTable =
                    visionTable.getSubTable("Photon").getSubTable(name);
                return new PhotonDiagPublishers(camTable);
            });

        pubs.hasTarget.set(hasTarget);
        pubs.tagCount.set(tagCount);
        pubs.bestAmbiguity.set(bestAmbiguity);
        pubs.bestTagId.set(bestTagId);
        pubs.poseAccepted.set(poseAccepted);
    }

    /* ---------------- Drive Telemetry ---------------- */

    private final double[] poseArray = new double[3];
    private final double[] moduleStatesArray = new double[8];
    private final double[] moduleTargetsArray = new double[8];

    public void telemeterize(SwerveDriveState state) {

        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        poseArray[0] = state.Pose.getX();
        poseArray[1] = state.Pose.getY();
        poseArray[2] = state.Pose.getRotation().getDegrees();

        for (int i = 0; i < 4; i++) {
            moduleStatesArray[i * 2] = state.ModuleStates[i].angle.getRadians();
            moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            moduleTargetsArray[i * 2] = state.ModuleTargets[i].angle.getRadians();
            moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        fieldTypePub.set("Field2d");
        fieldPub.set(poseArray);

        updateMechanisms(state);
    }

    /* ---------------- Mechanisms ---------------- */

    private final Mechanism2d[] moduleMechanisms = {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1)
    };

    private final MechanismLigament2d[] moduleSpeeds = {
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0))
    };

    private final MechanismLigament2d[] moduleDirections = {
        moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite)))
    };

    private void updateMechanisms(SwerveDriveState state) {
        for (int i = 0; i < 4; i++) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(
                state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed)
            );
            SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
        }
    }

    /* ---------------- Field Overlay ---------------- */

    private final Field2d fieldOverlay = new Field2d();

    public void publishPolygon(String name, java.util.List<Translation2d> polygon) {
        for (int i = 0; i < polygon.size(); i++) {
            Translation2d pt = polygon.get(i);
            fieldOverlay.getObject(name + "_pt" + i)
                .setPose(new Pose2d(pt, new Rotation2d()));
        }
        SmartDashboard.putData("FieldOverlay", fieldOverlay);
    }
}
