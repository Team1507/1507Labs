package frc.robot.subsystems;

import static frc.robot.Constants.kVision.APRILTAG_LAYOUT;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants.kVision;
import frc.robot.subsystems.lib.Vision1507;
import frc.robot.utilities.Telemetry;

public class PhotonVisionManagerSubsystem extends Vision1507 {

    public record CameraConfig(String name, Transform3d robotToCamera) {}

    public static record VisionMeasurement(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs,
        double xyStd
    ) {}

    // -------------------- Tunables --------------------

    private static final double VISION_PERIOD = 0.05;

    private static final double TRIG_XY_STD_COEFF = 0.18;
    private static final double CONSTR_XY_STD_COEFF = 0.40;
    private static final double CONSTR_ANG_STD_COEFF = 0.14;
    private static final double TRIG_ANG_STD = 1e5;

    // -------------------- State --------------------

    private final CameraUnit[] cameras;

    /** Only the measurements we actually want to fuse this cycle. */
    private List<VisionMeasurement> pendingMeasurements = List.of();

    private double lastVisionTime = 0.0;

    // -------------------- Construction --------------------

    public PhotonVisionManagerSubsystem(
        CommandSwerveDrivetrain drivetrain,
        Telemetry logger,
        CameraConfig... cameraConfigs
    ) {
        super(drivetrain, logger);
        cameras = new CameraUnit[cameraConfigs.length];
        for (int i = 0; i < cameraConfigs.length; i++) {
            cameras[i] = new CameraUnit(cameraConfigs[i]);
        }
    }

    @Override
    protected String getTelemetrySource() {
        return "PhotonVisionManager";
    }

    public void resetVisionSeeding() {
        pendingMeasurements = List.of();
        for (var cam : cameras) cam.resetEstimatorCaches();
    }

    // -------------------- Periodic --------------------

    @Override
    protected void update() {
        double now = Timer.getFPGATimestamp();
        if (now - lastVisionTime < VISION_PERIOD) return;
        lastVisionTime = now;

        boolean enabled = DriverStation.isEnabled();
        boolean usingTrig = enabled; // keep your current behavior: trig when enabled

        List<VisionMeasurement> measurements = new ArrayList<>();

        for (CameraUnit cam : cameras) {
            cam.setStrategy(usingTrig);
            cam.consumeAllUnreadResults(usingTrig, measurements);
        }

        // Pick ONE measurement to fuse (prevents tug-of-war).
        VisionMeasurement best = selectBest(measurements);
        if (best != null) {
            acceptPose(best.pose(), best.timestampSeconds());
            pendingMeasurements = List.of(best);
        } else {
            invalidatePose();
            pendingMeasurements = List.of();
        }

        for (CameraUnit cam : cameras) {
            cam.publishDiagnostics();
        }
    }

    @Override
    protected void addVisionMeasurementToDrivetrain() {
        for (var m : pendingMeasurements) {
            drivetrain.addVisionMeasurement(
                m.pose(),
                m.timestampSeconds(),
                m.stdDevs()
            );
        }
    }

    // -------------------- Camera Unit --------------------

    private class CameraUnit {

        private final Transform3d robotToCamera;
        private final PhotonCamera camera;
        private final String telemetryKey;
        private PhotonPoseEstimator estimator;

        private double lastProcessedResultTimestamp = -1.0;
        private double lastSeededTimestamp = -1.0;

        private boolean hasTarget = false;
        private int tagCount = 0;
        private double bestAmbiguity = -1.0;
        private int bestTagId = -1;
        private boolean poseAccepted = false;

        CameraUnit(CameraConfig config) {
            robotToCamera = config.robotToCamera();
            camera = new PhotonCamera(config.name());

            telemetryKey = switch (config.name()) {
                case kVision.BLU.NAME -> "Photon-BLU";
                case kVision.YEL.NAME -> "Photon-YEL";
                default -> "Photon-" + config.name();
            };

            rebuildEstimator();
        }

        void rebuildEstimator() {
            estimator = new PhotonPoseEstimator(
                APRILTAG_LAYOUT,
                PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                robotToCamera
            );
            resetEstimatorCaches();
        }

        void resetEstimatorCaches() {
            lastProcessedResultTimestamp = -1.0;
            lastSeededTimestamp = -1.0;
        }

        void setStrategy(boolean usingTrig) {
            estimator.setPrimaryStrategy(
                usingTrig
                    ? PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
                    : PoseStrategy.CONSTRAINED_SOLVEPNP
            );
        }

        void seedHeadingFromGyro(double timestampSeconds) {
            // Only seed once per camera frame timestamp.
            if (timestampSeconds > lastSeededTimestamp) {
                estimator.addHeadingData(
                    timestampSeconds,
                    drivetrain.getPigeon2().getRotation2d()
                );
                lastSeededTimestamp = timestampSeconds;
            }
        }

        void consumeAllUnreadResults(boolean usingTrig, List<VisionMeasurement> out) {
            poseAccepted = false;

            List<PhotonPipelineResult> unread = camera.getAllUnreadResults();
            if (unread.isEmpty()) return;

            for (PhotonPipelineResult result : unread) {
                double ts = result.getTimestampSeconds();
                if (ts == lastProcessedResultTimestamp) continue;
                lastProcessedResultTimestamp = ts;

                seedHeadingFromGyro(ts);

                hasTarget = result.hasTargets();
                tagCount = result.getTargets().size();

                if (hasTarget && result.getBestTarget() != null) {
                    bestAmbiguity = result.getBestTarget().getPoseAmbiguity();
                    bestTagId = result.getBestTarget().getFiducialId();
                } else {
                    bestAmbiguity = -1.0;
                    bestTagId = -1;
                }

                if (!hasTarget) continue;

                // Strong anti-flip rule: require >=2 tags.
                if (tagCount < 2) continue;

                var estimate = estimator.update(result);
                if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) continue;

                // Alliance tag filtering (Team 340-style): never allow “other side” tags to participate.
                boolean allTagsValid = true;
                for (var t : estimate.get().targetsUsed) {
                    if (!useTagForAlliance(t.fiducialId)) {
                        allTagsValid = false;
                        break;
                    }
                }
                if (!allTagsValid) continue;

                Pose2d visionPose = estimate.get().estimatedPose.toPose2d();

                // Optional sanity gate (keep your intent, but only when enabled).
                Pose2d odomPose = drivetrain.getState().Pose;
                // if (DriverStation.isEnabled()
                //     && visionPose.getTranslation().getDistance(odomPose.getTranslation()) > 1.0) {
                //     continue;
                // }

                double distance =
                    estimate.get().targetsUsed.get(0)
                        .bestCameraToTarget.getTranslation().getNorm();

                if (distance > 3.0) continue;

                double xyStd =
                    (usingTrig ? TRIG_XY_STD_COEFF : CONSTR_XY_STD_COEFF)
                        * distance * distance;

                double angStd =
                    usingTrig
                        ? TRIG_ANG_STD
                        : CONSTR_ANG_STD_COEFF * distance * distance;

                Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, angStd);

                out.add(new VisionMeasurement(
                    visionPose,
                    estimate.get().timestampSeconds,
                    stdDevs,
                    xyStd
                ));

                poseAccepted = true;

                // Publish per-camera pose (diagnostic)
                logger.publishVisionPose(telemetryKey, visionPose);
            }
        }

        void publishDiagnostics() {
            logger.publishPhotonCameraDiagnostics(
                camera.getName(),
                hasTarget,
                tagCount,
                bestAmbiguity,
                bestTagId,
                poseAccepted
            );
        }
    }

    // -------------------- Helpers --------------------

    private static VisionMeasurement selectBest(List<VisionMeasurement> measurements) {
        VisionMeasurement best = null;
        for (var m : measurements) {
            if (best == null || m.xyStd() < best.xyStd()) best = m;
        }
        return best;
    }

    private static boolean useTagForAlliance(int id) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return true;

        // Mirror Team 340 logic: only accept “reef” tags for current alliance.
        // Adjust ID ranges if your 2026 field differs.
        if (alliance.get() == Alliance.Blue) {
            return (id >= 17 && id <= 22);
        } else {
            return (id >= 6 && id <= 11);
        }
    }
}
