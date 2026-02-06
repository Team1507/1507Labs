package frc.robot.subsystems.lib;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Telemetry;

/**
 * Base class for all vision systems (PhotonVisionManager, QuestNav, etc).
 *
 * Responsibilities:
 *  - Store the last accepted vision pose
 *  - Enforce pose freshness
 *  - Forward accepted measurements to the drivetrain
 *  - Publish pose telemetry
 *
 * This class does NOT:
 *  - Decide pose validity
 *  - Perform filtering
 *  - Know about cameras, tags, or diagnostics
 *
 * Subclasses are responsible for all decision‑making.
 */
public abstract class Vision1507 extends SubsystemBase {

    /** Most recent accepted vision pose (null if none). */
    protected Pose2d latestPose = null;

    /** FPGA timestamp (seconds) when the pose was last accepted. */
    private double lastPoseTimestamp = 0.0;

    /** Maximum age (seconds) before a pose is considered stale. */
    private static final double MAX_POSE_AGE_SEC = 0.25;

    /** Reference to the drivetrain for pose fusion and resets. */
    protected final CommandSwerveDrivetrain drivetrain;

    /** Telemetry logger (AdvantageScope / dashboards). */
    protected final Telemetry logger;

    protected Vision1507(CommandSwerveDrivetrain drivetrain, Telemetry logger) {
        this.drivetrain = drivetrain;
        this.logger = logger;
    }

    /**
     * Periodic update.
     *
     * Subclasses must:
     *  - update(): read sensors, filter, accept poses
     *  - addVisionMeasurementToDrivetrain(): forward accepted poses
     */
    @Override
    public final void periodic() {
        update();
        addVisionMeasurementToDrivetrain();
        publishTelemetry();
    }

    /** Read sensors and decide whether to accept a pose. */
    protected abstract void update();

    /** Push accepted vision measurements into the drivetrain estimator. */
    protected abstract void addVisionMeasurementToDrivetrain();

    /**
     * Accepts a vision pose as valid.
     *
     * Subclasses should ONLY call this after all filtering is complete.
     */
    protected final void acceptPose(Pose2d pose, double timestampSeconds) {
        latestPose = pose;
        lastPoseTimestamp = timestampSeconds;
    }

    /**
     * Invalidates the current pose.
     *
     * Does NOT overwrite the last known pose with garbage.
     */
    protected final void invalidatePose() {
        latestPose = null;
    }

    /**
     * Returns the most recent valid vision pose, if it exists and is fresh.
     */
    public final Optional<Pose2d> getLatestPose() {
        if (latestPose == null) {
            return Optional.empty();
        }

        double age = Timer.getFPGATimestamp() - lastPoseTimestamp;
        if (age > MAX_POSE_AGE_SEC) {
            return Optional.empty();
        }

        return Optional.of(latestPose);
    }

    /**
     * Returns true if the vision system is currently tracking a valid pose.
     */
    public final boolean isTracking() {
        return getLatestPose().isPresent();
    }

    /**
     * Returns the timestamp of the last accepted pose.
     */
    public final double getLastPoseTimestamp() {
        return lastPoseTimestamp;
    }

    /**
     * Returns the last accepted pose, even if stale.
     *
     * Useful for debugging and telemetry.
     */
    public final Optional<Pose2d> getLastAcceptedPose() {
        return Optional.ofNullable(latestPose);
    }

    /**
     * Returns the last known vision pose, regardless of age.
     *
     * Intended ONLY for manual hard resets.
     */
    public final Optional<Pose2d> getLastKnownVisionPose() {
        return lastPoseTimestamp > 0
            ? Optional.ofNullable(latestPose)
            : Optional.empty();
    }

    /**
     * Hard‑resets the drivetrain pose to the last known vision pose.
     *
     * This should ONLY be called explicitly (button press, debug command).
     */
    public final void resetDrivetrainToVisionPose() {
        getLastKnownVisionPose().ifPresent(pose -> {
            drivetrain.resetPose(pose);
            System.out.println(
                "[Vision1507] HARD reset drivetrain pose from " +
                getTelemetrySource() + ": " + pose
            );
        });
    }

    /**
     * Returns the telemetry source key used when publishing poses.
     *
     * Subclasses should override this to provide a stable name
     * (e.g. "PhotonVisionManager", "QuestNav").
     */
    protected String getTelemetrySource() {
        return getClass().getSimpleName();
    }

    /**
     * Publishes pose telemetry.
     *
     * Called automatically once per periodic cycle.
     */
    protected void publishTelemetry() {
        if (logger != null && latestPose != null) {
            logger.publishVisionPose(getTelemetrySource(), latestPose);
        }
    }
}
