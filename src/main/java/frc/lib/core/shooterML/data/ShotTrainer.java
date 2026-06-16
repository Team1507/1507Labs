//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.shooterML.data;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.core.shooterML.ShooterTelemetryProvider;

/**
 * ShotTrainer collects unlabeled shot data for offline model training.
 *
 * This version is IO‑friendly and does not depend on hardware or subsystems.
 * All shooter telemetry is pulled through ShooterTelemetryProvider.
 */
public class ShotTrainer {

    private final ShooterTelemetryProvider shooter;
    private final Supplier<Pose2d> poseSupplier;
    private Translation2d targetPose;

    private int nextShotId = 0;

    private final List<ShotRecord> pendingShots = new ArrayList<>();

    private final NetworkTable rootTable =
        NetworkTableInstance.getDefault()
            .getTable("Shooter ML")
            .getSubTable("UnlabeledShots");

    public ShotTrainer(
        ShooterTelemetryProvider shooter,
        Supplier<Pose2d> poseSupplier,
        Translation2d targetPose
    ) {
        this.shooter = shooter;
        this.poseSupplier = poseSupplier;
        this.targetPose = targetPose;
    }

    /** Called by ShooterSubsystem when the shot sensor detects a fired ball. */
    public void notifyShotFired(double timestampSeconds) {
        recordShot(timestampSeconds);
    }

    private void recordShot(double timestampSeconds) {

        Pose2d pose = poseSupplier.get();
        double distance = pose.getTranslation().getDistance(targetPose);

        ShotRecord record = new ShotRecord(
            shooter.getShooterRPM(),
            shooter.getShooterVoltage(),
            shooter.getStatorCurrent(),
            shooter.getSupplyCurrent(),
            shooter.getClosedLoopError(),
            pose,
            distance
        );

        record.id = nextShotId++;
        pendingShots.add(record);

        publishShot(record);
    }

    private void publishShot(ShotRecord r) {

        NetworkTable t = rootTable.getSubTable(Integer.toString(r.id));

        t.getEntry("id").setDouble(r.id);
        t.getEntry("distance").setDouble(r.distanceToTarget);
        t.getEntry("rpm").setDouble(r.shooterRPM);
        t.getEntry("stator").setDouble(r.statorCurrent);
        t.getEntry("supply").setDouble(r.supplyCurrent);
        t.getEntry("error").setDouble(r.closedLoopError);
        t.getEntry("label").setString("unlabeled");
    }

    public void update() {
        checkForLabels();
    }

    private void checkForLabels() {

        Iterator<ShotRecord> it = pendingShots.iterator();

        while (it.hasNext()) {
            ShotRecord r = it.next();

            NetworkTable t = rootTable.getSubTable(Integer.toString(r.id));
            String label = t.getEntry("label").getString("unlabeled");

            if (label == null || label.isBlank() ||
                label.equals("unlabeled") || label.equals("processed")) {
                continue;
            }

            ShotLabel parsed = ShotLabelParser.parse(label);
            r.setLabel(parsed.made());

            CSVWriter.writeSingleRecord(r);

            t.getEntry("label").setString("processed");

            for (String key : t.getKeys()) {
                t.getEntry(key).unpublish();
            }

            rootTable.getEntry(Integer.toString(r.id)).unpublish();

            it.remove();
        }
    }
}
