//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.photonvision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.hardware.PhotonVisionHardware;
import frc.robot.localization.vision.PhotonPoseEstimator;

/**
 * Real PhotonVision implementation for a single Bluecam camera.
 */
public class PhotonVisionIOReal implements PhotonVisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final String cameraName;

    private final Supplier<Rotation2d> headingSupplier;

    public PhotonVisionIOReal(Supplier<Rotation2d> headingSupplier) {
        this.headingSupplier = headingSupplier;

        var cfg = PhotonVisionHardware.BLUECAM;

        this.cameraName = cfg.name;
        this.camera = new PhotonCamera(cfg.name);

        // Custom estimator (AprilTag layout + robotToCamera)
        this.estimator = new PhotonPoseEstimator(
            frc.lib.core.math.FieldInfo.aprilTags(),
            cfg.robotToCamera
        );
    }

    @Override
    public int getCameraCount() {
        return 1;
    }

    @Override
    public String[] getCameraNames() {
        return new String[] { cameraName };
    }

    @Override
    public PhotonPoseEstimator getEstimator(int index) {
        return estimator; // index always 0
    }

    @Override
    public void updateInputs(PhotonVisionInputs inputs) {

        double now = Timer.getFPGATimestamp();
        Rotation2d heading = headingSupplier.get();

        // Feed heading into estimator
        estimator.addHeadingData(now, heading);

        PhotonVisionInputs.CameraInputs out = inputs.camera;

        out.name = cameraName;
        out.connected = camera.isConnected();
        inputs.cameraConnected = out.connected;

        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            out.hasTarget = false;
            out.rawResult = null;
            return;
        }

        PhotonPipelineResult result = results.get(results.size() - 1);
        out.rawResult = result;
        out.hasTarget = result.hasTargets();
        out.timestamp = now;

        if (out.hasTarget) {
            out.yaw = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
            out.pitch = Rotation2d.fromDegrees(result.getBestTarget().getPitch());
        }
    }
}
