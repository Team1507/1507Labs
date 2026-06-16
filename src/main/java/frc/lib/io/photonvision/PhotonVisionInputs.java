//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

/**
 * IO container for all PhotonVision camera data.
 *
 * This matches the AdvantageKit IO pattern:
 *  - Per-camera raw observations
 *  - A fused PV pose (computed by PVManager)
 *  - Metadata for connectivity
 */
public class PhotonVisionInputs {

    /** Inputs for the single Bluecam camera */
    public static class CameraInputs {
        public String name = "Bluecam";

        public boolean connected = false;

        public boolean hasTarget = false;
        public Rotation2d yaw = Rotation2d.kZero;
        public Rotation2d pitch = Rotation2d.kZero;

        public double timestamp = 0.0;

        /** Last raw PhotonVision pipeline result */
        public PhotonPipelineResult rawResult = null;
    }

    /** Single camera input */
    public CameraInputs camera = new CameraInputs();

    /** Fused PV pose (computed by PVManager) */
    public Optional<Pose2d> fusedPose = Optional.empty();
    public double fusedTimestamp = 0.0;

    /** Metadata */
    public boolean cameraConnected = false;
    public int totalTags = 0;
}
