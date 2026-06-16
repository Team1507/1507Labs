//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.hardware;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.kVision;

/**
 * Hardware definitions for PhotonVision cameras.
 *
 * This contains ONLY physical configuration:
 *  - camera name
 *  - robot-to-camera transform
 *
 * No PhotonVision classes or logic belong here.
 */
public final class PhotonVisionHardware {

    public static final class CameraConfig {
        public final String name;
        public final Transform3d robotToCamera;

        public CameraConfig(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }
    }

    /** Single-camera system: Bluecam only */
    public static final CameraConfig BLUECAM =
        new CameraConfig("Bluecam", kVision.BLU.ROBOT_TO_CAMERA);

    private PhotonVisionHardware() {}
}