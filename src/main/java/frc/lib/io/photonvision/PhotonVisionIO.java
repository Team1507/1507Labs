//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.photonvision;

import frc.robot.localization.vision.PhotonPoseEstimator;

/**
 * IO abstraction for PhotonVision.
 *
 * PVManager depends on this interface instead of directly touching
 * PhotonCamera or PhotonPoseEstimator.
 */
public interface PhotonVisionIO {

    /**
     * Update all PhotonVision inputs.
     *
     * @param inputs The PhotonVisionInputs object to populate.
     * @param seeded Whether the pose estimator has been seeded.
     */
    void updateInputs(PhotonVisionInputs inputs);

    /** Number of cameras in the system. */
    int getCameraCount();

    /** Returns the human-readable camera names. */
    String[] getCameraNames();

    /** Returns the PhotonPoseEstimator for a given camera index. */
    PhotonPoseEstimator getEstimator(int index);
}
