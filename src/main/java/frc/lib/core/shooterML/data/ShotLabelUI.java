//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.shooterML.data;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * ShotLabelUI writes labels for a specific shot ID into NetworkTables.
 *
 * This replaces the Shuffleboard widget and directly mirrors the
 * ShotTrainer NT structure:
 *
 *   /Shooter ML/UnlabeledShots/<id>/label
 */
public class ShotLabelUI {

    private final NetworkTable shotTable;

    /**
     * @param shotId The ID of the shot you want to label.
     */
    public ShotLabelUI(int shotId) {
        this.shotTable =
            NetworkTableInstance.getDefault()
                .getTable("Shooter ML")
                .getSubTable("UnlabeledShots")
                .getSubTable(Integer.toString(shotId));
    }

    /** Mark the shot as made. */
    public void setMade() {
        shotTable.getEntry("label").setString("made");
    }

    /** Mark the shot as missed. */
    public void setMissed() {
        shotTable.getEntry("label").setString("missed");
    }

    /** Write any custom label string (e.g., "rim", "airball", etc.). */
    public void setLabel(String label) {
        shotTable.getEntry("label").setString(label);
    }
}
