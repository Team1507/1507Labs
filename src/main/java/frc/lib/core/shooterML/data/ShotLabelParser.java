//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.shooterML.data;

public class ShotLabelParser {

    public static ShotLabel parse(String label) {
        if (label == null) return new ShotLabel(false);

        switch (label.toLowerCase()) {
            case "scored":
            case "made":
            case "yes":
            case "y":
            case "went in":
                return new ShotLabel(true);

            case "missed":
            case "miss":
            case "no":
            case "n":
            case "did not score":
                return new ShotLabel(false);

            default:
                return new ShotLabel(false);
        }
    }
}

