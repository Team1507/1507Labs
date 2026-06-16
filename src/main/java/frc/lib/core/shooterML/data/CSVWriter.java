//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.shooterML.data;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.RobotBase;

public class CSVWriter {

    private static final String[] HEADER = {
        "rpm",
        "voltage",
        "statorCurrent",
        "supplyCurrent",
        "closedLoopError",
        "distance",
        "made",
        "pressDuration",
        "shotsInBurst"
    };

    private static final String CSV_PATH = 
        RobotBase.isReal()
            ? "/home/lvuser/shooter_data.csv"
            : "shooter_data.csv";

    public static void writeSingleRecord(ShotRecord r) {

        File file = new File(CSV_PATH);
        boolean writeHeader = !file.exists();

        try (FileWriter writer = new FileWriter(file, true)) {

            // Write header if file is new
            if (writeHeader) {
                writer.write(String.join(",", HEADER));
                writer.write("\n");
            }

            // Write the row
            writer.write(
                r.shooterRPM + "," +
                r.shooterVoltage + "," +
                r.statorCurrent + "," +
                r.supplyCurrent + "," +
                r.closedLoopError + "," +
                r.distanceToTarget + "," +
                r.made + "," +
                r.pressDuration + "," +
                r.shotsInBurst
            );

            writer.write("\n");

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
