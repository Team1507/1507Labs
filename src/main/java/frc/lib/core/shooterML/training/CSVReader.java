//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.shooterML.training;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.core.shooterML.data.ShotRecord;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class CSVReader {

    public static List<ShotRecord> read(String path) {
        List<ShotRecord> records = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(path))) {

            // Skip header
            String line = br.readLine();

            while ((line = br.readLine()) != null) {
                String[] parts = line.split(",");

                double rpm = Double.parseDouble(parts[0]);
                double voltage = Double.parseDouble(parts[1]);
                double stator = Double.parseDouble(parts[2]);
                double supply = Double.parseDouble(parts[3]);
                double error = Double.parseDouble(parts[4]);

                double distance = Double.parseDouble(parts[5]);
                boolean made = Boolean.parseBoolean(parts[6]);

                // NEW FIELDS
                double pressDuration = Double.parseDouble(parts[7]);
                int shotsInBurst = Integer.parseInt(parts[8]);

                // Pose is not stored in CSV → use dummy pose
                Pose2d dummyPose = new Pose2d();

                ShotRecord r = new ShotRecord(
                    rpm,
                    voltage,
                    stator,
                    supply,
                    error,
                    dummyPose,
                    distance
                );

                r.setLabel(made);
                r.setPressData(pressDuration, shotsInBurst);

                records.add(r);
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        return records;
    }
}
