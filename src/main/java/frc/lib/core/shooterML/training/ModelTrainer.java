//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.shooterML.training;

import java.io.FileWriter;
import java.util.List;

import frc.lib.core.shooterML.data.ShotRecord;
import frc.lib.core.shooterML.model.ModelConfig;

public class ModelTrainer {

    public static void train(String csvPath, String outputJsonPath) {

        // Load training data
        List<ShotRecord> records = CSVReader.read(csvPath);

        int n = records.size();
        int features = 7; 
        // distance, stator, supply, error, pressDuration, shotsInBurst, bias

        double[][] X = new double[n][features];
        double[] y = new double[n];

        for (int i = 0; i < n; i++) {
            ShotRecord r = records.get(i);

            X[i][0] = r.distanceToTarget;
            X[i][1] = r.statorCurrent;
            X[i][2] = r.supplyCurrent;
            X[i][3] = r.closedLoopError;
            X[i][4] = r.pressDuration;
            X[i][5] = r.shotsInBurst;
            X[i][6] = 1.0; // bias term

            y[i] = r.shooterRPM;
        }

        // Solve for coefficients using least squares
        double[] coeffs = RegressionMath.solveLeastSquares(X, y);

        // Build model config
        ModelConfig config = new ModelConfig();
        config.a = coeffs[0]; // distance
        config.b = coeffs[1]; // stator current
        config.c = coeffs[2]; // supply current
        config.d = coeffs[3]; // closed-loop error
        config.e = coeffs[4]; // pressDuration
        config.f = coeffs[5]; // shotsInBurst
        config.g = coeffs[6]; // bias

        // Write JSON
        try (FileWriter writer = new FileWriter(outputJsonPath)) {
            writer.write(config.toJson());
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public static void main(String[] args) {

        String csvPath;
        String outputPath;
    
        if (args.length >= 2) {
            csvPath = args[0];
            outputPath = args[1];
        } else {
            // Hardcoded defaults for VS Code Run/Debug
            csvPath = "shooter_data.csv";
            outputPath = "model.json";
            System.out.println("No arguments provided. Using defaults:");
            System.out.println("CSV: " + csvPath);
            System.out.println("JSON: " + outputPath);
        }
    
        System.out.println("Training model from: " + csvPath);
        System.out.println("Writing model to: " + outputPath);
    
        train(csvPath, outputPath);
    
        System.out.println("Done.");
    }    
}
