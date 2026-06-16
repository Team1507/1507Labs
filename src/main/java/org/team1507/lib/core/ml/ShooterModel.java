//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.ml;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Files;

// ─────────────────────────────────────────────────────────────────────────────
// ShooterModel
//
// Quadratic distance-to-RPM model trained on real shot data.
// Coefficients are loaded from deploy/model.json at startup.
//
// Formula: RPM = (b * d²) + (a * d) + g
//   where d = distance to target in meters.
//
// If model.json is missing or malformed, load() returns a linear fallback
// (b=0, a=200, g=2400) so the robot can still shoot.
//
// Usage:
//   ShooterModel model = ShooterModel.load();
//   double rpm = model.getRPM(distanceMeters);
// ─────────────────────────────────────────────────────────────────────────────
public final class ShooterModel {

    private final double a, b, g;

    private ShooterModel(double a, double b, double g) {
        this.a = a;
        this.b = b;
        this.g = g;
    }

    /** Returns predicted flywheel RPM for the given distance to target (meters). */
    public double getRPM(double distanceMeters) {
        double d = distanceMeters;
        return (b * d * d) + (a * d) + g;
    }

    /**
     * Loads deploy/model.json and returns a trained quadratic model.
     * Falls back to a linear approximation (RPM = 200*d + 2400) if loading fails.
     */
    public static ShooterModel load() {
        try {
            String json = Files.readString(
                Filesystem.getDeployDirectory().toPath().resolve("model.json"));
            JsonNode root = new ObjectMapper().readTree(json);
            return new ShooterModel(
                root.get("a").asDouble(),
                root.get("b").asDouble(),
                root.get("g").asDouble()
            );
        } catch (Exception e) {
            System.err.println("[ShooterModel] Failed to load model.json, using linear fallback: " + e.getMessage());
            return new ShooterModel(200.0, 0.0, 2400.0);
        }
    }
}
