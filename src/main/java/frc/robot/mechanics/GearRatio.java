package frc.robot.mechanics;

import java.util.ArrayList;
import java.util.List;

public class GearRatio {

    private final double motorToOutput;

    public GearRatio(double motorToOutput) {
        this.motorToOutput = motorToOutput;
    }

    // -----------------------------
    // Simple readable constructors
    // -----------------------------

    /** Example: reduction(10, 1) = 10:1 */
    public static GearRatio gearBox(double motor, double output) {
        return new GearRatio(motor / output);
    }

    // -----------------------------
    // Multi-stage gear builder
    // -----------------------------

    public static Builder Gears() {
        return new Builder();
    }

    public static class Builder {
        private final List<Double> stageRatios = new ArrayList<>();

        /**
         * Add a gear stage by specifying the input and output gear sizes.
         * Example: add(12, 36) = 12T driving 36T = 3:1 reduction.
         */
        public Builder add(double inputGear, double outputGear) {
            stageRatios.add(outputGear / inputGear);
            return this;
        }

        /**
         * Add a precomputed stage ratio directly.
         * Example: add(3.0) = 3:1 reduction.
         */
        public Builder add(double ratio) {
            stageRatios.add(ratio);
            return this;
        }

        public GearRatio build() {
            double total = 1.0;
            for (double r : stageRatios) {
                total *= r;
            }
            return new GearRatio(total);
        }
    }

    // -----------------------------
    // Conversion helpers
    // -----------------------------

    /** Convert output speed → motor shaft speed */
    public double toMotor(double outputSpeed) {
        return outputSpeed * motorToOutput;
    }

    /** Convert motor shaft speed → output speed */
    public double toOutput(double motorSpeed) {
        return motorSpeed / motorToOutput;
    }

    public double getRatio() {
        return motorToOutput;
    }
}
