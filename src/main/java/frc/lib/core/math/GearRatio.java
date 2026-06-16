//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.math;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a motor-to-output gear ratio for a mechanism.
 * <p>
 * A GearRatio stores the conversion factor between motor shaft speed and
 * mechanism output speed. A ratio greater than 1.0 indicates a reduction
 * (motor spins faster than the output). A ratio less than 1.0 indicates
 * an overdrive (output spins faster than the motor).
 * <p>
 * This class also includes a builder for constructing multi-stage gear
 * or pulley reductions by multiplying individual stage ratios.
 * <p>
 * Additionally, GearRatio can apply a linear scaling transform to convert
 * arbitrary sensor units (such as CTRE motor position values) into
 * real-world units (degrees, inches, etc.). This allows mechanisms to be
 * expressed in meaningful physical units rather than raw sensor values.
 */
public class GearRatio {

    /** Motor-to-output speed ratio (motorSpeed / outputSpeed). */
    private final double motorToOutput;

    /** Linear scaling slope: realUnits = slope * sensor + offset. */
    private double slope = 1.0;

    /** Linear scaling offset: realUnits = slope * sensor + offset. */
    private double offset = 0.0;

    /**
     * Creates a GearRatio with a precomputed motor-to-output ratio.
     *
     * @param motorToOutput the ratio of motor speed to output speed
     */
    public GearRatio(double motorToOutput) {
        this.motorToOutput = motorToOutput;
    }

    // -----------------------------
    // Simple readable constructors
    // -----------------------------

    /**
     * Creates a simple gearbox ratio from motor and output gear sizes.
     * <p>
     * Example: {@code gearBox(10, 1)} produces a 10:1 reduction.
     *
     * @param motor  number of teeth (or diameter) on the motor-side gear
     * @param output number of teeth (or diameter) on the output-side gear
     * @return a new GearRatio representing motor/output
     */
    public static GearRatio gearBox(double motor, double output) {
        return new GearRatio(motor / output);
    }

    // -----------------------------
    // Multi-stage gear builder
    // -----------------------------

    /**
     * Creates a new multi-stage gear ratio builder.
     * <p>
     * Use this when your mechanism has multiple gear or pulley stages.
     *
     * @return a new {@link Builder} instance
     */
    public static Builder Gears() {
        return new Builder();
    }

    /**
     * Builder for constructing multi-stage gear or pulley reductions.
     * <p>
     * Each stage is multiplied together to form the final motor-to-output ratio.
     * Stages may be added using gear sizes (input/output) or precomputed ratios.
     */
    public static class Builder {
        private final List<Double> stageRatios = new ArrayList<>();

        /**
         * Adds a gear or pulley stage using input and output sizes.
         * <p>
         * Example: {@code add(12, 36)} means a 12T gear driving a 36T gear,
         * which is a 3:1 reduction (motor spins 3× faster than output).
         *
         * @param inputGear  number of teeth (or diameter) on the driving gear
         * @param outputGear number of teeth (or diameter) on the driven gear
         * @return this builder for chaining
         */
        public Builder add(double inputGear, double outputGear) {
            stageRatios.add(outputGear / inputGear);
            return this;
        }

        /**
         * Adds a precomputed stage ratio directly.
         * <p>
         * Example: {@code add(3.0)} represents a 3:1 reduction.
         *
         * @param ratio motor-to-output ratio for this stage
         * @return this builder for chaining
         */
        public Builder add(double ratio) {
            stageRatios.add(ratio);
            return this;
        }

        /**
         * Builds the final GearRatio by multiplying all stage ratios.
         *
         * @return a new {@link GearRatio} representing the total reduction
         */
        public GearRatio build() {
            double total = 1.0;
            for (double r : stageRatios) {
                total *= r;
            }
            return new GearRatio(total);
        }
    }

    // -----------------------------
    // Scaling helpers
    // -----------------------------

    /**
     * Applies a linear scaling transform using two known calibration points.
     * <p>
     * This maps:
     * <pre>
     *   motorStart → realStart
     *   motorEnd   → realEnd
     * </pre>
     * and computes a linear transform such that:
     * <pre>
     *   real = slope * motor + offset
     * </pre>
     *
     * @param motorStart raw motor/sensor value at the first calibration point
     * @param realStart  real-world value at the first calibration point
     * @param motorEnd   raw motor/sensor value at the second calibration point
     * @param realEnd    real-world value at the second calibration point
     * @return this GearRatio for chaining
     */
    public GearRatio withScaling(double motorStart, double realStart,
                                 double motorEnd, double realEnd) {
        this.slope = (realEnd - realStart) / (motorEnd - motorStart);
        this.offset = realStart - slope * motorStart;
        return this;
    }

    /**
     * Applies a linear scaling transform assuming the real-world value at
     * {@code motorStart} is zero.
     * <p>
     * This maps:
     * <pre>
     *   motorStart → 0
     *   motorEnd   → realEnd
     * </pre>
     *
     * @param motorStart raw motor/sensor value corresponding to real = 0
     * @param motorEnd   raw motor/sensor value at the second calibration point
     * @param realEnd    real-world value at the second calibration point
     * @return this GearRatio for chaining
     */
    public GearRatio withScaling(double motorStart, double motorEnd, double realEnd) {
        this.slope = realEnd / (motorEnd - motorStart);
        this.offset = -slope * motorStart;
        return this;
    }

    /**
     * Converts a raw motor/sensor value into a real-world scaled value.
     *
     * @param motorValue raw motor or sensor reading
     * @return scaled real-world value
     */
    public double sensorToReal(double motorValue) {
        return slope * motorValue + offset;
    }

    /**
     * Converts a real-world value back into the raw motor/sensor units.
     *
     * @param realValue real-world value
     * @return equivalent motor/sensor reading
     */
    public double realToSensor(double realValue) {
        return (realValue - offset) / slope;
    }

    // -----------------------------
    // Gear ratio conversion helpers (UNSCALED)
    // -----------------------------

    /**
     * Converts output shaft speed to motor shaft speed.
     * <p>
     * Example: with an 8:1 reduction, {@code toMotor(500)} returns 4000.
     *
     * @param outputSpeed speed of the mechanism output (RPM, RPS, etc.)
     * @return equivalent motor shaft speed in the same units
     */
    public double toMotor(double outputSpeed) {
        return outputSpeed * motorToOutput;
    }

    /**
     * Converts motor shaft speed to output shaft speed.
     * <p>
     * Example: with an 8:1 reduction, {@code toOutput(4000)} returns 500.
     *
     * @param motorSpeed speed of the motor shaft (RPM, RPS, etc.)
     * @return equivalent mechanism output speed in the same units
     */
    public double toOutput(double motorSpeed) {
        return motorSpeed / motorToOutput;
    }

    /**
     * Returns the motor-to-output ratio.
     * <p>
     * A value of 8.0 means the motor spins 8× faster than the output.
     *
     * @return motor-to-output speed ratio
     */
    public double getRatio() {
        return motorToOutput;
    }
}
