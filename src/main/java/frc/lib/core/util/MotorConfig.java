//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.util;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.units.measure.Current;

public record MotorConfig(
        int slotNumber,

        ControlMode mode,
        boolean motorInverted,

        double kP, double kI, double kD,
        double kV, double kS, double kA,

        double kG,
        GravityType gravityType,

        double peakForwardVoltage,
        double peakReverseVoltage,

        Current statorCurrentLimit,
        Current supplyCurrentLimit,

        boolean forwardLimitEnable,
        boolean forwardLimitAutosetEnable,
        double forwardLimitAutosetValue,
        ForwardLimitTypeValue forwardLimitType,

        boolean reverseLimitEnable,
        boolean reverseLimitAutosetEnable,
        double reverseLimitAutosetValue,
        ReverseLimitTypeValue reverseLimitType,

        boolean brakeMode
) {

    public static enum ControlMode { DUTY_CYCLE, VELOCITY, POSITION, MOTION_MAGIC }
    public static enum GravityType { NONE, COSINE, SINE, CONSTANT }

    public static Builder builder() {
        return new Builder();
    }

    public static Builder builder(ControlMode mode) {
        return new Builder().mode(mode);
    }

    public static final class Builder {

        private int slotNumber = 0;

        private ControlMode mode = ControlMode.DUTY_CYCLE;
        private boolean motorInverted = false;

        private double kP = 0, kI = 0, kD = 0;
        private double kV = 0, kS = 0, kA = 0;

        private double kG = 0;
        private GravityType gravityType = GravityType.NONE;

        private double peakForwardVoltage = 12;
        private double peakReverseVoltage = -12;

        private Current statorCurrentLimit = Amps.of(120.0);
        private Current supplyCurrentLimit = Amps.of(70.0);

        private boolean forwardLimitEnable = false;
        private boolean forwardLimitAutosetEnable = false;
        private double forwardLimitAutosetValue = 0.0;
        private ForwardLimitTypeValue forwardLimitType = ForwardLimitTypeValue.NormallyOpen;

        private boolean reverseLimitEnable = false;
        private boolean reverseLimitAutosetEnable = false;
        private double reverseLimitAutosetValue = 0.0;
        private ReverseLimitTypeValue reverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        private boolean brakeMode = false; // DEFAULT = COAST

        // ============================
        // Builder Methods
        // ============================

        public Builder mode(ControlMode m) { this.mode = m; return this; }
        public Builder inverted(boolean inv) { this.motorInverted = inv; return this; }

        public Builder slot(int slot) {
            this.slotNumber = slot; return this;
        }

        public Builder withPID(double p, double i, double d) {
            this.kP = p; this.kI = i; this.kD = d; return this;
        }

        public Builder withFeedforward(double kS, double kV, double kA) {
            this.kS = kS; this.kV = kV; this.kA = kA; return this;
        }

        public Builder withGravity(double kG, GravityType type) {
            this.kG = kG; this.gravityType = type; return this;
        }

        public Builder withVoltageLimits(double fwd, double rev) {
            this.peakForwardVoltage = fwd; this.peakReverseVoltage = rev; return this;
        }

        public Builder withStatorCurrentLimit(Current statorCurrentLimit) {
            this.statorCurrentLimit = statorCurrentLimit;
            return this;
        }

        public Builder withSupplyCurrentLimit(Current statorCurrentLimit) {
            this.statorCurrentLimit = statorCurrentLimit;
            return this;
        }

        public Builder withForwardLimit(boolean enable, boolean autoset, double autosetValue) {
            this.forwardLimitEnable = enable;
            this.forwardLimitAutosetEnable = autoset;
            this.forwardLimitAutosetValue = autosetValue;
            return this;
        }

        public Builder forwardLimitType(ForwardLimitTypeValue type) {
            this.forwardLimitType = type;
            return this;
        }

        public Builder withReverseLimit(boolean enable, boolean autoset, double autosetValue) {
            this.reverseLimitEnable = enable;
            this.reverseLimitAutosetEnable = autoset;
            this.reverseLimitAutosetValue = autosetValue;
            return this;
        }

        public Builder reverseLimitType(ReverseLimitTypeValue type) {
            this.reverseLimitType = type;
            return this;
        }
        
        /** Sets the motor to brake mode. */
        public Builder withBrake() {
            this.brakeMode = true;
            return this;
        }

        /** Sets the motor to coast mode. */
        public Builder withCoast() {
            this.brakeMode = false;
            return this;
        }

        // ============================
        // Build
        // ============================

        public MotorConfig build() {
            return new MotorConfig(
                slotNumber,
                mode, motorInverted,
                kP, kI, kD,
                kV, kS, kA,
                kG, gravityType,
                peakForwardVoltage, peakReverseVoltage,

                statorCurrentLimit,
                supplyCurrentLimit,

                forwardLimitEnable,
                forwardLimitAutosetEnable,
                forwardLimitAutosetValue,
                forwardLimitType,

                reverseLimitEnable,
                reverseLimitAutosetEnable,
                reverseLimitAutosetValue,
                reverseLimitType,

                brakeMode
            );
        }
    }
}
