//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.framework.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;

import frc.lib.core.framework.io.CtreMotorIO;
import frc.lib.core.framework.io.MotorInputs1507;
import frc.lib.core.util.MotorConfig;
import frc.lib.core.util.MotorConfig.ControlMode;
import frc.lib.core.util.MotorConfig.GravityType;

/**
 * Unified motor abstraction for Team 1507.
 *
 * Owns motor hardware, configuration, telemetry, and control.
 * Subsystems decide when configuration is applied.
 */
public final class Motor1507 {

    public enum Type { FX, FXS }

    private final Object motor;
    private final MotorConfig[] configs;

    private final MotorInputs1507 inputs = new MotorInputs1507();
    private final CtreMotorIO telemetry;

    public Motor1507(Type type, int canId, MotorConfig... configs) {
        if (configs.length == 0 || configs[0].slotNumber() != 0) {
            throw new IllegalArgumentException("First MotorConfig must be slot 0");
        }

        this.motor = createMotor(type, canId);
        this.configs = configs;
        this.telemetry = CtreMotorIO.fromMotor(motor);
    }

    // ============================================================
    // CONFIGURATION
    // ============================================================

    public void applyConfig() {
        if (motor instanceof TalonFX fx) {
            applyFXConfig(fx);
        } else if (motor instanceof TalonFXS fxs) {
            applyFXSConfig(fxs);
        } else {
            throw new IllegalStateException("Unsupported motor type");
        }
    }

    private void applyFXConfig(TalonFX motor) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        for (MotorConfig config : configs) {
            applySlot(cfg, config);
        }

        MotorConfig base = configs[0];

        cfg.MotorOutput.Inverted = base.motorInverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        cfg.Voltage.withPeakForwardVoltage(Volts.of(base.peakForwardVoltage()))
                   .withPeakReverseVoltage(Volts.of(base.peakReverseVoltage()));

        cfg.MotorOutput.NeutralMode = base.brakeMode()
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        applyLimitSwitches(cfg.HardwareLimitSwitch, base);

        applyCurrentLimits(cfg.CurrentLimits, base);

        safeApply(motor, cfg);
    }

    private void applyFXSConfig(TalonFXS motor) {
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();

        cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        for (MotorConfig config : configs) {
            applySlot(cfg, config);
        }

        MotorConfig base = configs[0];

        cfg.MotorOutput.Inverted = base.motorInverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        cfg.Voltage.withPeakForwardVoltage(Volts.of(base.peakForwardVoltage()))
                   .withPeakReverseVoltage(Volts.of(base.peakReverseVoltage()));

        cfg.MotorOutput.NeutralMode = base.brakeMode()
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        applyLimitSwitches(cfg.HardwareLimitSwitch, base);

        applyCurrentLimits(cfg.CurrentLimits, base);

        safeApply(motor, cfg);
    }

    // ============================================================
    // APPLY CONFIG TO THE CORRECT SLOT (0, 1, OR 2)
    // ============================================================
    private void applySlot(TalonFXConfiguration cfg, MotorConfig config) {
        switch (config.slotNumber()) {
            case 1 -> applyToSlot(cfg.Slot1, config);
            case 2 -> applyToSlot(cfg.Slot2, config);
            default -> applyToSlot(cfg.Slot0, config);
        }
    }

    private void applySlot(TalonFXSConfiguration cfg, MotorConfig config) {
        switch (config.slotNumber()) {
            case 1 -> applyToSlot(cfg.Slot1, config);
            case 2 -> applyToSlot(cfg.Slot2, config);
            default -> applyToSlot(cfg.Slot0, config);
        }
    }

    // ============================================================
    // APPLY PID / FF / GRAVITY TO EACH SLOT TYPE
    // ============================================================
    private void applyToSlot(Slot0Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();

            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }

        if (config.gravityType() != GravityType.NONE) {
            switch (config.gravityType()) {
                case COSINE -> slot.GravityType = GravityTypeValue.Arm_Cosine;
                case CONSTANT -> slot.GravityType = GravityTypeValue.Elevator_Static;
                default -> {}
            }
            slot.kG = config.kG();
        }
    }

    private void applyToSlot(Slot1Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();

            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }

        if (config.gravityType() != GravityType.NONE) {
            switch (config.gravityType()) {
                case COSINE -> slot.GravityType = GravityTypeValue.Arm_Cosine;
                case CONSTANT -> slot.GravityType = GravityTypeValue.Elevator_Static;
                default -> {}
            }
            slot.kG = config.kG();
        }
    }

    private void applyToSlot(Slot2Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();

            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }

        if (config.gravityType() != GravityType.NONE) {
            switch (config.gravityType()) {
                case COSINE -> slot.GravityType = GravityTypeValue.Arm_Cosine;
                case CONSTANT -> slot.GravityType = GravityTypeValue.Elevator_Static;
                default -> {}
            }
            slot.kG = config.kG();
        }
    }

    // ============================================================
    // LIMIT SWITCH CONFIGURATION
    // ============================================================
    private void applyLimitSwitches(HardwareLimitSwitchConfigs hw, MotorConfig config) {

        // Forward limit switch
        hw.ForwardLimitEnable = config.forwardLimitEnable();
        hw.ForwardLimitAutosetPositionEnable = config.forwardLimitAutosetEnable();
        hw.ForwardLimitAutosetPositionValue = config.forwardLimitAutosetValue();
        hw.ForwardLimitType = config.forwardLimitType();

        // Reverse limit switch
        hw.ReverseLimitEnable = config.reverseLimitEnable();
        hw.ReverseLimitAutosetPositionEnable = config.reverseLimitAutosetEnable();
        hw.ReverseLimitAutosetPositionValue = config.reverseLimitAutosetValue();
        hw.ReverseLimitType = config.reverseLimitType();
    }

    // ============================================================
    // CURRENT LIMIT CONFIGURATION
    // ============================================================
    private void applyCurrentLimits(CurrentLimitsConfigs cl, MotorConfig config) {

        // ----------------------------
        // Stator current limiting
        // Mechanical protection
        // ----------------------------
        cl.StatorCurrentLimit = config.statorCurrentLimit().in(Amps);
        cl.StatorCurrentLimitEnable = true;

        // ----------------------------
        // Supply current limiting
        // Brownout protection
        // ----------------------------
        cl.SupplyCurrentLimit = config.supplyCurrentLimit().in(Amps);
        cl.SupplyCurrentLimitEnable = true;
    }

    // ============================================================
    // TELEMETRY
    // ============================================================

    public void update() {
        telemetry.updateInputs(inputs);
    }

    public MotorInputs1507 inputs() {
        return inputs;
    }

    // ============================================================
    // CONTROL
    // ============================================================

    /**
     * Applies a CTRE control request to the motor.
     *
     * <p>This is the single internal choke point for all motor control
     * requests, including both open-loop and closed-loop modes.</p>
     *
     * <p>All public control methods must route through this method
     * unless explicitly performing a hard stop.</p>
     *
     * <p><b>Design Notes:</b>
     * <ul>
     *   <li>No safety checks are performed here.</li>
     *   <li>No unit conversions are performed here.</li>
     *   <li>This method operates strictly in motor-native units.</li>
     *   <li>Subsystems are responsible for enforcing limits and intent.</li>
     * </ul>
     * </p>
     *
     * @param request CTRE control request to apply
     */
    private void setControl(ControlRequest request) {
        if (motor instanceof TalonFX fx) {
            fx.setControl(request);
        } else if (motor instanceof TalonFXS fxs) {
            fxs.setControl(request);
        }
    }

    /**
     * Runs the motor in open-loop duty cycle mode.
     *
     * <p>This commands the motor using a normalized duty cycle in the
     * range [-1.0, 1.0]. This is equivalent to percent output and does
     * not use any closed-loop control.</p>
     *
     * <p>This mode is appropriate for simple mechanisms such as rollers,
     * intakes, or conveyors where precise position or velocity control
     * is not required.</p>
     *
     * <p><b>Units:</b> Duty cycle (-1.0 to 1.0)</p>
     *
     * @param dutyCycle Open-loop output percentage
     */
    public void runDuty(double dutyCycle) {
        setControl(new DutyCycleOut(dutyCycle));
    }

    /**
     * Commands the motor to a target position using duty-cycle-based
     * position control.
     *
     * <p>This mode does not use PID gains, feedforward, or gravity
     * compensation. It is intended for simple, bounded mechanisms
     * such as linear extenders that move to a position and stop.</p>
     *
     * <p>This control mode should be used only when precise holding
     * behavior is not required.</p>
     *
     * <p><b>Units:</b> Motor rotations</p>
     *
     * @param rotations Target motor position in rotations
     */
    public void setPositionDuty(double rotations) {
        setControl(new PositionDutyCycle(rotations));
    }

    /**
     * Commands the motor to a target position using voltage-based
     * closed-loop control.
     *
     * <p>This mode uses the configured PID gains, feedforward, and
     * optional gravity compensation. It is intended for precision
     * mechanisms such as arms or elevators.</p>
     *
     * <p><b>Units:</b>
     * <ul>
     *   <li>Position: motor rotations</li>
     *   <li>Feedforward: volts</li>
     * </ul>
     * </p>
     *
     * @param rotations Target motor position in rotations
     * @param ffVolts   Feedforward voltage
     */
    public void setPositionVoltage(double rotations, double ffVolts) {
        setControl(
            new PositionVoltage(rotations)
                .withFeedForward(ffVolts)
        );
    }

    /**
     * Commands the motor to a target velocity in rotations per second
     * using closed-loop voltage control.
     *
     * <p>This overload applies zero feedforward and relies solely on
     * the configured PID gains.</p>
     *
     * <p><b>Units:</b> Motor rotations per second (RPS)</p>
     *
     * @param motorRPS Target motor velocity in rotations per second
     */
    public void setVelocityRPS(double motorRPS) {
        setControl(new VelocityVoltage(motorRPS));
    }

    /**
     * Commands the motor to a target velocity in rotations per second
     * using closed-loop voltage control with feedforward.
     *
     * <p>This mode is appropriate for mechanisms such as shooters
     * where consistent velocity under load is required.</p>
     *
     * <p><b>Units:</b>
     * <ul>
     *   <li>Velocity: motor rotations per second (RPS)</li>
     *   <li>Feedforward: volts</li>
     * </ul>
     * </p>
     *
     * @param motorRPS Target motor velocity in rotations per second
     * @param ffVolts  Feedforward voltage
     */
    public void setVelocityRPS(double motorRPS, double ffVolts) {
        setControl(
            new VelocityVoltage(motorRPS)
                .withFeedForward(ffVolts)
        );
    }

    /**
     * Immediately stops the motor and cancels any active control request.
     *
     * <p>This method performs a hard stop using the motor controller's
     * native stop command. It bypasses the control request pipeline
     * and does not rely on closed-loop behavior.</p>
     *
     * <p>The motor's configured neutral mode (brake or coast) determines
     * the physical response.</p>
     */
    public void stop() {
        if (motor instanceof TalonFX fx) {
            fx.stopMotor();
        } else if (motor instanceof TalonFXS fxs) {
            fxs.stopMotor();
        }
    }

    // ============================================================
    // INTERNAL
    // ============================================================

    private static Object createMotor(Type type, int canId) {
        return switch (type) {
            case FX  -> new TalonFX(canId);
            case FXS -> new TalonFXS(canId);
        };
    }

    private static void safeApply(TalonFX motor, TalonFXConfiguration cfg) {
        var status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("[WARN] Failed to apply config to FX " + motor.getDeviceID()
                    + ": " + status.toString());
        }
    }

    private static void safeApply(TalonFXS motor, TalonFXSConfiguration cfg) {
        var status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("[WARN] Failed to apply config to FXS " + motor.getDeviceID()
                    + ": " + status.toString());
        }
    }
}
