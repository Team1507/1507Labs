package frc.lib.core.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ConcurrentModificationException;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

/**
 * A lightweight, allocation-free command builder designed for IO-based subsystems.
 *
 * <p>This class avoids WPILib decorators and FunctionalCommand wrappers, ensuring:
 * <ul>
 *   <li>No per-loop allocations</li>
 *   <li>Deterministic scheduler behavior</li>
 *   <li>Clear separation of driver intent and mechanical fault handling</li>
 *   <li>Simple, readable, declarative command construction</li>
 * </ul>
 *
 * <p>Supports three independent termination causes:
 * <ul>
 *   <li>Normal finish condition</li>
 *   <li>Timeout expiration</li>
 *   <li>Stall detection</li>
 * </ul>
 *
 * <p>The {@code onEnd} handler receives all three cause flags.
 */
public class CommandBuilder extends Command {

    // -----------------------------
    // Lifecycle callbacks
    // -----------------------------
    private Runnable onInit = () -> {};
    private Runnable onExecute = () -> {};

    /**
     * End handler with full termination cause information.
     */
    @FunctionalInterface
    public interface EndHandler {
        /**
         * @param interrupted true if the scheduler interrupted the command
         * @param timedOut    true if the command ended due to timeout
         * @param stalled     true if the command ended due to stall detection
         */
        void accept(boolean interrupted, boolean timedOut, boolean stalled);
    }

    private EndHandler onEnd = (i, t, s) -> {};

    // -----------------------------
    // Finish conditions
    // -----------------------------
    private BooleanSupplier finishCondition = () -> false;
    private BooleanSupplier stallCondition = null;

    // -----------------------------
    // Timeout tracking
    // -----------------------------
    private double timeoutSeconds = -1.0;
    private double startTime = 0.0;
    private boolean timedOut = false;

    // -----------------------------
    // Stall tracking
    // -----------------------------
    private boolean stalled = false;

    /**
     * Creates a new CommandBuilder with subsystem requirements.
     *
     * @param requirements subsystems required by this command
     */
    public CommandBuilder(Subsystem... requirements) {
        addRequirements(requirements);
    }

    /**
     * Assigns a name to the command for debugging.
     *
     * @param name command name
     * @return this builder
     */
    public CommandBuilder named(String name) {
        setName(name);
        return this;
    }

    /**
     * Sets the initialization routine, called once when the command is scheduled.
     *
     * @param r initialization action
     * @return this builder
     */
    public CommandBuilder onInitialize(Runnable r) {
        ensureNotRunning();
        this.onInit = r;
        return this;
    }

    /**
     * Sets the execution routine, called every scheduler cycle.
     *
     * @param r execution action
     * @return this builder
     */
    public CommandBuilder onExecute(Runnable r) {
        ensureNotRunning();
        this.onExecute = r;
        return this;
    }

    /**
     * Sets the end handler with full termination cause information.
     *
     * @param r end handler
     * @return this builder
     */
    public CommandBuilder onEnd(EndHandler r) {
        ensureNotRunning();
        this.onEnd = r;
        return this;
    }

    /**
     * Backwards-compatible overload for handlers that only care about
     * interrupted and timeout causes.
     *
     * @param r handler receiving (interrupted, timedOut)
     * @return this builder
     */
    public CommandBuilder onEnd(BiConsumer<Boolean, Boolean> r) {
        return onEnd((i, t, s) -> r.accept(i, t));
    }

    /**
     * Sets the normal finish condition.
     *
     * @param condition boolean supplier that returns true when the command should end
     * @return this builder
     */
    public CommandBuilder isFinished(BooleanSupplier condition) {
        ensureNotRunning();
        this.finishCondition = condition;
        return this;
    }

    /**
     * Convenience overload for constant finish conditions.
     *
     * @param finished whether the command should finish immediately
     * @return this builder
     */
    public CommandBuilder isFinished(boolean finished) {
        return isFinished(() -> finished);
    }

    /**
     * Sets a timeout for the command. When the timeout expires, the command ends
     * and {@code timedOut} is set to true.
     *
     * @param seconds timeout duration
     * @return this builder
     */
    public CommandBuilder timeout(double seconds) {
        ensureNotRunning();
        this.timeoutSeconds = seconds;
        return this;
    }

    /**
     * Forces the command to run until the timeout expires, ignoring any normal
     * finish condition.
     *
     * @param seconds timeout duration
     * @return this builder
     */
    public CommandBuilder timeoutOnly(double seconds) {
        ensureNotRunning();
        this.timeoutSeconds = seconds;
        this.finishCondition = () -> false;
        return this;
    }

    /**
     * Sets a stall finish condition. When the supplier returns true, the command
     * ends and {@code stalled} is set to true.
     *
     * @param stallCondition boolean supplier indicating stall state
     * @return this builder
     */
    public CommandBuilder stallFinish(BooleanSupplier stallCondition) {
        ensureNotRunning();
        this.stallCondition = stallCondition;
        return this;
    }

    // -----------------------------
    // WPILib lifecycle
    // -----------------------------

    @Override
    public void initialize() {
        timedOut = false;
        stalled = false;
        startTime = Timer.getFPGATimestamp();
        onInit.run();
    }

    @Override
    public void execute() {
        onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.accept(interrupted, timedOut, stalled);
    }

    @Override
    public boolean isFinished() {

        // Timeout check
        if (timeoutSeconds >= 0) {
            double elapsed = Timer.getFPGATimestamp() - startTime;
            if (elapsed >= timeoutSeconds) {
                timedOut = true;
                return true;
            }
        }

        // Stall check
        if (stallCondition != null && stallCondition.getAsBoolean()) {
            stalled = true;
            return true;
        }

        // Normal finish
        return finishCondition.getAsBoolean();
    }

    // -----------------------------
    // Internal safety
    // -----------------------------
    private void ensureNotRunning() {
        if (isScheduled()) {
            throw new ConcurrentModificationException(
                "Cannot modify a running CommandBuilder"
            );
        }
    }
}