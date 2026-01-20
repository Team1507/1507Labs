package frc.robot.commands;

// Java Libraries
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

// WPI Libraries
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// CTRE Libraries
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// Robot Subsystems
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command that performs a structured, multi-phase PID tuning sequence
 * for the shooter subsystem. The tuner executes controlled step tests,
 * spike tests, and recovery cycles, collecting telemetry and generating
 * a detailed report including overshoot, rise time, settling time, and
 * PID recommendations.
 *
 * <p>This command is designed to run autonomously and produce a
 * timestamped report file for long-term analysis.</p>
 */
public class CmdShooterPIDTuner extends Command {

    // -----------------------------
    // Shooter subsystem
    // -----------------------------
    private final ShooterSubsystem shooter;

    // -----------------------------
    // Test configuration
    // -----------------------------
    private final double maxRpm;
    private final double slowRampSec = 2.0;
    private final double holdSec = 1.0;
    private final double rpmTolerance = 50.0;
    private final double maxTestTimeSec = 4.0;

    // -----------------------------
    // State machine
    // -----------------------------

    /**
     * Enumeration of all phases in the PID tuning sequence.
     * Each phase defines a specific type of test:
     * <ul>
     *  <li>Slow ramp-up steps</li>
     *  <li>Slow ramp-down steps</li>
     *  <li>Fast spike to max RPM</li>
     *  <li>Recovery spike cycles</li>
     * </ul> 
     */
    private enum Phase { 
        // Phase 1: Baseline characterization
        SLOW_UP_STEP,
        SLOW_DOWN_STEP,
        FAST_SPIKE_MAX,

        // Phase 2: Recovery cycles
        RECOVER1_SPIKE_MAX,
        RECOVER2_SPIKE_4_5,
        RECOVER3_SPIKE_4_5_REPEAT,

        DONE
    }

    private Phase phase = Phase.DONE;

    // Step indexing for Phase 1
    private int stepIndex = 0;

    // Controls how far SLOW_DOWN_STEP goes and what comes next
    private int minSlowDownStepIndex = 0;
    private Phase nextPhaseAfterSlowDown = Phase.DONE;

    private double startTime;
    private double startSetpoint;
    private double endSetpoint;

    // -----------------------------
    // Data Collection
    // -----------------------------
    
    private final List<Sample> samples = new ArrayList<>();
    private final StringBuilder fullReport = new StringBuilder();
    private final List<String> phaseSuggestions = new ArrayList<>();
    private final List<String> phaseFfSuggestions = new ArrayList<>();
    private List<Double> phaseFfSeverities = new ArrayList<>();
    private final List<Phase> phaseTypes = new ArrayList<>();

    private double testStartTime;

    private double maxOvershootSeen = 0;

    private double totalRiseTime = 0;
    private int riseTimeCount = 0;

    private double totalSettlingTime = 0;
    private int settlingTimeCount = 0;

    enum FfCategory {
        INCREASE_KV,
        DECREASE_KV,
        ADJUST_KA,
        OK
    }

    /** 
     * Represents a single telemetry sample collected during a phase. 
     * Each sample records time, setpoint, measured RPM, and voltage.
     */
    private static class Sample {
        final double t;
        final double setpoint;
        final double rpm;

        Sample(double t, double setpoint, double rpm, double voltage) {
            this.t = t;
            this.setpoint = setpoint;
            this.rpm = rpm;
        }
    }

    // -----------------------------
    // Constructor
    // -----------------------------

    /**
     * Creates a new PID tuner for the shooter subsystem.
     * 
     * @param shooter the shooter subsystem being tuned 
     * @param maxRpm the maximum RPM used for spike and step tests 
     */
    public CmdShooterPIDTuner(ShooterSubsystem shooter, double maxRpm) {
        this.shooter = shooter;
        this.maxRpm = maxRpm;
        addRequirements(shooter);
    }

    // -----------------------------
    // initialize()
    // -----------------------------

    /**
     * Initializes the tuning sequence by resetting timers, clearing
     * previous data, and entering the first phase of the test.
     */
    @Override
    public void initialize() {
        testStartTime = Timer.getFPGATimestamp();

        phaseFfSeverities.clear();

        shooter.resetSimulationState();

        phase = Phase.SLOW_UP_STEP;
        stepIndex = 1;
        minSlowDownStepIndex = 0;
        nextPhaseAfterSlowDown = Phase.FAST_SPIKE_MAX;
        configurePhase();

        // clear previous run 
        fullReport.setLength(0); 
        fullReport.append("=== Shooter PID + Feedforward Tuning Test Starting ===\n\n");
        phaseSuggestions.clear();
        phaseFfSuggestions.clear();
        maxOvershootSeen = 0;
        totalRiseTime = 0;
        riseTimeCount = 0;
        totalSettlingTime = 0;
        settlingTimeCount = 0;

        shooter.setTargetRPM(0);
    }

    // -----------------------------
    // execute()
    // -----------------------------

    /**
     * Executes the active phase of the tuning sequence. This method:
     * <ul>
     *   <li>Computes the current setpoint</li>
     *   <li>Commands the shooter subsystem</li>
     *   <li>Collects telemetry samples</li>
     *   <li>Checks for phase completion</li>
     *   <li>Advances the state machine when needed</li>
     * </ul>
     */
    @Override
    public void execute() {
        if (phase == Phase.DONE) return;

        double now = Timer.getFPGATimestamp();
        double t = now - startTime;

        double setpoint = computeSetpoint(t);
        shooter.setTargetRPM(setpoint);

        samples.add(new Sample(
            t,
            setpoint,
            shooter.getShooterRPM(),
            shooter.getShooterVoltage()
        ));

        if (phaseComplete(t)) {
            analyzeAndReport();
            advancePhase();
        }

        // Publish data to NT
        SmartDashboard.putNumber("PIDTuner/ActualRPM", shooter.getShooterRPM());
        SmartDashboard.putNumber("PIDTuner/TargetRPM", setpoint);
        SmartDashboard.putNumber("PIDTuner/Voltage", shooter.getShooterVoltage());
        SmartDashboard.putNumber("PIDTuner/StatorCurrent", shooter.getStatorCurrent());
        SmartDashboard.putNumber("PIDTuner/SupplyCurrent", shooter.getSupplyCurrent());
        SmartDashboard.putNumber("PIDTuner/ClosedLoopError", shooter.getClosedLoopError());
    }

    // -----------------------------
    // isFinished()
    // -----------------------------

    /**
     * Indicates whether the tuning sequence has completed all phases.
     *
     * @return true when the state machine reaches DONE
     */
    @Override
    public boolean isFinished() {
        return phase == Phase.DONE;
    }

    // -----------------------------
    // end()
    // -----------------------------
    
    /** 
     * Finalizes the tuning sequence by generating the summary section 
     * and writing the full report to a timestamped file. 
     * 
     * @param interrupted whether the command was interrupted 
     */
    @Override
    public void end(boolean interrupted) {
        if (phase == Phase.DONE) {
            appendFinalSummary();
            writeFullReportToFile();
        }
    }

    // =====================================================================
    // PRIVATE HELPERS
    // =====================================================================

    /**
     * Configures the start and end setpoints for the current phase.
     * This method resets the sample buffer and timestamps the phase.
     */
    private void configurePhase() {
        samples.clear();
        startTime = Timer.getFPGATimestamp();

        switch (phase) { 
            // -----------------------------
            // Phase 1: Slow Up Steps
            // (0 → 1/5 → 2/5 → ... → 5/5)
            // -----------------------------
            case SLOW_UP_STEP:
                startSetpoint = (stepIndex - 1) * (maxRpm / 5.0);
                endSetpoint = stepIndex * (maxRpm / 5.0);
                break;
                
            // -----------------------------
            // Phase 1: Slow Down Steps
            // (N/5 → (N-1)/5)
            // -----------------------------
            case SLOW_DOWN_STEP:
                startSetpoint = shooter.getShooterRPM();
                endSetpoint = stepIndex * (maxRpm / 5.0);
                break;

            // -----------------------------
            // Phase 1: Fast Spike to MAX
            // -----------------------------
            case FAST_SPIKE_MAX:
                startSetpoint = 0;
                endSetpoint = maxRpm;
                break;

            // -----------------------------
            // Phase 2: Spike phases
            // -----------------------------
            case RECOVER1_SPIKE_MAX:
                startSetpoint = shooter.getShooterRPM();
                endSetpoint = maxRpm;
                break;

            case RECOVER2_SPIKE_4_5:
            case RECOVER3_SPIKE_4_5_REPEAT:
                startSetpoint = shooter.getShooterRPM();
                endSetpoint = maxRpm * 0.8;
                break;

            default:
                break;
        }

        SmartDashboard.putString("PIDTuner Phase", phase.toString());
    }

    /**
     * Computes the shooter RPM setpoint for the active phase.
     *
     * Behavior:
     * <ul>
     *   <li>Ramp phases interpolate linearly between start and end setpoints</li>
     *   <li>Spike phases immediately return the final setpoint</li>
     * </ul>
     *
     * @param t elapsed time since the phase began (seconds)
     * @return computed RPM setpoint for this timestamp
     */
    private double computeSetpoint(double t) {
        switch (phase) {

            case SLOW_UP_STEP:
            case SLOW_DOWN_STEP:
                double alpha = Math.min(1.0, t / slowRampSec);
                return startSetpoint + (endSetpoint - startSetpoint) * alpha;

            case FAST_SPIKE_MAX:
            case RECOVER1_SPIKE_MAX:
            case RECOVER2_SPIKE_4_5:
            case RECOVER3_SPIKE_4_5_REPEAT:
                return endSetpoint;

            default:
                return 0;
        }
    }

    /**
     * Determines whether the current phase has completed based on
     * elapsed time and phase type.
     *
     * @param t elapsed time since the phase began
     * @return true if the phase should end
     */
    private boolean phaseComplete(double t) {
        switch (phase) {

            case SLOW_UP_STEP:
            case SLOW_DOWN_STEP:
                return t >= slowRampSec + holdSec;

            case FAST_SPIKE_MAX:
            case RECOVER1_SPIKE_MAX:
            case RECOVER2_SPIKE_4_5:
            case RECOVER3_SPIKE_4_5_REPEAT:
                return t >= maxTestTimeSec;

            default:
                return false;
        }
    }

    /**
     * Advances the state machine to the next phase of the tuning sequence.
     * This method handles all transitions between step tests, spike tests,
     * and recovery cycles.
     */
    private void advancePhase() {
        switch (phase) {
            // ----------------------------- 
            // Phase 1: Slow Up Steps
            // -----------------------------
            case SLOW_UP_STEP:
                stepIndex++;
                if (stepIndex <= 5) {
                    configurePhase();
                } else {
                    // Finished 0→5/5, now do full slow down 5/5→0
                    stepIndex = 4;
                    minSlowDownStepIndex = 0;
                    nextPhaseAfterSlowDown = Phase.FAST_SPIKE_MAX;
                    phase = Phase.SLOW_DOWN_STEP;
                    configurePhase();
                }
                break;

            // -----------------------------
            // Phase 1: Slow Down Steps
            // -----------------------------
            case SLOW_DOWN_STEP:
                stepIndex--;
                if (stepIndex >= minSlowDownStepIndex) {
                    configurePhase();
                } else {
                    phase = nextPhaseAfterSlowDown;
                    configurePhase();
                }
                break;
            
            // -----------------------------
            // Phase 1: Fast Spike to MAX
            // -----------------------------
            case FAST_SPIKE_MAX:
                // After fast spike to MAX, slow down 5/5→1/5, then RECOVER1_SPIKE_MAX
                stepIndex = 5;
                minSlowDownStepIndex = 1;
                nextPhaseAfterSlowDown = Phase.RECOVER1_SPIKE_MAX;
                phase = Phase.SLOW_DOWN_STEP;
                configurePhase();
                break;

            // -----------------------------
            // Phase 2: Spike to MAX
            // -----------------------------
            case RECOVER1_SPIKE_MAX:
                // After fast spike to MAX, slow down 5/5→1/5, then RECOVER1_SPIKE_MAX
                stepIndex = 5;
                minSlowDownStepIndex = 1;
                nextPhaseAfterSlowDown = Phase.RECOVER2_SPIKE_4_5;
                phase = Phase.SLOW_DOWN_STEP;
                configurePhase();
                break;
            
            // -----------------------------
            // Phase 2: Spike to 4/5
            // -----------------------------
            case RECOVER2_SPIKE_4_5:
                stepIndex = 4;
                minSlowDownStepIndex = 1;
                nextPhaseAfterSlowDown = Phase.RECOVER3_SPIKE_4_5_REPEAT;
                phase = Phase.SLOW_DOWN_STEP;
                configurePhase();
                break;

            // -----------------------------
            // Phase 2: Spike to 4/5 (repeat)
            // -----------------------------
            case RECOVER3_SPIKE_4_5_REPEAT:
                stepIndex = 4;
                minSlowDownStepIndex = 0;
                nextPhaseAfterSlowDown = Phase.DONE;
                phase = Phase.SLOW_DOWN_STEP;
                configurePhase();
                break;

            // -----------------------------
            // Final slow drop to zero
            // -----------------------------
            case DONE:
                shooter.setTargetRPM(0); 
                fullReport.append("=== Shooter PID Tuning Test Complete ===\n");
                break;
                
            default:
                break;
        }
        SmartDashboard.putString("PIDTuner Phase", phase.toString());
    }

    // =====================================================================
    // Analysis and Reporting
    // =====================================================================
    
    /**
     * Analyzes the collected samples for the current phase and appends
     * a detailed report section including overshoot, rise time, settling
     * time, oscillation, and PID recommendations.
     */
    private void analyzeAndReport() {
        if (samples.isEmpty()) return;

        // ------------------------------------------------------------
        // Compute setpoint bounds for this phase
        // ------------------------------------------------------------
        double maxSetpoint = samples.stream()
            .mapToDouble(s -> s.setpoint)
            .max().orElse(endSetpoint);

        double minSetpoint = samples.stream()
            .mapToDouble(s -> s.setpoint)
            .min().orElse(endSetpoint);

        // Determine if this is a slow-up or slow-down phase
        boolean isSlowUp = endSetpoint > startSetpoint;

        // ------------------------------------------------------------
        // CATASTROPHIC FAILURE EXIT (only during SLOW_UP_STEP)
        // ------------------------------------------------------------
        if (phase == Phase.SLOW_UP_STEP) {

            double maxRpmSeenCat = samples.stream()
                .mapToDouble(s -> s.rpm)
                .max().orElse(0);

            double fracCat = maxRpmSeenCat / Math.max(maxSetpoint, 1);

            // Read current kV from motor config
            TalonFXConfiguration cfg = new TalonFXConfiguration();
            shooter.getShooterMotor().getConfigurator().refresh(cfg);
            double currentKV = cfg.Slot0.kV;

            // Compute recommended kV
            double recommendedKV = currentKV * (maxSetpoint / Math.max(maxRpmSeenCat, 1));

            if (fracCat  < 0.20) {
                appendCatastrophicFailure(
                    String.format(
                        "Shooter reached only %.0f%% of target. Suggested new kV = %.4f (current kV = %.4f).",
                        fracCat * 100,
                        recommendedKV,
                        currentKV
                    )
                );
                phase = Phase.DONE;
                return;
            }

            if (fracCat < 0.50) {
                appendCatastrophicFailure(
                    String.format(
                        "Shooter reached only %.0f%% of target. Suggested new kV = %.4f (current kV = %.4f).",
                        fracCat * 100,
                        recommendedKV,
                        currentKV
                    )
                );
                phase = Phase.DONE;
                return;
            }

            if (fracCat < 0.90) {
                appendCatastrophicFailure(
                    String.format(
                        "Shooter reached only %.0f%% of target. Suggested new kV = %.4f (current kV = %.4f).",
                        fracCat * 100,
                        recommendedKV,
                        currentKV
                    )
                );
                phase = Phase.DONE;
                return;
            }
        }

        // ------------------------------------------------------------
        // FAILURE CHECK: Did the shooter reach a meaningful fraction
        // of the target RPM?
        // ------------------------------------------------------------
        double maxRpmSeen = samples.stream()
            .mapToDouble(s -> s.rpm)
            .max().orElse(0);

        double frac = maxRpmSeen / Math.max(maxSetpoint, 1);

        // Hard failure: shooter cannot reach even 20% of target
        if (frac < 0.20) {
            appendFailureReport("Shooter reached only " + (int)(frac * 100) +
                "% of target. kV is far too low; increase kV by 5–10×.");
            return;
        }

        // Severe failure: shooter cannot reach 50% of target
        if (frac < 0.50) {
            appendFailureReport("Shooter reached only " + (int)(frac * 100) +
                "% of target. Increase kV significantly (3–5×) and retest.");
            return;
        }

        // Moderate failure: shooter cannot reach 90% of target
        if (frac < 0.90) {
            appendFailureReport("Shooter reached only " + (int)(frac * 100) +
                "% of target. Increase kV until shooter reaches at least 95%.");
            return;
        }

        // ------------------------------------------------------------
        // Compute overshoot / undershoot depending on phase direction
        // ------------------------------------------------------------
        double overshoot;

        if (isSlowUp) {
            overshoot = Math.max(0, maxRpmSeen - maxSetpoint);
        } else {
            // Slow-down: actual dips BELOW target
            double minRpmSeen = samples.stream()
                .mapToDouble(s -> s.rpm)
                .min().orElse(minSetpoint);

            overshoot = Math.max(0, minSetpoint - minRpmSeen);
        }

        // Track global max overshoot
        maxOvershootSeen = Math.max(maxOvershootSeen, overshoot);

        // ------------------------------------------------------------
        // Rise time (ignore first 200 ms to avoid ramp artifacts)
        // ------------------------------------------------------------
        double riseTime = -1;
        final double ignoreTime = 0.20; // seconds

        for (Sample s : samples) {
            if (s.t < ignoreTime) continue;
            if (Math.abs(s.rpm - maxSetpoint) <= rpmTolerance) {
                riseTime = s.t;
                break;
            }
        }

        if (riseTime >= 0) {
            totalRiseTime += riseTime;
            riseTimeCount++;
        }

        // ------------------------------------------------------------
        // Settling time (only after entering tolerance band)
        // ------------------------------------------------------------
        double settlingTime = -1;
        boolean everInBand = false;

        for (int i = 0; i < samples.size(); i++) {
            Sample s = samples.get(i);

            if (!everInBand &&
                Math.abs(s.rpm - maxSetpoint) <= rpmTolerance) {
                everInBand = true;
            }

            if (everInBand &&
                Math.abs(s.rpm - maxSetpoint) > rpmTolerance) {
                // left the band — not settled yet
                settlingTime = -1;
            }

            if (everInBand &&
                Math.abs(s.rpm - maxSetpoint) <= rpmTolerance) {
                // candidate settling time
                settlingTime = s.t;
            }
        }

        if (settlingTime >= 0) {
            totalSettlingTime += settlingTime;
            settlingTimeCount++;
        }

        // ------------------------------------------------------------
        // Oscillation detection (only after entering band)
        // ------------------------------------------------------------
        boolean inBand = false;
        int crossings = 0;
        Double prevErr = null;
        double firstInBand = 0;
        double lastCross = 0;

        for (Sample s : samples) {
            double err = maxSetpoint - s.rpm;

            if (!inBand && Math.abs(err) <= rpmTolerance) {
                inBand = true;
                firstInBand = s.t;
            }

            if (inBand) {
                if (prevErr != null &&
                    Math.signum(prevErr) != Math.signum(err)) {
                    crossings++;
                    lastCross = s.t;
                }
                prevErr = err;
            }
        }

        double oscDuration = (crossings > 0)
            ? (lastCross - firstInBand)
            : 0;

        // ------------------------------------------------------------
        // Initial spike metric (0–200ms, up & down)
        // ------------------------------------------------------------
        double maxUpSpike = 0;
        double maxDownSpike = 0;

        for (int i = 1; i < samples.size(); i++) {
            Sample prev = samples.get(i - 1);
            Sample curr = samples.get(i);

            double dt = curr.t - prev.t;
            if (dt <= 0) continue;

            double accel = (curr.rpm - prev.rpm) / dt;  // RPM per second

            // Only look at the first 200 ms of the phase
            if (curr.t < 0.20) {
                maxUpSpike = Math.max(maxUpSpike, accel);            
                maxDownSpike = Math.min(maxDownSpike, accel);
            }
        }

        double initialSpike = Math.max(maxUpSpike, Math.abs(maxDownSpike));

        // ------------------------------------------------------------
        // Correction spike metric (after entering band)
        // ------------------------------------------------------------ 
        double correctionSpike = computeMaxNegativeCorrectionSlope(samples, maxSetpoint);

        // ------------------------------------------------------------ 
        // PID suggestion (always) 
        // ------------------------------------------------------------ 
        String pidSuggestion = generatePidSuggestion(
            overshoot,
            riseTime,
            settlingTime,
            oscDuration,
            initialSpike,
            correctionSpike,
            maxSetpoint,
            phase
        );

        phaseSuggestions.add(pidSuggestion);

        // ------------------------------------------------------------ 
        // Feedforward suggestion + severity 
        // - Only for SLOW_UP_STEP and spike phases 
        // - For SLOW_DOWN_STEP: text only, no severity 
        // ------------------------------------------------------------ 
        
        // Steady-state fraction (last 20 samples) 
        int n = samples.size(); 
        int tail = Math.min(20, n); 
        double sumTail = 0; 
        for (int i = n - tail; i < n; i++) sumTail += samples.get(i).rpm;
        double steadyRpm = sumTail / tail;
        double steadyFrac = steadyRpm / Math.max(maxSetpoint, 1); 
        
        String ffSuggestion;

        phaseTypes.add(phase);
        
        if (phase == Phase.SLOW_DOWN_STEP) { 
            ffSuggestion = "Feedforward not evaluated during slow-down phases."; 
            // No severity, but still need a severity placeholder
            phaseFfSeverities.add(0.0);
        } else { 
            double ffSeverity = computeFfSeverity( 
                overshoot, 
                steadyFrac, 
                riseTime, 
                phase 
            ); 
            
            if (phaseFfSeverities == null) 
                phaseFfSeverities = new ArrayList<>(); 
            phaseFfSeverities.add(ffSeverity); 
            
            ffSuggestion = generateFfSuggestion( 
                samples, 
                maxSetpoint, 
                overshoot, 
                riseTime 
            ); 
        } 
        
        phaseFfSuggestions.add(ffSuggestion);

        // ------------------------------------------------------------
        // 7. Append report
        // ------------------------------------------------------------
        fullReport.append("=== Shooter PID Phase Report ===\n");
        fullReport.append("Phase: ").append(phase).append("\n");
        fullReport.append("  Target: ").append(maxSetpoint).append(" RPM\n");
        fullReport.append("  Overshoot: ").append(overshoot).append(" RPM\n");
        fullReport.append("  Rise time: ").append(riseTime).append(" s\n");
        fullReport.append("  Settling time: ").append(settlingTime).append(" s\n");
        fullReport.append("  Oscillation duration: ").append(oscDuration)
                .append(" s (").append(crossings).append(" crossings)\n");
        fullReport.append("  Initial spike: ").append(initialSpike).append(" RPM/s\n");
        fullReport.append("  Correction spike: ").append(correctionSpike).append(" RPM/s\n");
        fullReport.append(" PID Recommendation: ").append(pidSuggestion).append("\n");
        fullReport.append(" Feedforward Recommendation: ").append(ffSuggestion).append("\n");
        fullReport.append("--------------------------------\n\n");
    }

    /**
     * Measures how violently the RPM changes after entering the tolerance band.
     * This is meant to detect sharp correction spikes near the target, even when
     * overshoot is small or zero.
     *
     * @param samples phase samples
     * @param target  target RPM (maxSetpoint for this phase)
     * @return max |dRPM/dt| after entering band, or 0 if never in band
     */
    private double computeMaxNegativeCorrectionSlope(List<Sample> samples, double target) {
        if (samples.size() < 2) return 0.0;

        final double band = rpmTolerance;
        boolean inBand = false;
        double minSlope = 0.0; // most negative

        Sample prev = samples.get(0);

        for (int i = 1; i < samples.size(); i++) {
            Sample s = samples.get(i);

            if (!inBand && Math.abs(s.rpm - target) <= band) {
                inBand = true;
            }

            if (inBand) {
                double dt = s.t - prev.t;
                if (dt > 0) {
                    double slope = (s.rpm - prev.rpm) / dt;
                    minSlope = Math.min(minSlope, slope);
                }
            }

            prev = s;
        }

        return minSlope; // negative RPM/s
    }

    /**
     * Appends the final summary section to the report, including
     * aggregated metrics and PID configuration values.
     */
    private void appendFinalSummary() {
        fullReport.append("=== Final Summary ===\n");
    
        // Count phases (each analyzeAndReport call = one phase)
        int phaseCount = fullReport.toString().split("=== Shooter PID Phase Report ===").length - 1;
        fullReport.append("Total Phases: ").append(phaseCount).append("\n");
    
        // Total duration
        double totalDuration = Timer.getFPGATimestamp() - testStartTime;
        fullReport.append("Total Duration: ").append(totalDuration).append(" s\n\n");
    
        // Read PID values from the motor (Phoenix 6)
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        shooter.getShooterMotor().getConfigurator().refresh(cfg);

        fullReport.append("PID Values Used:\n");
        fullReport.append("  Kp: ").append(cfg.Slot0.kP).append("\n");
        fullReport.append("  Ki: ").append(cfg.Slot0.kI).append("\n");
        fullReport.append("  Kd: ").append(cfg.Slot0.kD).append("\n\n");

        fullReport.append("Feedforward Values Used:\n");
        fullReport.append("  kV: ").append(cfg.Slot0.kV).append("\n");
        fullReport.append("  kS: ").append(cfg.Slot0.kS).append("\n");
        fullReport.append("  kA: ").append(cfg.Slot0.kA).append("\n\n");
    
        fullReport.append("Overall Observations:\n");
        double maxRpmAchieved = samples.stream()
            .mapToDouble(s -> s.rpm)
            .max().orElse(0);

        fullReport.append("  - Max RPM achieved: ").append(maxRpmAchieved).append("\n");
        fullReport.append("  - Max overshoot observed: ").append(maxOvershootSeen).append(" RPM\n");
        fullReport.append("  - Average rise time: ")
          .append(riseTimeCount > 0 ? totalRiseTime / riseTimeCount : -1)
          .append(" s\n");
        fullReport.append("  - Average settling time: ")
          .append(settlingTimeCount > 0 ? totalSettlingTime / settlingTimeCount : -1)
          .append(" s\n");
        fullReport.append("  - Oscillation detected in 0 phases\n\n");
        

        fullReport.append("Overall PID Recommendation:\n");
        fullReport.append("  ").append(computeOverallPidRecommendation()).append("\n\n");

        fullReport.append("Overall Feedforward Recommendation:\n"); 
        fullReport.append(" ").append(computeOverallFfRecommendation()).append("\n\n");

        fullReport.append("=== End of Report ===\n");
    }

    /**
     * Writes the full tuning report to a timestamped file on the robot
     * (or local filesystem when running in simulation).
     */
    private void writeFullReportToFile() {
        // Build timestamp: YYYY-MM-DD_HH-MM-SS
        String timestamp = java.time.LocalDateTime.now()
            .format(java.time.format.DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss"));
    
        String filePath = edu.wpi.first.wpilibj.RobotBase.isReal()
            ? "/home/lvuser/shooter_pid_report_" + timestamp + ".txt"
            : "shooter_pid_report_" + timestamp + ".txt";
    
        System.out.println("Writing PID report to: " + filePath);
    
        try (FileWriter writer = new FileWriter(new File(filePath))) {
            writer.write(fullReport.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Generates a PID tuning recommendation based on overshoot,
     * rise time, settling time, and oscillation behavior.
     *
     * @param overshoot     measured overshoot in RPM
     * @param riseTime      time to enter tolerance band
     * @param settlingTime  time to remain within tolerance band
     * @param oscDuration   duration of oscillation in seconds
     * @param setpoint      target RPM
     * @param phase         the sequence phase
     * @return recommendation string for PID adjustment
     */
    private String generatePidSuggestion(
        double overshoot,
        double riseTime,
        double settlingTime,
        double oscDuration,
        double initialSpike,
        double correctionSpike,
        double setpoint,
        Phase phase
    ) {
        final double initialSpikeThreshold = setpoint * 2.5;
        final double correctionSpikeThreshold = 1500;

        // ------------------------------------------------------------
        // 1. Initial spike detection (FAST_SPIKE_MAX)
        // ------------------------------------------------------------
        if ((phase == Phase.FAST_SPIKE_MAX ||
            phase == Phase.RECOVER1_SPIKE_MAX ||
            phase == Phase.RECOVER2_SPIKE_4_5 ||
            phase == Phase.RECOVER3_SPIKE_4_5_REPEAT)
            && initialSpike > initialSpikeThreshold) {

            return "Transient spike detected; reduce Kp slightly or add Kd to soften the initial response.";
        }

        // ------------------------------------------------------------
        // 2. Correction spike detection
        // ------------------------------------------------------------
        if (correctionSpike > correctionSpikeThreshold) {
            return "Sharp correction spike near target detected; reduce Kp slightly or add Kd.";
        }

        // ------------------------------------------------------------
        // 3. Overshoot logic
        // ------------------------------------------------------------
        if (overshoot > setpoint * 0.02) {
            return "Moderate overshoot detected; reduce Kp slightly or add a touch of Kd.";
        }

        // ------------------------------------------------------------
        // 4. Rise time logic
        // ------------------------------------------------------------
        if (riseTime > 0 && riseTime > 0.8) {
            return "Rise time is slow; increase Kp by ~10%.";
        }

        // ------------------------------------------------------------
        // 5. Settling time logic
        // ------------------------------------------------------------
        if (settlingTime > 3.0) {
            return "Settling time is long; increase Kp or add a bit more Kd.";
        }

        // ------------------------------------------------------------
        // 6. Default
        // ------------------------------------------------------------
        return "Response looks stable. PID values appear well-tuned.";
    }

    private double computeFfSeverity(
        double overshoot,
        double steadyFrac,
        double riseTime,
        Phase phase
    ) {
        // Only evaluate FF on upward or spike phases
        if (phase == Phase.SLOW_DOWN_STEP) {
            return 0.0;
        }

        // --- kV TOO LOW: steady-state fraction < 0.95 ---
        if (steadyFrac < 0.95) {
            double severity = (0.95 - steadyFrac) * 10.0; // scale 0–5
            return +severity; // positive = increase kV
        }

        // --- kV TOO HIGH: overshoot grows with setpoint ---
        double overshootPct = overshoot / Math.max(1.0, steadyFrac * 1000.0);

        // If overshoot > 3% of setpoint → classic kV too high
        if (overshoot > 0 && overshootPct > 0.03) {
            return -2.0; // negative = decrease kV
        }

        // --- kA too high: fast spike overshoot ---
        if (phase == Phase.FAST_SPIKE_MAX || phase == Phase.RECOVER1_SPIKE_MAX) {
            if (overshoot > 20) {
                return -1.0; // reduce kA
            }
        }

        // Otherwise FF looks fine
        return 0.0;
    }

    /**
     * Generates a feedforward tuning recommendation based on
     * steady-state behavior, acceleration behavior, and voltage usage.
     *
     * Heuristics:
     *  - kS: handles static friction and low-speed hesitation
     *  - kV: handles steady-state voltage vs RPM
     *  - kA: handles acceleration voltage vs acceleration rate
     */
    private String generateFfSuggestion(
        List<Sample> samples,
        double setpoint,
        double overshoot,
        double riseTime
    ) {
        // Compute steady-state RPM
        int n = samples.size();
        int tail = Math.min(20, n);
        double sum = 0;
        for (int i = n - tail; i < n; i++) sum += samples.get(i).rpm;
        double steady = sum / tail;
        double steadyFrac = steady / Math.max(setpoint, 1);

        // --- kV too low ---
        if (steadyFrac < 0.95) {
            return String.format(
                "kV too low: shooter only reaches %.0f%% of target RPM. Increase kV.",
                steadyFrac * 100
            );
        }

        // --- kV too high: overshoot grows with setpoint ---
        double overshootPct = overshoot / Math.max(setpoint, 1);
        if (overshootPct > 0.03) {
            return "kV slightly too high: overshoot increases with setpoint. Reduce kV by ~3–5%.";
        }

        // --- kA too high ---
        if (overshoot > 20 && riseTime < 0.4) {
            return "Reduce kA: acceleration spike is too aggressive.";
        }

        return "Feedforward appears reasonable for this phase.";
    }

    /**
     * Computes an overall PID recommendation by analyzing the
     * frequency of per-phase suggestions and selecting the most
     * commonly occurring one.
     *
     * @return summarized PID recommendation for the entire test
     */
    private String computeOverallPidRecommendation() {
        int worstSeverity = -1;
        String worstMessage = "Response looks stable. PID values appear well-tuned.";

        for (String s : phaseSuggestions) {
            int sev = pidSeverity(s);
            if (sev > worstSeverity) {
                worstSeverity = sev;
                worstMessage = s;
            }
        }

        return worstMessage;
    }

    private int pidSeverity(String s) {
        if (s.contains("CATASTROPHIC")) return 100;
        if (s.contains("Overshoot is high")) return 90;
        if (s.contains("Moderate overshoot")) return 80;
        if (s.contains("Small overshoot")) return 70;
        if (s.contains("Rise time is slow")) return 60;
        if (s.contains("Settling time is long")) return 50;
        if (s.contains("Oscillation")) return 40;
        return 0;
    }

    /**
     * Computes an overall Feedforward recommendation by analyzing the
     * frequency of per-phase suggestions and selecting the most
     * commonly occurring one.
     *
     * @return summarized FF recommendation for the entire test
     */
    private String computeOverallFfRecommendation() {

        if (phaseFfSeverities == null || phaseFfSeverities.isEmpty()) {
            return "No feedforward recommendation available.";
        }

        double sum = 0;
        int count = 0;

        for (int i = 0; i < phaseFfSeverities.size(); i++) {
            Phase p = phaseTypes.get(i);

            // Ignore slow-down phases entirely
            if (p == Phase.SLOW_DOWN_STEP) continue;

            sum += phaseFfSeverities.get(i);
            count++;
        }

        if (count == 0) {
            return "Feedforward not evaluated (only slow-down phases detected).";
        }

        double avg = sum / count;

        if (avg > 0.5) return "Increase kV: shooter is running below target RPM.";
        if (avg < -0.5) return "Decrease kV: overshoot increases with setpoint.";
        if (avg < -0.1) return "Slightly reduce kV (~3–5%).";

        return "Feedforward appears reasonable; focus on PID tuning.";
    }

    private void appendCatastrophicFailure(String message) {
        double maxRpmSeen = samples.stream()
            .mapToDouble(s -> s.rpm)
            .max().orElse(0);

        phaseSuggestions.add(message);
        phaseFfSuggestions.add("Increase kV: shooter cannot reach target RPM.");

        fullReport.append("=== Shooter PID Phase Report ===\n");
        fullReport.append("Phase: ").append(phase).append("\n");
        fullReport.append("  Target: ").append(endSetpoint).append(" RPM\n");
        fullReport.append("  CATASTROPHIC FAILURE: Shooter cannot reach target RPM.\n");
        fullReport.append("  Max RPM achieved: ").append(maxRpmSeen).append(" RPM\n");
        fullReport.append("  Recommendation: ").append(message).append("\n");
        fullReport.append("  Feedforward Recommendation: Increase kV.\n");
        fullReport.append("--------------------------------\n\n");

        fullReport.append("=== TEST ABORTED EARLY DUE TO FAILURE ===\n");
    }

    private void appendFailureReport(String message) {
        double maxRpmSeen = samples.stream()
            .mapToDouble(s -> s.rpm)
            .max().orElse(0);

        phaseSuggestions.add(message);
        phaseFfSuggestions.add("Increase kV: shooter cannot reach target RPM.");

        fullReport.append("=== Shooter PID Phase Report ===\n");
        fullReport.append("Phase: ").append(phase).append("\n");
        fullReport.append("  Target: ").append(endSetpoint).append(" RPM\n");
        fullReport.append("  ERROR: Shooter failed to reach target RPM.\n");
        fullReport.append("  Max RPM achieved: ").append(maxRpmSeen).append(" RPM\n");
        fullReport.append("  Recommendation: ").append(message).append("\n");
        fullReport.append("  Feedforward Recommendation: Increase kV.\n");
        fullReport.append("--------------------------------\n\n");
    }
}