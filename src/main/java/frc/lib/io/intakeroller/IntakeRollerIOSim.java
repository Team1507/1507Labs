//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakeroller;

import edu.wpi.first.wpilibj.Timer;

/**
 * Simulation implementation of IntakeRollerIO.
 */
public class IntakeRollerIOSim implements IntakeRollerIO {

    // Sim state
    private double dutyCycle = 0.0;
    private double rollerSpeedRps = 0.0; // simulated speed

    // Constants for simple physics
    private static final double MAX_SPEED_RPS = 80.0;   // arbitrary but reasonable
    private static final double SPEED_RAMP = 40.0;      // how fast it accelerates (RPS/sec)
    private static final double BASE_CURRENT = 2.0;      // idle current
    private static final double LOAD_CURRENT = 18.0;     // additional current at full speed
    private static final double TEMP_RISE_RATE = 0.05;   // deg C per second at full load
    private static final double COOL_RATE = 0.02;        // deg C per second cooling

    private double temperatureC = 30.0;
    private double lastTime = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(IntakeRollerInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        // --- Speed simulation ---
        double targetSpeed = dutyCycle * MAX_SPEED_RPS;

        // Ramp speed toward target
        double delta = targetSpeed - rollerSpeedRps;
        double maxDelta = SPEED_RAMP * dt;
        delta = Math.max(-maxDelta, Math.min(maxDelta, delta));
        rollerSpeedRps += delta;

        // --- Current simulation ---
        double loadFactor = Math.abs(rollerSpeedRps) / MAX_SPEED_RPS;
        double current = BASE_CURRENT + loadFactor * LOAD_CURRENT;

        // --- Temperature simulation ---
        if (Math.abs(rollerSpeedRps) > 1e-3) {
            temperatureC += TEMP_RISE_RATE * loadFactor * dt;
        } else {
            temperatureC -= COOL_RATE * dt;
        }
        temperatureC = Math.max(20.0, temperatureC); // don't go below room temp

        // --- Fill inputs ---
        inputs.dutyCycle = dutyCycle;
        inputs.currentA = current;
        inputs.temperatureC = temperatureC;
    }

    @Override
    public void runDuty(double duty) {
        dutyCycle = duty;
    }

    @Override
    public void runPower(double power) {
        dutyCycle = power; // same behavior in sim
    }

    @Override
    public void stop() {
        dutyCycle = 0.0;
    }

    @Override
    public void increaseSpeed(double rollerSpeed) {
        dutyCycle += 0.1;
    }

    @Override
    public  void resetSpeed(double rollerSpeed) {
        dutyCycle = 0.3;
    }

    
}
