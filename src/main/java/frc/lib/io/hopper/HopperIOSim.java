// //  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
// //  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
// //  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
// //  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
// //  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
// //   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
// //                           TEAM 1507 WARLOCKS

package frc.lib.io.hopper;

import frc.lib.core.math.GearRatio;
import frc.lib.hardware.HopperHardware;
import frc.robot.Constants.kHopper;

/**
 * Simplistic simulation of HopperIO.
 * Instantly moves to commanded positions and reports hopperExtended.
 */
public class HopperIOSim implements HopperIO {

    private static final double LOOP_PERIOD_SEC = 0.02; // 20 ms
    private static final double MAX_SPEED_IN_PER_SEC = 8.0; // tune this

    private final GearRatio ratio = HopperHardware.RATIO;

    // Simulated state
    private double positionInches = 0.0;
    private double targetInches = 0.0;

    @Override
    public void updateInputs(HopperInputs inputs) {
        // Move toward target at fixed speed
        double error = targetInches - positionInches;
        double maxStep = MAX_SPEED_IN_PER_SEC * LOOP_PERIOD_SEC;

        if (Math.abs(error) <= maxStep) {
            positionInches = targetInches;
        } else {
            positionInches += Math.copySign(maxStep, error);
        }

        // Publish state
        inputs.position = positionInches;
        inputs.motorRot = ratio.realToSensor(positionInches);

        inputs.currentA = 0.0;
        inputs.temperatureC = 25.0;

        inputs.hopperExtended = inputs.position >= kHopper.EXTENDED_POS;
    }

    @Override
    public void setPosition(double position) {
        // In real code this is inches, not degrees
        targetInches = position;
    }

    @Override
    public boolean getMagSensor() {
        return false; // no homing in sim
    }

    @Override
    public void runPower(double power) {
        // Optional: simulate manual power mode
        double speed = power * MAX_SPEED_IN_PER_SEC;
        targetInches = positionInches + speed * LOOP_PERIOD_SEC;
    }

    @Override
    public void hopperStop() {
        targetInches = positionInches;
    }
}