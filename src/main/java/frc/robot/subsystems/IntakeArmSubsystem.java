//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.intakearm.IntakeArmMotorIO;
import frc.lib.io.intakearm.IntakeArmMotorInputs;
import frc.robot.Constants.kIntake;
/**
 * Thin, IO-based intake arm subsystem.
 */
public class IntakeArmSubsystem extends SubsystemBase {

    private final IntakeArmMotorIO io;
    private final IntakeArmMotorInputs inputs = new IntakeArmMotorInputs();

    public IntakeArmSubsystem(IntakeArmMotorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setAngle(double degrees) {
        double safeDeg = Math.min(degrees, kIntake.kArm.MAX_ANGLE_DEGREES);
        io.setAngle(safeDeg);
    }

    public IntakeArmMotorInputs getInputs() {
        return inputs;
    }

    public double getBLUPositionDegrees() {
        return inputs.bluPositionDeg;
    }

    public double getYELPositionDegrees() {
        return inputs.yelPositionDeg;
    }

    public boolean getBLUReverseLimit() {
        return inputs.bluReverseLimit;
    }

    public boolean getYELReverseLimit() {
        return inputs.yelReverseLimit;
    }

    public boolean isAtPosition(double targetDeg, double toleranceDeg) {
        return Math.abs(inputs.bluPositionDeg - targetDeg) < toleranceDeg &&
               Math.abs(inputs.yelPositionDeg - targetDeg) < toleranceDeg;
    }
    public void runPower(double power) {
        double safePower = power;

        if((inputs.bluPositionDeg > kIntake.kArm.MAX_ANGLE_DEGREES)|| (inputs.yelPositionDeg > kIntake.kArm.MAX_ANGLE_DEGREES) && power > 0) {
            safePower = 0;
        }
        if(safePower > 0.5) {
            io.runPower(kIntake.kArm.MANUAL_POSITIVE_POWER);
        }
        else if(safePower< -0.5){
            io.runPower(kIntake.kArm.MANUAL_NEGATIVE_POWER);
        }
        else {
            io.runPower(0);
        }
    }
    public void stop() {
        io.stop();
    }
}
