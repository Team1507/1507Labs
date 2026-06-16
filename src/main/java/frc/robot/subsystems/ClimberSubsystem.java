//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.climber.ClimberIO;
import frc.lib.io.climber.ClimberInputs;

/**
 * Thin, IO-based climber subsystem.
 */
public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberInputs inputs = new ClimberInputs();

    private double targetPosition = 0.0;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setPosition(double position) {
        targetPosition = position;
        io.setPosition(position);
    }

    public void setServo(double position) {
        io.setServo(position);
    }

    public boolean isAtPosition() {
        double tol = 1.0;
        return Math.abs(inputs.position - targetPosition) <= tol;
    }

    public ClimberInputs getInputs() {
        return inputs;
    }

    public double getPosition() {
        return inputs.position;
    }

    public boolean getLimitSwitch() {
        return inputs.limitSwitch;
    }
}
