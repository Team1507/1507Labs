//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import static org.team1507.robot.Constants.kShooter.*;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.Constants.kShooter;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends Subsystem1507 {

    private final Motor1507 bluMotor;
    private final Motor1507 yelMotor;
    private final BaseStatusSignal[] shooterSignals;
    private final DigitalInput bluBallSensor;
    private final DigitalInput yelBallSensor;

    private double targetRPM = 0.0;

    public Shooter() {
        super("Shooter");

        bluMotor = new Motor1507(key("BLU"), Motor1507.Type.FX, RobotMap.SHOOTER_BLU, BLU_CONFIG);
        yelMotor = new Motor1507(key("YEL"), Motor1507.Type.FX, RobotMap.SHOOTER_YEL, YEL_CONFIG);

        BaseStatusSignal[] blu = bluMotor.getSignals();
        BaseStatusSignal[] yel = yelMotor.getSignals();
        shooterSignals = new BaseStatusSignal[blu.length + yel.length];
        System.arraycopy(blu, 0, shooterSignals, 0, blu.length);
        System.arraycopy(yel, 0, shooterSignals, blu.length, yel.length);

        bluBallSensor = new DigitalInput(RobotMap.SHOOTER_BLU_DIO);
        yelBallSensor = new DigitalInput(RobotMap.SHOOTER_YEL_DIO);
    }

    // =========================================================================
    // Control
    // =========================================================================

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        bluMotor.setVelocityRPS(rpm / 60.0);
        yelMotor.setVelocityRPS(rpm / 60.0);
    }

    public void stop() {
        targetRPM = 0.0;
        bluMotor.stop();
        yelMotor.stop();
    }

    // =========================================================================
    // State
    // =========================================================================

    public double getBluRPM() {
        return bluMotor.getRotorVelocity() * 60.0;
    }

    public double getYelRPM() {
        return yelMotor.getRotorVelocity() * 60.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isBluBallPresent() {
        if (RobotBase.isSimulation()) return false;
        return !bluBallSensor.get();
    }

    public boolean isYelBallPresent() {
        if (RobotBase.isSimulation()) return false;
        return !yelBallSensor.get();
    }

    public boolean atVelocity() {
        if (targetRPM == 0.0) return false;
        return Math.abs(targetRPM - getBluRPM()) <= TOLERANCE_RPM
            && Math.abs(targetRPM - getYelRPM()) <= TOLERANCE_RPM;
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void simulationPeriodic() {
        bluMotor.simulationPeriodic(0.02);
        yelMotor.simulationPeriodic(0.02);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(shooterSignals);

        log("BLU/RPM",         getBluRPM());
        log("YEL/RPM",         getYelRPM());
        log("TargetRPM",       targetRPM);
        log("AtVelocity",      atVelocity());
        log("BLU/StatorAmps",  bluMotor.getStatorCurrent());
        log("YEL/StatorAmps",  yelMotor.getStatorCurrent());
        log("BLU/BallPresent", isBluBallPresent());
        log("YEL/BallPresent", isYelBallPresent());
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /** Spins both flywheels to the given RPM and holds until interrupted. */
    public Command spinUpCommand(double rpm) {
        return new CommandBuilder(this)
            .named("shooter.spinUp")
            .onInitialize(() -> setTargetRPM(rpm))
            .onExecute(() -> setTargetRPM(rpm))
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Sets target RPM and finishes immediately — for use inside command sequences. */
    public Command setRPMCommand(double rpm) {
        return new CommandBuilder(this)
            .named("shooter.setRPM")
            .onInitialize(() -> setTargetRPM(rpm))
            .isFinished(true);
    }

    /** Holds flywheels at idle speed to keep them warm. Stops on interrupt. */
    public Command idleCommand() {
        return new CommandBuilder(this)
            .named("shooter.idle")
            .onExecute(() -> setTargetRPM(kShooter.IDLE_RPM))
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Stops both flywheels immediately. */
    public Command stopCommand() {
        return new CommandBuilder(this)
            .named("shooter.stop")
            .onInitialize(this::stop)
            .isFinished(true);
    }
}
