//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.hopper.HopperIO;
import frc.lib.io.hopper.HopperInputs;
import frc.robot.Constants.kHopper;

/**
 * Thin, IO-based hopper subsystem.
 */
public class HopperSubsystem extends SubsystemBase {

    private final HopperIO io;
    private final HopperInputs inputs = new HopperInputs();

    public HopperSubsystem(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setPosition(double position) {
        double safePos = Math.min(position, kHopper.MAX_POS);
        io.setPosition(safePos);
    }

    public HopperInputs getInputs() {
        return inputs;
    }

    public double getPositionDegrees() {
        return inputs.position;
    }

    public boolean isHopperSafeForIntake() {
        return inputs.position > kHopper.SAFE_EXTENDED;
    }

    public boolean isHopperFullyExtended(){
        return inputs.hopperExtended;
    }

    public void runPower(double power)
    {
        double safePower = power;

        if((inputs.position > kHopper.MAX_POS) && power < 0) {
            safePower = 0;
        }
        if(safePower > 0.5) {
            io.runPower(kHopper.MANUAL_POSITIVE_POWER);
        }
        else {
            io.runPower(0);
        }
    }
    public void stop() {
        io.hopperStop();
    }
    public void MagnetOffset (){
        
    }
}
