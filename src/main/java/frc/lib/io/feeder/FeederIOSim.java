//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.feeder;

/**
 * Simulation implementation of FeederIO.
 */
public class FeederIOSim implements FeederIO {

    private double rpm = 0.0;
    private double duty = 0.0;

    @Override
    public void updateInputs(FeederInputs inputs) {
        inputs.velocityRPM = rpm;
        inputs.motorRPS = rpm / 60.0;
        inputs.currentA = Math.abs(duty) * 5.0;
        inputs.temperatureC = 25.0;
    }

    @Override
    public void runRPM(double rpm) {
        this.rpm = rpm;
    }

    @Override
    public void runDuty(double duty) {
        this.duty = duty;
        this.rpm = duty * 500; // fake model
    }

    @Override
    public void setVoltage(double voltage) {
        this.rpm = voltage * 50; // fake model
    }

    @Override
    public void stop() {
        this.rpm = 0.0;
        this.duty = 0.0;
    }
}
