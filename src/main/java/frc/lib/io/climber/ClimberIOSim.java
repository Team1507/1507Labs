//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.climber;

/**
 * Simulation implementation of ClimberIO.
 */
public class ClimberIOSim implements ClimberIO {

    private double position = 0.0;
    private double servoPos = 0.0;

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.position = position;
        inputs.motorPosition = position; // 1:1 in sim
        inputs.currentA = Math.abs(position) * 2.0;
        inputs.temperatureC = 25.0;
        inputs.limitSwitch = position <= 0.0;
        inputs.servoPosition = servoPos;
    }

    @Override
    public void setPosition(double mechanismPosition) {
        this.position = mechanismPosition;
    }

    @Override
    public void setServo(double position) {
        this.servoPos = position;
    }

    @Override
    public void stop() {
        // no-op for sim
    }
}
