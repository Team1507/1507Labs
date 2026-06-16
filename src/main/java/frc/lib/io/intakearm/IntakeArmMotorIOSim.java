package frc.lib.io.intakearm;

import frc.lib.core.math.GearRatio;

/**
 * Simple simulation for a single intake arm motor.
 */
public class IntakeArmMotorIOSim {

    private final GearRatio ratio;
    private double motorRot = 0.0;

    public IntakeArmMotorIOSim(GearRatio ratio) {
        this.ratio = ratio;
    }

    public void updateInputs(IntakeArmMotorInputs inputs) {
        inputs.motor.motorRot = motorRot;
        inputs.positionDeg = ratio.sensorToReal(motorRot);
        inputs.motor.tempC = 25.0;
        inputs.motor.currentA = 0.0;
        inputs.reverseLimit = false;
    }

    public void setAngle(double degrees) {
        motorRot = ratio.realToSensor(degrees);
    }

    public void stop() {}
}
