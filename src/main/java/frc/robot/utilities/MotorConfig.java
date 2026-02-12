package frc.robot.utilities;

import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.mechanics.GearRatio;


public record MotorConfig(
        int CAN_ID,
        
        double KP,
        double KI, 
        double KD, 

        double KV, 
        double KS,  
        double KA,

        double peakForwardVoltage,
        double peakReverseVoltage,
        GearRatio ratio,
        Transform2d robotToMechanism
) {

    public MotorConfig(
        int CAN_ID, 
        double KP, double KI, double KD, 
        double KV, double KS, double KA,
        double peakForwardVoltage,
        double peakReverseVoltage,
        GearRatio ratio
    ) {
        this(CAN_ID, KP, KI, KD, KV, KS, KA,
        peakForwardVoltage, peakReverseVoltage,
        ratio,
        new Transform2d());
    }
}
