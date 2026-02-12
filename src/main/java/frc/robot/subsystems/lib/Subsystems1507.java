//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems.lib;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.MotorConfig;

public abstract class Subsystems1507 extends SubsystemBase {
   
    protected void configureFXMotor(MotorConfig config, TalonFX motor) {
        
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = config.KP();
        cfg.Slot0.kI = config.KI();
        cfg.Slot0.kD = config.KD();

        cfg.Slot0.kV = config.KV();
        cfg.Slot0.kS = config.KS();
        cfg.Slot0.kA = config.KA();

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(config.peakForwardVoltage()))
            .withPeakReverseVoltage(Volts.of(config.peakReverseVoltage()));

        motor.getConfigurator().apply(cfg);
    }

    protected void configureFXSMotor(MotorConfig config, TalonFXS motor) {
    
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();

        cfg.Slot0.kP = config.KP();
        cfg.Slot0.kI = config.KI();
        cfg.Slot0.kD = config.KD();

        cfg.Slot0.kV = config.KV();
        cfg.Slot0.kS = config.KS();
        cfg.Slot0.kA = config.KA();

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(config.peakForwardVoltage()))
            .withPeakReverseVoltage(Volts.of(config.peakReverseVoltage()));
        motor.getConfigurator().apply(cfg);
    }

}