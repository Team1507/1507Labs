//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.framework;

import frc.robot.subsystems.*;

public record SubsystemsRecord(
    SwerveSubsystem swerve,
    AgitatorSubsystem agitator,
    ClimberSubsystem climber,
    FeederSubsystem BLUfeeder,
    FeederSubsystem YELfeeder,
    HopperSubsystem hopper,
    IntakeArmSubsystem intakeArm,
    IntakeRollerSubsystem intakeRoller,
    ShooterSubsystem BLUshooter,
    ShooterSubsystem YELshooter
) {}
