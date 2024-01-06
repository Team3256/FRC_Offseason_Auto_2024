package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.DriveSubsystem;

public class Stop extends Command {
    private final DriveSubsystem driveSubsystem;

  public Stop(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.drive(0,0,0,true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
