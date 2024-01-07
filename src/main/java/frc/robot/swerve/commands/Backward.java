package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.DriveSubsystem;

public class Backward extends Command {
    private final DriveSubsystem driveSubsystem;

  public Backward(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.drive(-1,0,0,true);
    System.out.println("Backward init");
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Backward finished");
  }
}
