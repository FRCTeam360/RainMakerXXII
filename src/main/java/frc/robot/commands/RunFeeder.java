// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends CommandBase {
  private Feeder myFeeder;
  private final DriverControl drive;
  /** Creates a new RunFeeder. */
  public RunFeeder() {
    myFeeder = Feeder.getInstance();
    drive = DriverControl.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(drive.getLeftTrigger()) {
      myFeeder.runFeeder(1.0);
      if(drive.getYButton()) {
        myFeeder.runFeeder(-1.0);
      }
    } else { 
      myFeeder.stopFeeder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
