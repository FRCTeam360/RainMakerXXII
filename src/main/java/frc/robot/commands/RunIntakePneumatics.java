// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePneumatics;

public class RunIntakePneumatics extends CommandBase {
  Boolean isExtending;
  IntakePneumatics myIntakePneumatics = IntakePneumatics.getInstance();
  /** Creates a new RunIntakePneumatics. */
  public RunIntakePneumatics(Boolean isExtendingBoolean) {
      isExtending = isExtendingBoolean;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isExtending){
      myIntakePneumatics.intakeExtension();
    }
    else{
      myIntakePneumatics.intakeRectraction();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
