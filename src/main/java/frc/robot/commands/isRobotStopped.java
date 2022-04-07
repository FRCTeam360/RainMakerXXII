// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class isRobotStopped extends CommandBase {
  private final DriveTrain mDriveTrain = DriveTrain.getInstance();
  /** Creates a new isRobotStopped. */
  public isRobotStopped() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("DriveTrain speed is: " + mDriveTrain.getLeftEncoderMetersPerSec());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveTrain has been stopped.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double minDriveTrainSpeed = 0.1;
    return Math.abs(mDriveTrain.getLeftEncoderMetersPerSec()) < minDriveTrainSpeed && Math.abs(mDriveTrain.getRightEncoderMetersPerSec()) < minDriveTrainSpeed;
  }
}
