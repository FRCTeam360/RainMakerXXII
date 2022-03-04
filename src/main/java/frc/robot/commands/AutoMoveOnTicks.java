// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoMoveOnTicks extends CommandBase {

  private DriveTrain myDriveTrain;
  private double target;
  private double distance;
  /** Creates a new AutoMoveOnTicks. */
  public AutoMoveOnTicks(DriveTrain driveTrain, double meters) {

    myDriveTrain = driveTrain;
    target = meters;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    myDriveTrain.drive(0.125, 0.125);
    distance = (myDriveTrain.getLeftEncoderMeters() + myDriveTrain.getRightEncoderMeters()) / 2;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance >= target;
  }
}
