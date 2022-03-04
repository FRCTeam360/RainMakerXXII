// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.filter.SlewRateLimiter;

import static frc.robot.Constants.OIConstants.*;

public class ArcadeDrive extends CommandBase {
  private final DriveTrain myDriveTrain;

  private final DriverControl driverCont;

  private final SlewRateLimiter filter;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DriveTrain driveTrain) {
    driverCont = DriverControl.getInstance();

    myDriveTrain = driveTrain;

    filter = new SlewRateLimiter(myDriveTrain.getAccelerationLimit());

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turn = 0;
    double forward = 0;
    double driveRight = 0;
    double driveLeft = 0;

    if (Math.abs(driverCont.getLeftY()) >= xboxDeadzone) {
      forward = driverCont.getLeftY() * driverCont.getLeftY();
      if (driverCont.getLeftY() < 0) {
        forward = forward * -1;
      }
    }
    if (Math.abs(driverCont.getLeftX()) >= xboxDeadzone) {
      turn = driverCont.getLeftX() * driverCont.getLeftX();
      if (driverCont.getLeftX() < 0) {
        turn = turn * -1;
      }
    }

    forward = -forward;
    // if (myDriveTrain.isAccelerating()) {
      forward = filter.calculate(forward);
    // }
    // turn = turn * -1;

    driveLeft = forward + turn;
    driveRight = forward - turn;

    driveLeft = Math.min(driveLeft, 1);
    driveRight = Math.min(driveRight, 1);
    driveLeft = Math.max(driveLeft, -1);
    driveRight = Math.max(driveRight, -1);

    myDriveTrain.drive(driveLeft, driveRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myDriveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
