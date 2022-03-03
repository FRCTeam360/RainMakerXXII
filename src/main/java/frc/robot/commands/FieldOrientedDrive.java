// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class FieldOrientedDrive extends CommandBase {
  private static final String addRequirements = null;

  private final DriveTrain myDriveTrain;

  private final DriverControl driverCont; // driverCont?

  private final SlewRateLimiter filter;

  /** Creates a new FieldOrientedDrive. */
  public FieldOrientedDrive(DriveTrain driveTrain) {
    driverCont = DriverControl.getInstance();

    myDriveTrain = driveTrain;

    filter = new SlewRateLimiter(myDriveTrain.getAccelerationLimit());

    addRequirements(myDriveTrain); // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double gyroAngle = myDriveTrain.getHeadingAngle();
    double gyroRadians = Math.toRadians(gyroAngle);

    double rightLeftSquared = 0;
    double upDownSquared = 0;
    double driveRight = 0;
    double driveLeft = 0;

    // sets doubled right/left value of xboxController
    if (Math.abs(driverCont.getLeftX()) >= xboxDeadzone) {
      rightLeftSquared = driverCont.getLeftX() * driverCont.getLeftX();
      if (driverCont.getLeftX() < 0) {
        rightLeftSquared = rightLeftSquared * -1;
      }
    }
    // sets doubled up/down value of xboxController
    if (Math.abs(driverCont.getLeftY()) >= xboxDeadzone) {
      upDownSquared = -1 * driverCont.getLeftY() * driverCont.getLeftY();
      if (driverCont.getLeftY() < 0) {
        upDownSquared = upDownSquared * -1;
      }
    }

    // field oriented drive conversion. forward = robot-based forward value, right =
    // robot-based turning adjustment
    double forward = upDownSquared * Math.cos(gyroRadians) + rightLeftSquared * Math.sin(gyroRadians);
    // if(myDriveTrain.isAccelerating()){
      forward = filter.calculate(forward);
    // }
    double right = -1 * upDownSquared * Math.sin(gyroRadians) + rightLeftSquared * Math.cos(gyroRadians);

    // System.out.println("forward: " + forward);
    // System.out.println("right: " + right);

    // sets drive values using previous values for right/left and forward/back
    driveLeft = forward + right;
    driveRight = forward - right;

    // ensures motors are not passed value greater than 1 or less than -1
    driveLeft = Math.min(driveLeft, 1);
    driveRight = Math.min(driveRight, 1);
    driveLeft = Math.max(driveLeft, -1);
    driveRight = Math.max(driveRight, -1);

    // drive reversed if bumper held
    if (driverCont.getLeftStickButton()) {
      myDriveTrain.drive(driveRight * 0.5, driveLeft * 0.5);
    } else {
      myDriveTrain.drive(driveLeft * 0.5, driveRight * 0.5);
    }

    // double contRadians = Math.atan2(driverCont.getY(getLeftX),
    // driverCont.getX(Hand.kRight)); //arctan of stick inputs for radians
    // double lStickAngle = Math.toDegrees(contRadians); //radians to degrees

    if (driverCont.getDPadUp()) {
      myDriveTrain.resetEncPos(); // reset angle when Y pressed
    }

    // _________rotation control_____________

    // double rotationRight = -1 * driverCont.getY(Hand.kRight) *
    // Math.sin(gyroRadians) + driverCont.getX(Hand.kRight) * Math.cos(gyroRadians);

    // rotate based on right stick
    if (Math.abs(driverCont.getRightX()) >= xboxDeadzone) {
      double turn = driverCont.getRightX() * 0.3;
      myDriveTrain.drive(turn, -turn);
    }

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
