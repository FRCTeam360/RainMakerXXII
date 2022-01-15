// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveTrain;


import static frc.robot.Constants.OIConstants.*;

public class ArcadeDrive extends CommandBase {

  private final DriveTrain myDriveTrain;

  private final XboxController driverCont;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DriveTrain driveTrain) {
    driverCont = new XboxController(driverContPort);

    myDriveTrain = driveTrain;


    addRequirements(myDriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightLeftSquared = 0;
    double upDownSquared = 0;
    double driveRight = 0;
    double driveLeft = 0;

    if(Math.abs(driverCont.getRightY()) >= xboxDeadzone) {
      rightLeftSquared = driverCont.getRightY() * driverCont.getRightY();
      if(driverCont.getLeftX() < 0){
        upDownSquared = upDownSquared * -1;
      }
    }


    driveLeft = upDownSquared + rightLeftSquared;
    driveRight = upDownSquared - rightLeftSquared;

    driveLeft = Math.min(driveLeft, 1);
    driveRight = Math.min(driveRight, 1);
    driveLeft = Math.max(driveLeft, -1);
    driveRight = Math.max(driveRight, -1);

    myDriveTrain.driveL(driveLeft * 1.0);
    myDriveTrain.driveR(driveRight * 1.0);
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