// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.operatorInterface.DriverControl;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.OIConstants.*;

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {

  private final DriveTrain myDriveTrain;

  private final DriverControl driverCont;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDrive() {
    myDriveTrain = DriveTrain.getInstance();
    driverCont = DriverControl.getInstance();
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
    double leftMotorPercentage = 0;
    double rightMotorPercentage = 0;

    // create deadzone and look into new documentation
    if (Math.abs(driverCont.getRightY()) >= xboxDeadzone) {
      rightMotorPercentage = -1 * driverCont.getRightY();
    }

    if (Math.abs(driverCont.getLeftY()) >= xboxDeadzone) {
      leftMotorPercentage = -1 * driverCont.getLeftY();
    }

    myDriveTrain.drive(leftMotorPercentage, rightMotorPercentage);
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
