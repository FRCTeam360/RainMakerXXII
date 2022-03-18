// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Tower;

public class RunTowerManually extends CommandBase {
  private final Tower myTower;
  private final OperatorControl operatorCont;
  private final DriverControl driverCont;
  /** Creates a new RunFeederManually. */
  public RunTowerManually() {
    myTower = Tower.getInstance();
    operatorCont = OperatorControl.getInstance();
    driverCont = DriverControl.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operatorCont.getXButton() || driverCont.getXButton()) {
      myTower.runTower(-1.0);
    } else {
      myTower.runTower(1.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myTower.runTower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
