// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class RunFeederAutomatically extends CommandBase {

  Shooter myShooter = Shooter.getInstance();
  Feeder myFeeder = Feeder.getInstance();
  Tower myTower = Tower.getInstance();
  DriverControl driverCont = DriverControl.getInstance();
  OperatorControl operatorCont = OperatorControl.getInstance();

  private boolean isInAuto = false;

  /** Creates a new runFeederWithSensor. */
  public RunFeederAutomatically() {

    addRequirements(myFeeder);

    this.isInAuto = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public RunFeederAutomatically(boolean isInAuto) {
    addRequirements(myFeeder);
    this.isInAuto = isInAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isInAuto && myShooter.isAtSpeed()) {
      myFeeder.runFeeder(1);
    } else if (driverCont.getLeftTrigger() || operatorCont.getLeftTrigger() || isInAuto) {
      if (myTower.ballNotInBottom()) {
        myFeeder.runFeeder(1);
      } else {
        myFeeder.runFeeder(0);
      }
    } else {
      myFeeder.runFeeder(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myFeeder.runFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
