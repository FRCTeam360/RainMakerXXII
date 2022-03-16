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

public class RunTowerAutomatically extends CommandBase {

  Shooter myShooter = Shooter.getInstance();
  Feeder myFeeder = Feeder.getInstance();
  Tower myTower = Tower.getInstance();
  DriverControl driverCont = DriverControl.getInstance();
  OperatorControl operatorCont = OperatorControl.getInstance();
  /** Creates a new runFeederWithSensor. */
  public RunTowerAutomatically() {

    addRequirements(myTower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(myShooter.isAtSpeed()) {
      myTower.runTower(1);
    } else if(driverCont.getLeftTrigger() || operatorCont.getLeftTrigger()){
      if(myTower.ballNotInTower()) {
        myTower.runTower(1);
      }
    } else {
      myTower.runTower(0);
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
