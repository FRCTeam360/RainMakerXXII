// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//used to be named RunFeeder.java 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.operatorInterface.*;

public class RunFeeder extends CommandBase {

  private final Tower myTower;
  private final Feeder myFeeder;
  private final Shooter myShooter;
  private final OperatorControl operatorCont;
  private final DriverControl driverCont;

  private double towerPower = 0;
  private double feederPower = 0;

  public RunFeeder() {
    operatorCont = OperatorControl.getInstance();
    driverCont = DriverControl.getInstance();

    myShooter = Shooter.getInstance();
    myTower = Tower.getInstance();
    myFeeder = Feeder.getInstance();

    addRequirements(myTower, myFeeder);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if shooter at speed, run all
    // if (myShooter.isAtSpeed()) {
    //   towerPower = 1.0;
    //   feederPower = 1.0;

    //   // if intake running, run shooter if no ball at top of tower
    // } else 
    // if (driverCont.getLeftTrigger()) {
    //   // if (myTower.ballNotInTower()) {
    //     towerPower = 1.0;
    //     feederPower = 0.0;
    //   // } else {
    //   //   towerPower = 0.0;
    //   //   feederPower = 0.0;
    //   // }
    // } else {

      // manually run feeder
      if (operatorCont.getLeftTrigger() || driverCont.getLeftBumper()) {
        if (operatorCont.getXButton() || driverCont.getXButton()) {
          feederPower = -1.0;
        } else {
          feederPower = 0.5;
        }
      } else {
        feederPower = 0.0;
      }

      // manually run tower;
      if (operatorCont.getRightTrigger() || driverCont.getRightBumper()) {
        if (operatorCont.getXButton() || driverCont.getXButton()) {
          towerPower = -1.0;
        } else {
          towerPower = 1.0;
        }
      } else {
        towerPower = 0;

      }
    // }

    myTower.runTower(towerPower);
    myFeeder.runFeeder(feederPower);
  }

  // Returns true when the command should end.

  @Override
  public void end(boolean interrupted) {
    myTower.runTower(0.0);
    myFeeder.runFeeder(0.0);
  }

  public boolean isFinished() {
    return false;
  }
}
