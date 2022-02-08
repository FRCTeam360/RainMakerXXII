// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//used to be named RunFeeder.java 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Feeder;

import frc.robot.operatorInterface.*;

public class RunFeeder extends CommandBase {

  private final Tower myTower;
  private final Feeder myFeeder;
  private final OperatorControl operatorCont;
  
  public RunFeeder() {
    operatorCont = OperatorControl.getInstance();
    myTower = Tower.getInstance();
    myFeeder = Feeder.getInstance(); 

    addRequirements(myTower);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //runs feeder
    if(operatorCont.getLeftTrigger()){
      if(operatorCont.getAButton()){
        myFeeder.runFeeder(-1.0);
      } else {
        myFeeder.runFeeder(1.0);
      }
    } else {
      myFeeder.runFeeder(0.0);
    }

    //runs tower
    if(operatorCont.getRightTrigger()){
      if(operatorCont.getXButton()){
        myTower.runTower(-1.0);
      } else {
        myTower.runTower(1.0);
      }
    } else {
      myTower.runTower(0.0);
    }
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
