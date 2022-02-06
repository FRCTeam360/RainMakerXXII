// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.operatorInterface.*;

public class RunFeeder extends CommandBase {

  private final Feeder myFeeder;
  private final Shooter myShooter;
  private final OperatorControl operatorCont;
  
  public RunFeeder() {
    operatorCont = OperatorControl.getInstance();
    myShooter = Shooter.getInstance();
    myFeeder = Feeder.getInstance();

    addRequirements(myFeeder);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if shooter at speed, run all
    if(myShooter.isAtSpeed()){
      System.out.println("tower time");
      myFeeder.runBoth(1);
    }else{


      //manually run feeder
      if(operatorCont.getLeftTrigger()){
        if(operatorCont.getXButton()){
          myFeeder.runFeeder(-0.5);
        } else {
          myFeeder.runFeeder(0.5);
        }
      } else {
        myFeeder.runFeeder(0.0);
      }

      //manually run tower
      if(operatorCont.getRightTrigger()){
        if(operatorCont.getXButton()){
          myFeeder.runTower(-1.0);
        } else {
          myFeeder.runTower(1.0);
        }
      } else {
        myFeeder.runTower(0.0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myFeeder.runBoth(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
