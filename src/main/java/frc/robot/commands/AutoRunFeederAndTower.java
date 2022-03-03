// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class AutoRunFeederAndTower extends CommandBase {
  private final Tower myTower;
  private final Feeder myFeeder;
  private final Shooter myShooter;
  
  // private Boolean willEnd;
  // private Boolean ballPresentInitially;
  private Boolean ballPassedSensor = false;
  /** Creates a new AutoRunFeederAndTower.
   * 
   * @param endOnExit if true will end when tower sensor has no ball
   */
  public AutoRunFeederAndTower() {
    myTower = Tower.getInstance();
    myFeeder = Feeder.getInstance();
    myShooter = Shooter.getInstance();

    // willEnd = endOnExit;

    addRequirements(myTower, myFeeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // ballPresentInitially = !myTower.ballNotInTower();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (myShooter.isAtSpeed()) {
      myTower.runTower(1.0);
      myFeeder.runFeeder(1.0);
    }

    if (!myTower.ballNotInTower()){
      ballPassedSensor = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myTower.runTower(0);
    myFeeder.runFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(ballPassedSensor){
      return myTower.ballNotInTower();
    } else {
      return false;
    }
  }
}
