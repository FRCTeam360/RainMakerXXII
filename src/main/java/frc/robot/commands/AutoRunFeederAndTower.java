// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj.Timer;

public class AutoRunFeederAndTower extends CommandBase {
  private final Tower myTower = Tower.getInstance();
  private final Feeder myFeeder = Feeder.getInstance();
  private final Shooter myShooter = Shooter.getInstance();
  private final Timer myTimer = new Timer();
  
  // private Boolean willEnd;
  // private Boolean ballPresentInitially;
  private Boolean ballPassedSensor = false;
  /** Creates a new AutoRunFeederAndTower.
   * 
   * @param endOnExit if true will end when tower sensor has no ball
   */
  public AutoRunFeederAndTower() {

    addRequirements(myTower, myFeeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballPassedSensor = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (myShooter.isAtSpeed()) {
      myTower.runTower(1.0);
      myFeeder.runFeeder(1.0);
    }

    if (!myTower.ballNotInBottom()){
      ballPassedSensor = true;
    }

    if(ballPassedSensor && myTower.ballNotInBottom()){
      myTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myTower.runTower(0);
    myFeeder.runFeeder(0);
    myTimer.stop();
    myTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(myTimer.get() >= 0.25){
      return true;
    } else {
      return false;
    }
  }
}
