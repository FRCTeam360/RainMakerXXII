// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;
import static frc.robot.Constants.DigitalInputPorts.*;

public class WaitUntilBallPassedSensor extends CommandBase {

  boolean ballPassedSensor = false;
  boolean endCommand = false;
  Tower myTower = Tower.getInstance();

  public enum Sensor{
    TOP, MID, FEED
  }

  Sensor selectedSensor;
  int sensorPort;
  /** Creates a new WaitUntilBallPassedSensor. */
  public WaitUntilBallPassedSensor(Sensor sensor) {
    selectedSensor = sensor;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public boolean ballPassedSensor(){
    switch (selectedSensor){
      case TOP:
      default:
        return !myTower.ballNotInTower();
    }
  }

  public boolean ballNotInSensor(){
    switch (selectedSensor){
      case TOP:
      default:
        return myTower.ballNotInTower();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ballPassedSensor() && ballNotInSensor();
  }
}
