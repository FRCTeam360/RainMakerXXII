// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj.XboxController;

public class RunFeeder extends CommandBase {

  private final XboxController cont;
  private final Feeder myFeeder;
  
  public RunFeeder() {
    cont = new XboxController(driverContPort);
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
    if(cont.getBButton()){
      if(!cont.getLeftStickButton()){
        myFeeder.runFeeder(1.0);
      } else {
        myFeeder.runFeeder(-1.0);
      }
    }else{
      myFeeder.runFeeder(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}