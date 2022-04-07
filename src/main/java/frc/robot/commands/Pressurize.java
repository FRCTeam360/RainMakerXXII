// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

import org.ejml.equation.ManagerFunctions.InputN;


import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class Pressurize extends CommandBase {

  private Timer timer;
  private boolean shouldRun;

  private final Pneumatics pneumatics;

  /** Creates a new Pressurize. */

  public Pressurize(Pneumatics inPneumatics) {
    pneumatics = inPneumatics;
    timer = new Timer();
    shouldRun = true;
    timer.reset();
    timer.stop();

    addRequirements(inPneumatics);// Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneumatics.pressurize();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneumatics.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
