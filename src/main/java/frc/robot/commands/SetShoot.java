// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.Shooter;

public class SetShoot extends CommandBase {

  private final Shooter shooter;
  private final XboxController cont;
  /** Creates a new setShoot. */
  public SetShoot(Shooter shooter) {
    this.shooter = shooter;
    cont = new XboxController(1);
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(cont.getAButton()){
      shooter.setVelocity(2000);
    } else {
      shooter.setSpeed(0.0);
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
