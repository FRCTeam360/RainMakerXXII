// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.LimelightConstants.*;

public class Align extends CommandBase {

  private Limelight myLimelight;
  private Turret myTurret;

  private double aimAdjust;
  private double aimError;

  public Align(Limelight limelight, Turret turret) {
    myLimelight = limelight;
    myTurret = turret;

    aimAdjust = 0;
    aimError = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aimError = myLimelight.getX() / 29.8;
    aimAdjust = kP * aimError;
    if (myLimelight.getX() > .2) {
      aimAdjust += AimMinCmd;
    } else if (myLimelight.getX() < -.2) {
      aimAdjust -= AimMinCmd;
    }
    myTurret.turnTurret(aimAdjust);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return Math.abs(myLimelight.getX()) < 0.4;
  }
}
