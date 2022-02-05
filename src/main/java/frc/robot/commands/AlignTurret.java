// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class AlignTurret extends CommandBase {

  private Limelight myLimelight;
  private Turret myTurret;

  private double aimAdjust;
  private double aimError;

  public enum Direction{LEFT, MIDDLE, RIGHT};
  public enum Mode{SEEK_RIGHT, SEEK_LEFT, TARGET_IN_VIEW, LOCKED_ON_TARGET, WAIT_TO_SEEK};

  public AlignTurret(Limelight limelight, Turret turret) {
    myLimelight = limelight;
    myTurret = turret;
    aimError = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myLimelight, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aimError = myLimelight.getX() / 29.8;
  }

  public void align() {
    double aimAdjust = Turret.kP * this.aimError;
    if (this.aimError > 0.2) {
      aimAdjust += Turret.AimMinCmd;
    } else if (aimError < -0.2) {
      aimAdjust -= Turret.AimMinCmd;
    }
    myTurret.turn(-aimAdjust);
  }

  public void seek(Direction direction){
    switch(direction){
      case LEFT:
        myTurret.turn(-1);
        case RIGHT:
        default: 
        myTurret.turn(1);
    }
  }

  // Called once the commandd ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return Math.abs(myLimelight.getX()) < 0.4;
  }
}
