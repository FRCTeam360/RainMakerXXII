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

  public enum Direction {
    LEFT, RIGHT
  };

  public enum Mode {
    SEEK_RIGHT, SEEK_LEFT, TARGET_IN_VIEW, LOCKED_ON_TARGET, WAIT_TO_SEEK_RIGHT, WAIT_TO_SEEK_LEFT, CALLIBRATE
  };

  private Mode mode;

  public AlignTurret(Limelight limelight, Turret turret) {
    myLimelight = limelight;
    myTurret = turret;
    aimError = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myLimelight, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aimError = myLimelight.getX() / 29.8;

    switch (this.mode) {
    case SEEK_RIGHT:
      this.seek(Direction.RIGHT);
      break;
    case SEEK_LEFT:
      this.seek(Direction.LEFT);
      break;
    case TARGET_IN_VIEW:
    case LOCKED_ON_TARGET:
      this.align();
      break;
    case WAIT_TO_SEEK_RIGHT:
      this.waitToSeek(Direction.RIGHT);
      break;
    case WAIT_TO_SEEK_LEFT:
      this.waitToSeek(Direction.LEFT);
    case CALLIBRATE:
      this.callibrate();
      break;
    default:
    }
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

  public void seek(Direction direction) {
    if (myLimelight.validTarget()) {
      this.mode = Mode.TARGET_IN_VIEW;
      return;
    }

    switch (direction) {
    case LEFT:
      myTurret.turn(-1);
    case RIGHT:
    default:
      myTurret.turn(1);
    }
  }

  public void setTurretMode(Mode mode) {
    this.mode = mode;
  }

  public void callibrate() {
    if (myTurret.checkMiddleLimitSwitch()) {
      myTurret.resetEncoderTicks();
      myTurret.turn(0);
    } else {
      myTurret.turn(0.3);
    }
  }

  public void waitToSeek(Direction direction) {
    myTurret.turn(0);
    double currentTX = myLimelight.getX();

    if (direction == Direction.LEFT) {
      if (currentTX < -10) {
        this.mode = Mode.SEEK_RIGHT;
      } else if (currentTX > 0) {
        this.mode = Mode.TARGET_IN_VIEW;
      }
    } else {
      if (currentTX > 10) {
        this.mode = Mode.SEEK_LEFT;
      } else if (currentTX < 0) {
        this.mode = Mode.TARGET_IN_VIEW;
      }
    }
  }

  // Called once the commandd ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return Math.abs(myLimelight.getX()) < 0.4;
  }
}
