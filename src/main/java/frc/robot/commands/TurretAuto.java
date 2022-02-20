// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj.Timer;

public class TurretAuto extends CommandBase {

  private Limelight myLimelight;
  private Turret myTurret;

  public enum Direction {
    LEFT, RIGHT
  };

  public enum Mode {
    SEEK_RIGHT, SEEK_LEFT, TARGET_IN_VIEW, LOCKED_ON_TARGET, AT_RIGHT_LIMIT, AT_LEFT_LIMIT, CALLIBRATE,
    TARGET_BLOCKED
  };

  private Mode mode;

  public double lastTargetPosition = 0;

  public Timer myTimer;

  private Direction pastSeekDirection;

  public TurretAuto(Limelight limelight, Turret turret) {
    myLimelight = limelight;
    myTurret = turret;

    this.mode = Mode.SEEK_RIGHT;
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
    this.softLimit();
    this.updateLastKnownTargetAngle();

    System.out.println("Mode: " + mode);
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
      case AT_RIGHT_LIMIT:
        this.waitToSeek(Direction.RIGHT);
        break;
      case AT_LEFT_LIMIT:
        this.waitToSeek(Direction.LEFT);
        /*
         * case CALLIBRATE:
         * this.callibrate();
         */
        break;
      case TARGET_BLOCKED:
        this.targetBlocked();
      default:
    }

  }

  public void align() {
  
    if (!myLimelight.validTarget()) {
      if (myTurret.getAngle() <= 0) {
        this.mode = Mode.SEEK_LEFT;
      } else {
        this.mode = Mode.SEEK_RIGHT;
      }
      return;
    }
    double aimError = myLimelight.getX();

    double aimAdjust = Turret.kP * aimError;
    if (aimError > 0.2) {
      aimAdjust += Turret.AimMinCmd;
      this.mode = Mode.TARGET_IN_VIEW;
    } else if (aimError < -0.2) {
      aimAdjust -= Turret.AimMinCmd;
      this.mode = Mode.TARGET_IN_VIEW;
    } else {
      this.mode = Mode.LOCKED_ON_TARGET;
    }
    myTurret.turn(-aimAdjust);

  }

  public void seek(Direction direction) {
    if (myLimelight.validTarget()) {
      this.mode = Mode.TARGET_IN_VIEW;
      return;
    }
    pastSeekDirection = direction;
    switch (direction) {
      case LEFT:
        myTurret.turn(0.2);
        break;
      case RIGHT:
      default:
        myTurret.turn(-0.2);
    }
  }

  public void setTurretMode(Mode mode) {
    this.mode = mode;
  }

  /*
   * public void callibrate() {
   * if (myTurret.checkMiddleLimitSwitch()) {
   * myTurret.resetEncoderTicks();
   * myTurret.turn(0);
   * } else {
   * // start turret on the left
   * myTurret.turn(0.3);
   * }
   * }
   */
  public void waitToSeek(Direction direction) {
    myTurret.turn(0);
    double currentTX = myLimelight.getX();
    boolean validTarget = myLimelight.validTarget();

    if (direction == Direction.LEFT) {
      if (!validTarget || currentTX < -10) {
        this.mode = Mode.SEEK_RIGHT;
      } else if (currentTX > 0) {
        this.mode = Mode.TARGET_IN_VIEW;
      }
    } else {
      if (!validTarget || currentTX > 10) {
        this.mode = Mode.SEEK_LEFT;
      } else if (currentTX < 0) {
        this.mode = Mode.TARGET_IN_VIEW;
      }
    }
  }

  public void softLimit() {
    if (myTurret.isAtLeftLimit()) {
      this.mode = Mode.AT_LEFT_LIMIT;
    } else if (myTurret.isAtRightLimit()) {
      this.mode = Mode.AT_RIGHT_LIMIT;
    }
  }

  public void updateLastKnownTargetAngle() {
    lastTargetPosition = myTurret.getAngle() + myLimelight.getX();
  }

  public void targetBlocked() {
    myTurret.angleTurn(lastTargetPosition);

    if (myLimelight.validTarget()) {
      this.mode = Mode.TARGET_IN_VIEW;
    } else if (myTimer.get() >= 50 && pastSeekDirection == Direction.LEFT) {
      this.mode = Mode.SEEK_LEFT;
    } else if (myTimer.get() >= 50 && pastSeekDirection == Direction.RIGHT) {
      this.mode = Mode.SEEK_RIGHT;
    }
  }

  // Called once the command ends or is interrupted.
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
