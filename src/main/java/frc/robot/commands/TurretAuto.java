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
    SEEK_RIGHT, SEEK_LEFT, TARGET_IN_VIEW, LOCKED_ON_TARGET, AT_RIGHT_LIMIT, AT_LEFT_LIMIT, CALLIBRATE, TARGET_BLOCKED
  };

  private Mode mode;

  private double lastTargetPosition = 0;

  private Timer myTimer;

  private Direction pastSeekDirection;

  public TurretAuto(Limelight limelight, Turret turret) {
    myLimelight = limelight;
    myTurret = turret;

    myTimer = new Timer();

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
    this.checkLimits();
    this.updateLastKnownTargetAngle();
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
        this.waitAtLimit(Direction.RIGHT);
        break;
      case AT_LEFT_LIMIT:
        this.waitAtLimit(Direction.LEFT);
        /*
         * case CALLIBRATE: this.callibrate();
         */
        break;
      case TARGET_BLOCKED:
        this.targetBlocked();
      default:
    }
  }

  /**
   * 94-106 finds error from limelight to adjust alignment to target and turns
   * turret to fulfill alignment Checks for a valid target and if there is none
   * checks Turret angle and seeks in opposite direction
   */
  private void align() {
    if (!myLimelight.validTarget()) {

      this.mode = Mode.TARGET_BLOCKED;

      // if (myTurret.getAngle() <= 0) {
      // this.mode = Mode.SEEK_LEFT;
      // } else {
      // this.mode = Mode.SEEK_RIGHT;
      // }
      return;
    }

    double alignTX = myLimelight.getX();

    myTurret.alignLimelight(alignTX);

    if ((-1 <= alignTX && alignTX <= 1)) {
      this.mode = Mode.LOCKED_ON_TARGET;
    } else {
      this.mode = Mode.TARGET_IN_VIEW;
    }
  }

  /**
   * Turns turret in direction provided If there is a valid target mode changes to
   * target in view
   * 
   * @param direction direction to turn turret towards
   */
  private void seek(Direction direction) {
    if (myLimelight.validTarget()) {
      this.mode = Mode.TARGET_IN_VIEW;
      return;
    }
    pastSeekDirection = direction;
    switch (direction) {
      case LEFT:
        myTurret.turn(Turret.maxSpeed);
        break;
      case RIGHT:
      default:
        myTurret.turn(-Turret.maxSpeed);
    }
  }

  // public void setTurretMode(Mode mode) {
  // this.mode = mode;
  // }

  public void callibrate() {
    if (myTurret.checkMiddleLimitSwitch()) {
      myTurret.resetEncoderTicks();
      myTurret.turn(0);
    } else { // start turret on the left
      myTurret.turn(0.3);
    }
  }

  /**
   * If the target outside of the deadzone and is at the opposite side of the
   * limit the turret will go into seek mode to find it. If the target is outside
   * of the deadzone and is at the same side of the limit the turret will stay in
   * TARGET_IN_VIEW mode. When the target is in the deadzone the turret stays in
   * place.
   * 
   * @param limitSide which limit the turret is at
   */
  private void waitAtLimit(Direction limitSide) {
    myTurret.turn(0);
    double currentTX = myLimelight.getX();
    boolean validTarget = myLimelight.validTarget();

    if (limitSide == Direction.LEFT) {
      if (!validTarget || currentTX < -Turret.getDeadzoneAngleSize()) {
        this.mode = Mode.SEEK_RIGHT;
      } else if (currentTX > 0) {
        this.mode = Mode.TARGET_IN_VIEW;
      }
    } else {
      if (!validTarget || currentTX > Turret.getDeadzoneAngleSize()) {
        this.mode = Mode.SEEK_LEFT;
      } else if (currentTX < 0) {
        this.mode = Mode.TARGET_IN_VIEW;
      }
    }
  }

  /**
   * Checks to see if the turret is at either limit, right now for soft limits
   */
  private void checkLimits() {
    if (myTurret.isAtLeftLimit()) {
      this.mode = Mode.AT_LEFT_LIMIT;
    } else if (myTurret.isAtRightLimit()) {
      this.mode = Mode.AT_RIGHT_LIMIT;
    }
  }

  /**
   * Finds the last position of target detected
   */
  private void updateLastKnownTargetAngle() { /** Finds the last position of target detected */
    lastTargetPosition = myTurret.getAngle() + myLimelight.getX();
  }

  /**
   * Changes the turret mode to the opposite seeking direction if the target is
   * blocked, or to TARGET_IN_VIEW if there is a valid target.
   */
  private void targetBlocked() {
    myTurret.angleTurn(lastTargetPosition);
    myTimer.start();

    if (myLimelight.validTarget()) {
      this.mode = Mode.TARGET_IN_VIEW;
    } else if (myTimer.get() >= 1 && myTurret.getAngle() <= 0) {
      this.mode = Mode.SEEK_LEFT;
      myTimer.stop();
      myTimer.reset();
    } else if (myTimer.get() >= 1) {
      this.mode = Mode.SEEK_RIGHT;
      myTimer.stop();
      myTimer.reset();
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
