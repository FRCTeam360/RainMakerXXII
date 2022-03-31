// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class QueueBalls extends CommandBase {

  private final Shooter mShooter = Shooter.getInstance();
  private final Tower mTower = Tower.getInstance();
  private final Feeder mFeeder = Feeder.getInstance();
  private final Limelight mLimelight = Limelight.getInstance();

  private enum ComponentState {
    IDLE, FORWARD, REVERSE
  }

  private boolean ignoreShooter;

  private ComponentState towerAction = ComponentState.IDLE, feederAction = ComponentState.IDLE;

  private ComponentState pastTowerAction = ComponentState.IDLE, pastFeederAction = ComponentState.IDLE;

  /** Creates a new QueueCargo. */
  public QueueBalls(boolean shouldIgnoreShooter) {
    ignoreShooter = shouldIgnoreShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mTower, mFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting QueueBalls");
    this.towerAction = ComponentState.IDLE;
    this.pastTowerAction = ComponentState.IDLE;
    this.feederAction = ComponentState.IDLE;
    this.pastFeederAction = ComponentState.IDLE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.updateState();
    this.runState();
  }

  public void updateState() {
    if ((!ignoreShooter && mShooter.isAtSpeed() && mLimelight.isOnTarget()) || !mTower.ballInBottom()) {
      this.towerAction = ComponentState.FORWARD;
    } else {
      this.towerAction = ComponentState.IDLE;
    }

    if (!(mFeeder.ballInFeeder() && mTower.ballInBottom())) {
      this.feederAction = ComponentState.FORWARD;
    } else {
      this.feederAction = ComponentState.IDLE;
    }
  }

  public void runState() {
    switch (this.towerAction) {
      case FORWARD:
        this.runTower();
        break;
      case REVERSE:
      case IDLE:
      default:
        this.stopTower();
    }

    switch (this.feederAction) {
      case FORWARD:
        this.runFeeder();
        break;
      case REVERSE:
      case IDLE:
      default:
        this.stopFeeder();
    }
  }

  public void runTower() {
    mTower.runTower(1);
  }

  public void stopTower() {
    mTower.runTower(0);
  }

  public void runFeeder() {
    mFeeder.runFeeder(0.7);
  }

  public void stopFeeder() {
    mFeeder.runFeeder(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending QueueBalls");
    this.stopTower();
    this.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
