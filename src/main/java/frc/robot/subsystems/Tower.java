// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.operatorInterface.XboxCont;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.DigitalInputPorts.*;

import java.io.DataInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Tower extends SubsystemBase {

  DigitalInput topSensor;
  DigitalInput bottomSensor;
  // DigitalInput middleSensor = new DigitalInput(1);
  // DigitalInput bottomSensor = new DigitalInput(2);
  static Tower instance;
  // need to createbthird input for middle sensor

  private CANSparkMax tower;
  private Tower pastTowerState;
  private boolean pastBallState = false;
  private BallTrackingState ballTrackingState = BallTrackingState.BOTTOM;
  private boolean pastBallInTop = false, pastBallInBottom = false;

  private CargoCounter mCargoCounter = CargoCounter.getInstance();

  private enum BallTrackingState {
    NO_BALL, BOTTOM, SHOOTING_ZONE
  }

  private Tower() {

    topSensor = new DigitalInput(topTowerSensor);
    bottomSensor = new DigitalInput(bottomTowerSensor);

    tower = new CANSparkMax(towerId, MotorType.kBrushless);

    tower.setIdleMode(IdleMode.kBrake);

    tower.setInverted(false);

    tower.setSmartCurrentLimit(20);

  }

  public static Tower getInstance() {
    if (instance == null) {
      instance = new Tower();
    }
    return instance;
  }

  public void runTower(double speed) {
    tower.set(speed);
  }

  public boolean ballNotInBottom() {
    return bottomSensor.get();
  }

  public boolean ballInBottom() {
    return !bottomSensor.get();
  }

  public boolean ballInTop() {
    return !topSensor.get();
  }

  public void trackShots() {
    switch (this.ballTrackingState) {
      case NO_BALL:
        this.trackNoBall();
        break;
      case BOTTOM:
        this.trackBallInBottom();
        break;
      case SHOOTING_ZONE:
        this.trackBallInShootingZone();
        break;
      default:
    }
    this.setPastBallInBottom();
    this.setPastBallInTop();
  }

  public double getMotorSpeed() {
    return this.tower.getEncoder().getVelocity();
  }

  private boolean isRisingEdgeTop() {
    return ballInTop() && !this.pastBallInTop;
  }
  
  private boolean isFallingEdgeTop() {
    return !ballInTop() && this.pastBallInTop;
  }

  private boolean isRisingEdgeBottom() {
    return ballInBottom() && !this.pastBallInBottom;
  }
  
  private boolean isFallingEdgeBottom() {
    return !ballInBottom() && this.pastBallInBottom;
  }

  private void setPastBallInTop() {
    this.pastBallInTop = this.ballInTop();
  }

  private void setPastBallInBottom() {
    this.pastBallInBottom = this.ballInBottom();
  }

  private void trackNoBall(){
    if(ballInBottom()){
      this.ballTrackingState = BallTrackingState.BOTTOM;
    }
  }

  private void trackBallInBottom(){
    double motorSpeed = this.getMotorSpeed();
    boolean isFallingEdgeBottom = this.isFallingEdgeBottom();
    if(isFallingEdgeBottom && motorSpeed > 0){
      this.ballTrackingState = BallTrackingState.SHOOTING_ZONE;
    }else if(isFallingEdgeBottom && motorSpeed < 0){
      this.ballTrackingState = BallTrackingState.NO_BALL;
    }
  }

  private void trackBallInShootingZone(){
    double motorSpeed = this.getMotorSpeed();
    boolean isFallingEdgeTop = this.isFallingEdgeTop();
    if(isFallingEdgeTop && motorSpeed >= 0){
      this.ballTrackingState = BallTrackingState.NO_BALL;
      this.mCargoCounter.incrementShootCount();
    } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Tower Temp", tower.getMotorTemperature());
    SmartDashboard.putBoolean("Top Sensor", topSensor.get());
    SmartDashboard.putBoolean("Bottom Sensor", bottomSensor.get());
    SmartDashboard.putString("Tracking State", this.ballTrackingState.toString());
    SmartDashboard.putNumber("Motor Speed", this.getMotorSpeed());

    this.trackShots();
  }
}