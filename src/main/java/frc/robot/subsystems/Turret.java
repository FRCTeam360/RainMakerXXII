// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.DigitalInputPorts.*;

public class Turret extends SubsystemBase {
  private DigitalInput leftLimitSwitch;
  private DigitalInput middleLimitSwitch = new DigitalInput(middleLimitSwitchPort);
  private DigitalInput rightLimitSwitch;

  public static final double maxSpeed = 0.8;

  public static final double kPAngle = 0.05;
  public static final double kIAngle = 0;
  public static final double kDAngle = 0.01;
  public static final double kFAngle = 0;

  public static final double kPLimelight = 0.030; // values may be altered, seperate for clarification , changer
                                                  // kPLimelight from .05
  public static final double kILimelight = 0; // *
  public static final double kDLimelight = 0.04; // * changed from 0.01
  public static final double kFLimelight = 0; // *

  public static final double AimMinCmd = 0.01;

  public static final double gearBoxRatio = 1.0 / 20.0;
  public static final double pulleyRatio = 1.5 / 17.5;
  public static final double degreesPerRotation = 360 / 1;
  public static final double rotationsPerTick = 1 / 42;

  private static Turret instance;

  private DriveTrain myDriveTrain = DriveTrain.getInstance();

  private double previousAngle;
  private double angleTurnIntegral;

  public static final float leftSoftLimit = 150;
  public static final float rightSoftLimit = -150;

  public static final float leftSoftLimitEncoder = (float) (leftSoftLimit / gearBoxRatio / pulleyRatio
      / degreesPerRotation);
  public static final float rightSoftLimitEncoder = (float) (rightSoftLimit / gearBoxRatio / pulleyRatio
      / degreesPerRotation);

  public static double getDeadzoneAngleSize() {
    return 360 - leftSoftLimit + rightSoftLimit;
  }

  private double alignIntegral;
  private double previousTX;

  private boolean pastMiddleLimitSwitchState;

  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  private CANSparkMax turretMotor;

  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new CANSparkMax(turretMotorID, MotorType.kBrushless);

    turretMotor.restoreFactoryDefaults();

    turretMotor.setIdleMode(IdleMode.kBrake);

    turretMotor.setInverted(false);

    // turretMotor.getEncoder().setVelocityConversionFactor(rotationsPerTick * gearBoxRatio * pulleyRatio * degreesPerRotation);

    turretMotor.setSoftLimit(SoftLimitDirection.kForward, leftSoftLimitEncoder);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, rightSoftLimitEncoder);

    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // leftLimitSwitch = new DigitalInput(Port);
    // rightLimitSwitch = new DigitalInput(rightLimitSwitchPort);
  }

  /**
   * retrieves the encoder position and returns it multiplied by pulleyratio,
   * gearboxratio, and degreesperrotation to get the turret's angle relative to
   * front of robot
   */
  public double getAngle() {
    double encoderPosition = turretMotor.getEncoder().getPosition();
    return encoderPosition * gearBoxRatio * pulleyRatio * degreesPerRotation;
  }

  public void zero() {
    this.angleTurn(0);
  }

  public void limitSwitchResetAngle() {
    boolean currentMiddleLimitState = this.checkMiddleLimitSwitch();

    if (currentMiddleLimitState == false && pastMiddleLimitSwitchState == true) {
        resetEncoderTicks();
    }

    pastMiddleLimitSwitchState = currentMiddleLimitState;
  }

  public void turn(double speed) {

    // Math.min(maxSpeed, speed);
    // Math.max(-maxSpeed, speed);

    turretMotor.set(speed);

    // if (isAtLeftLimit() || isAtRightLimit()) {
    //   turretMotor.set(0);
    // } else {
    //   turretMotor.set(speed);
    // }
  }

  /*
   * Turns turret to match angle provided to turret
   * 
   * @param inputAngle inputAngle is the value for the turret to turn towards
   */
  public void angleTurn(double inputAngle) {
    double angle = this.getAngle();
    double error = inputAngle - angle;

    double deriv = angle - previousAngle;
    previousAngle = angle;

    angleTurnIntegral = angleTurnIntegral + error;

    double turretInput = (error * Turret.kPAngle) + (angleTurnIntegral * Turret.kIAngle) - (deriv * Turret.kDAngle)
        + (kFAngle);

    this.turn(turretInput);
  }

  public void alignLimelight(double currentTX) {

    double aimError = - shootOnMove(currentTX);
    double deriv = aimError - previousTX;

    alignIntegral = alignIntegral + aimError;
    previousTX = aimError;

    double aimAdjust = (aimError * Turret.kPLimelight) + (alignIntegral * Turret.kILimelight)
        - (deriv * Turret.kDLimelight) + (kFLimelight);

    this.turn(aimAdjust);
  }

  public double getTargetRelativeVelocity(){
    return myDriveTrain.getVelocityMetersPerSec() * Math.sin(Math.toRadians(getAngle()));
  }

  private double shootOnMove(double angle){
    return angle + (getTargetRelativeVelocity() * 125);
  }

  // private double shootOnMove(double angle){
  //   System.out.println("speed: " + myDriveTrain.getVelocityMetersPerSec());
  //   if(getAngle() > 45 && getAngle() < 135){
  //     return angle + (myDriveTrain.getVelocityMetersPerSec() * 125);
  //   } else if (getAngle() < -45 && getAngle() > -135){
  //     return angle + (myDriveTrain.getVelocityMetersPerSec() * -125);
  //   } else {
  //     return angle;
  //   }
  // }

  public void resetEncoderTicks() {
    turretMotor.getEncoder().setPosition(0);
  }

  public boolean checkMiddleLimitSwitch() {
    return middleLimitSwitch.get();
  }

  public Boolean isAtLeftLimit() {
    return this.getAngle() >= leftSoftLimit && turretMotor.getEncoder().getVelocity() > 0;
  }

  public Boolean isAtRightLimit() {
    return this.getAngle() <= rightSoftLimit && turretMotor.getEncoder().getVelocity() < 0;
  }

  public double getEncoderTick() {
    return turretMotor.getEncoder().getPosition();

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Angle", getAngle());
    // SmartDashboard.putNumber("Turret Encoder", turretMotor.getEncoder().getPosition());
    // System.out.println("limit: " + middleLimitSwitch.get());
    this.limitSwitchResetAngle();
    SmartDashboard.putNumber("turret temp", turretMotor.getMotorTemperature());
  }
}
