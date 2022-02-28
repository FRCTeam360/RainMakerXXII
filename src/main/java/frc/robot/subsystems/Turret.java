// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.DigitalInputPorts.*;

public class Turret extends SubsystemBase {

  private DigitalInput leftLimitSwitch;
  private DigitalInput middleLimitSwitch;
  private DigitalInput rightLimitSwitch;

  public static final double maxSpeed = 0.6;

  public static final double kPAngle = 0.05;
  public static final double kIAngle = 0;
  public static final double kDAngle = 0.01;
  public static final double kFAngle = 0;

  public static final double kPLimelight = 0.013; // values may be altered, seperate for clarification , changer
                                                  // kPLimelight from .05
  public static final double kILimelight = 0; // *
  public static final double kDLimelight = 0.0; // * changed from 0.01
  public static final double kFLimelight = 0; // *

  public static final double AimMinCmd = 0.01;

  public static final double gearBoxRatio = 1.0 / 20.0;
  public static final double pulleyRatio = 1.5 / 17.5;
  public static final double degreesPerRotation = 360 / 1;

  private static Turret instance;

  private double previousAngle;
  private double angleTurnIntegral;

  public static final double leftSoftLimit = 90;
  public static final double rightSoftLimit = -90;

  public static final float leftSoftLimitEncoder = (float) (leftSoftLimit / gearBoxRatio / pulleyRatio
      / degreesPerRotation);
  public static final float rightSoftLimitEncoder = (float) (rightSoftLimit / gearBoxRatio / pulleyRatio
      / degreesPerRotation);

  public static double getDeadzoneAngleSize() {
    return 360 - leftSoftLimit + rightSoftLimit;
  }

  private double alignIntegral;
  private double previousTX;

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

    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, leftSoftLimitEncoder);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, rightSoftLimitEncoder);

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

  public void turn(double speed) {

    Math.min(maxSpeed, speed);
    Math.max(-maxSpeed, speed);

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

    double aimError = -currentTX;
    double deriv = aimError - previousTX;

    alignIntegral = alignIntegral + aimError;
    previousTX = aimError;

    double aimAdjust = (aimError * Turret.kPLimelight) + (alignIntegral * Turret.kILimelight)
        - (deriv * Turret.kDLimelight) + (kFLimelight);

    this.turn(aimAdjust);
  }

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
    // System.out.println("Turret Angle: " + getAngle());
    SmartDashboard.putNumber("Turret Encoder", turretMotor.getEncoder().getPosition());
  }
}
