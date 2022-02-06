// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.DigitalInputPorts.*;

public class Turret extends SubsystemBase {

  private DigitalInput leftLimitSwitch;
  private DigitalInput middleLimitSwitch;
  private DigitalInput rightLimitSwitch;

  public static final double kP = 0.6;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kF = 0;

  public static final double AimMinCmd = 0.01;

  public static final double gearBoxRatio = 1/20;
  public static final double pulleyRatio = 1.5/17.5;
  public static final double degreesPerRotation = 360/1;

  private static Turret instance;

  private double previousAngle;
  private double integral;


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

    leftLimitSwitch = new DigitalInput(leftLimitSwitchPort);
    rightLimitSwitch = new DigitalInput(rightLimitSwitchPort);
  }

  public double getAngle(){
    double encoderPosition = turretMotor.getEncoder().getPosition();
    return encoderPosition * gearBoxRatio * pulleyRatio * degreesPerRotation;
  }

  public void zero(){
    if(middleLimitSwitch.get()){
      turretMotor.set(0);
      turretMotor.getEncoder().setPosition(0);
    } else {
      if(this.getAngle() > 135){
    } else {
    }}
  }

  public void turn(double speed){
    turretMotor.set(speed);
    
  }

  public void angleTurn(double inputAngle){
    double angle = this.getAngle();
    double error = inputAngle - angle;

    double deriv = angle - previousAngle;
    previousAngle = angle;
    integral = integral + error;

    double turretInput = (error * Turret.kP) + (integral * Turret.kI) - (deriv * Turret.kD);

    this.turn(turretInput);
  }

  public void resetEncoderTicks(){
    turretMotor.getEncoder().setPosition(0);
  }

  public boolean checkMiddleLimitSwitch(){
    return middleLimitSwitch.get();
  }

  public double getEncoderTick(){
    return turretMotor.getEncoder().getPosition();
    

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
