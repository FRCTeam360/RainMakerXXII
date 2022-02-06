// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.CANIds.*;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Shooter extends SubsystemBase {

  private static final String kFF = null;
  private CANSparkMax shooterLead;
  private CANSparkMax shooterFollow;
  private SparkMaxPIDController shooterPidController;
  private RelativeEncoder shooterEncoder;

  private static Shooter instance;

  public boolean shooterReady;

  // public double kP = 0;
  // public double kI = 0;
  // public double kD = 0;
  // public double kF = 0;

  private double previousVelocity;
  private double integral;

  public boolean isAtSpeed;


    private Shooter() {
        shooterLead = new CANSparkMax(shooterLeadId, MotorType.kBrushless);
        shooterFollow = new CANSparkMax(shooterFollowId, MotorType.kBrushless);

        shooterPidController = shooterLead.getPIDController();

        shooterEncoder = shooterLead.getEncoder();

        shooterLead.restoreFactoryDefaults();
        shooterFollow.restoreFactoryDefaults();
    
        shooterFollow.follow(shooterLead, true);

        shooterLead.setSmartCurrentLimit(40);
        shooterFollow.setSmartCurrentLimit(40);

        shooterLead.setIdleMode(IdleMode.kCoast);
        shooterFollow.setIdleMode(IdleMode.kCoast);

    
        //shooterLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative , kPIDLoopIdx , kTimeOutMs);
    
        shooterLead.setInverted(false);
        shooterFollow.setInverted(false);
    
        //shooterLead.setSensorPhase(true); //the Follower isn't harvested for it's encoder therefor rotation doesn't need to be modified
       
        // SmartDashboard.putNumber("kP", 0.0);
        // SmartDashboard.putNumber("kI", 0.0);
        // SmartDashboard.putNumber("kD", 0.0);
        // SmartDashboard.putNumber("kF", 0.0);

        // set PID coefficients

    }
  /**
   * Gets the Singleton Shooter instance
   * @return the Singleton Shooter instance
   */
  public static Shooter getInstance(){
    if(instance == null){
      instance = new Shooter();
    }
    return instance;
  }


public double getVelocity(){
  return shooterLead.getEncoder().getVelocity();
}

// public void run () {
//   shooterPidController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity);
// }
  @Override
public void periodic() {
  // shooterReady = this.getVelocity() >= targetVelocity && this.getVelocity() <= targetVelocity;
  // SmartDashboard.putBoolean("Shooter Ready", shooterReady);
  // SmartDashboard.putNumber("Shooter Velocity", this.getVelocity());
  // kP = SmartDashboard.getNumber("kP", 0.0);
  // kI = SmartDashboard.getNumber("kI", 0.0);
  // kD = SmartDashboard.getNumber("kD", 0.0);
  // kF = SmartDashboard.getNumber("kF", 0.0);

  SmartDashboard.putNumber("Shooter Velocity", this.getVelocity());
}
// public void setVelocity (double output) {
//   shooterPidController.setReference(output, CANSparkMax.ControlType.kVelocity);

//   shooterReady = this.getVelocity() >= output && this.getVelocity() <= output;
//   SmartDashboard.putBoolean("Shooter Ready", shooterReady);
//   SmartDashboard.putNumber("Shooter Velocity", this.getVelocity());
// }

public void setSpeed(double output){
  shooterLead.set(output);
}

public void setVelocity(double velocityTarget){

  velocityTarget = velocityTarget * -1;

  double error = velocityTarget - this.getVelocity();
  
  double deriv = velocityTarget - previousVelocity;
  previousVelocity = this.getVelocity();
  integral = integral + error;

  double speed = (velocityTarget / kF) + (error * kP) + (integral * kI) - (deriv * kD);

  speed = Math.min(speed, 0.7);
  speed = Math.max(speed, -0.7);
  
  this.setSpeed(speed);
  SmartDashboard.putNumber("speed set", speed);

  isAtSpeed = ((this.getVelocity()<=( velocityTarget + 100)));
  System.out.println("Is at speed  " + isAtSpeed);
  System.out.println("Velocity target  " + velocityTarget);
  }

  
  public boolean isAtSpeed(){
    return isAtSpeed;
  }
}


