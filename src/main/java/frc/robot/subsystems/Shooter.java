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

  public boolean shooterReady;

  public double kP = 0;
  public double kI = 0;
  public double kD = 0;
  public double kF = 0;

  private double previousVelocity;
  private double integral;


    public Shooter() {
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
       
        SmartDashboard.putNumber("kP", 0.0);
        SmartDashboard.putNumber("kI", 0.0);
        SmartDashboard.putNumber("kD", 0.0);
        SmartDashboard.putNumber("kF", 0.0);

        // set PID coefficients

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
  kP = SmartDashboard.getNumber("kP", 0.0);
  kI = SmartDashboard.getNumber("kI", 0.0);
  kD = SmartDashboard.getNumber("kD", 0.0);
  kF = SmartDashboard.getNumber("kF", 0.0);

  shooterPidController.setP(kP, 0);
  shooterPidController.setI(kI, 0);
  shooterPidController.setD(kD, 0);
  shooterPidController.setFF(kF);
  shooterPidController.setOutputRange(-1, 1);

  updatePID();
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

public void updatePID(){
  // kP = SmartDashboard.getNumber("kP", 0.0);
  // kI = SmartDashboard.getNumber("kI", 0.0);
  // kD = SmartDashboard.getNumber("kD", 0.0);
  // kF = SmartDashboard.getNumber("kF", 0.0);
}

public void setVelocity(double velocityTarget){
  double error = velocityTarget - this.getVelocity();
  
  double deriv = velocityTarget - previousVelocity;
  previousVelocity = this.getVelocity();
  integral = integral + error;

  this.setSpeed( (error * kP) + (integral * kI) * (deriv * kD));
}


}
