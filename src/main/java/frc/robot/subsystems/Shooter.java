// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



import static frc.robot.Constants.ShooterConstants.*;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Shooter extends SubsystemBase {

  private static final String kFF = null;
  private CANSparkMax shooterLead;
  private CANSparkMax shooterFollow;
  private SparkMaxPIDController shooterPidController;

  public boolean shooterReady;


    public Shooter() {
        shooterLead = new CANSparkMax(shooterLeadId, MotorType.kBrushless);
        shooterFollow = new CANSparkMax(shooterFollowId, MotorType.kBrushless);

        shooterPidController = shooterLead.getPIDController();

        shooterLead.restoreFactoryDefaults();
        shooterFollow.restoreFactoryDefaults();
    
        shooterFollow.follow(shooterLead);
    
        //shooterLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative , kPIDLoopIdx , kTimeOutMs);
    
        shooterLead.setInverted(true);
        shooterFollow.setInverted(false);
    
        //shooterLead.setSensorPhase(true); //the Follower isn't harvested for it's encoder therefor rotation doesn't need to be modified
       

        // set PID coefficients
        shooterPidController.setP(kP, 0);
        shooterPidController.setI(kI, 0);
        shooterPidController.setD(kD, 0);
        shooterPidController.setFF(kF);
        shooterPidController.setOutputRange(-1, 1);
    }


public double getVelocity(){
  return shooterLead.getEncoder().getVelocity();
}

public void run () {
  shooterPidController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity);
}
  @Override
public void periodic() {
  shooterReady = this.getVelocity() >= targetVelocity && this.getVelocity() <= targetVelocity;
  SmartDashboard.putBoolean("Shooter Ready", shooterReady);
  SmartDashboard.putNumber("Shooter Velocity", this.getVelocity());
}
}
