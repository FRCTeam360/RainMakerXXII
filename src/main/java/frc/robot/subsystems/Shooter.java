// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import static frc.robot.Constants.ShooterConstants.*;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooterLead;
  private CANSparkMax shooterFollow;

    public Shooter() {
        shooterLead = new CANSparkMax(shooterLeadId, MotorType.kBrushless);
        shooterFollow = new CANSparkMax(shooterFollowId, MotorType.kBrushless);
    
        shooterLead.restoreFactoryDefaults();
        shooterFollow.restoreFactoryDefaults();
    
        shooterFollow.follow(shooterLead);
    
        //shooterLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative , kPIDLoopIdx , kTimeOutMs);
    
        shooterLead.setInverted(true);
        shooterFollow.setInverted(false);
    
        //shooterLead.setSensorPhase(true); //the Follower isn't harvested for it's encoder therefor rotation doesn't need to be modified
    
        // set PID coefficients
        //shooterLead.setP(kP, 0);
        //shooterLead.setI(kI, 0);
        //shooterLead.setD(kD, 0);
        //shooterLead.setIZone(kIz);
        //shooterLead.setFF(kFF);
        //shooterLead.setOutputRange(kMinOutput, kMaxOutput);
         //}
        
         shooterLead.configNominalOutputForward( 0 , kTimeOutMs);
         shooterLead.configNominalOutputReverse( 0 , kTimeOutMs);
         shooterLead.configPeakOutputForward( 1 , kTimeOutMs);
         shooterLead.configPeakOutputReverse( -1 , kTimeOutMs);
     
         shooterLead.config_kF(kPIDLoopIdx, kF , kTimeOutMs );
         shooterLead.config_kP(kPIDLoopIdx, kP , kTimeOutMs );
         shooterLead.config_kI(kPIDLoopIdx, kI , kTimeOutMs );
         shooterLead.config_kD(kPIDLoopIdx, kD , kTimeOutMs );
        }}

        