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
<<<<<<< HEAD
    
        //set PID coefficients
        shooterPidController.setP(kP, 0);
        shooterPidController.setI(kI, 0);
        shooterPidController.setD(kD, 0);
        shooterPidController.setFF(kF);
        shooterPidController.setOutputRange(kMin.Output, kMax.Output); //first kMIN.Input -> originally = kMin.Output
        }}
        
         //shooterLead.configNominalOutputForward( 0 , kTimeOutMs);
         //shooterLead.configNominalOutputReverse( 0 , kTimeOutMs);
         //shooterLead.configPeakOutputForward( 1 , kTimeOutMs);
         //shooterLead.configPeakOutputReverse( -1 , kTimeOutMs);
     
         //shooterLead.config_kF(kPIDLoopIdx, kF , kTimeOutMs );
         //shooterLead.config_kP(kPIDLoopIdx, kP , kTimeOutMs );
         //shooterLead.config_kI(kPIDLoopIdx, kI , kTimeOutMs );
         //shooterLead.config_kD(kPIDLoopIdx, kD , kTimeOutMs );
        //}}

        
=======
       

        // set PID coefficients
        shooterLead.setP(kP, 0);
        shooterLead.setI(kI, 0);
        shooterLead.setD(kD, 0);
        shooterLead.setIZone(kIz);
        shooterLead.setFF(kFF);
        shooterLead.setOutputRange(kMinOutput, kMaxOutput);
    }


/*public double getVelocity(){
  return shooterLead.getSelectedSensorVelocity(0);
}
*/
@Override
public void periodic() {
  
}
}
>>>>>>> 34c8c92e7205c1c2ec01bde1e4fbab49d2741c52
