// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import static frc.robot.Constants.CANIds.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
    public class Shooter extends SubsystemBase {
      private WPI_TalonFX motorLead;
      private WPI_TalonFX motorFollow;
      private static Shooter instance;
      
      /** Creates a new Feeder. */
      public Shooter() {
        motorLead = new WPI_TalonFX (shooterLeadId);
        motorFollow = new WPI_TalonFX (shooterFollowId);
        motorLead.configFactoryDefault();
        motorFollow.configFactoryDefault();
        motorFollow.follow(motorLead);
        motorFollow.setInverted(true);
      }
      public void runShooterLead(double speed){
        motorLead.set(speed);
      }
      public void stopShooterLead(){
        motorLead.set(0);
      }

        public static Shooter getInstance(){
          if(instance==null){
            instance = new Shooter();
          }
          return instance;
        }
      

      
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
      }
    }

