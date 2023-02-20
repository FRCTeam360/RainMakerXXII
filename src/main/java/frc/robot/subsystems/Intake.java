// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private CANSparkMax motor;
  /** Creates a new Intake. */
  
  public Intake() {
  motor = new CANSparkMax(10,MotorType.kBrushless);
  }
  public static Intake getInstance(){
    if(instance==null){
      instance = new Intake();
    }
    return instance;
  }
  public void runIntake(double speed){
    motor.set(speed);
  }
  public void stopIntake(){
    motor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
