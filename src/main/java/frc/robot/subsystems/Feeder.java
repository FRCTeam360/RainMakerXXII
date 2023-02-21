// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.*;

public class Feeder extends SubsystemBase {
  private CANSparkMax motor;
  private static Feeder instance;
  /** Creates a new Feeder. */
  public Feeder() {
    motor = new CANSparkMax(CANIds.feederId,MotorType.kBrushless);
  }
  public static Feeder getInstance(){
    if(instance==null){
      instance = new Feeder();
    }
    return instance;
  }
  public void runFeeder(double speed){
    motor.set(speed);
  }
  public void stopFeeder(double speed){
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
