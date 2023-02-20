// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.CANIds.*;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private CANSparkMax motor;
  private static Feeder instance;
  /** Creates a new Feeder. */
  public Feeder() {
    motor = new CANSparkMax (feederId, MotorType.kBrushless);
  }

  public void runFeeder(double speed){
    motor.set(speed);
  }

  public void stopFeeder(){
    motor.set(0);
  }

  public static Feeder getInstance() {
    if(instance == null){
      instance = new Feeder();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 