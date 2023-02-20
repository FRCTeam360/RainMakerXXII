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

public class Tower extends SubsystemBase {
  private CANSparkMax motor;
  private static Tower instance;
  /** Creates a new Feeder. */
  public Tower() {
    motor = new CANSparkMax (towerId, MotorType.kBrushless);
  }

  public void runTower(double speed){
    motor.set(speed);
  }

  public void stopTower(){
    motor.set(0);
  }

  public static Tower getInstance() {
    if(instance == null){
      instance = new Tower();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
