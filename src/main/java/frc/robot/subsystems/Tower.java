// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import static frc.robot.Constants.CANIds.*;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class Tower extends SubsystemBase {
  private static Tower instance;
  private CANSparkMax motor;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Tower(){
    motor = new CANSparkMax(towerId,MotorType.kBrushless);
  }
  public static Tower getInstance(){
    if(instance==null){
      instance = new Tower();
    }
    return instance;
  }
  public void runTower(double speed){
    motor.set(speed);
  }
  public void stopTower(double speed){
    motor.set(0);
  }
}