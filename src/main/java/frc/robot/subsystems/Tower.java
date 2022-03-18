// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.operatorInterface.XboxCont;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.DigitalInputPorts.*;

import java.io.DataInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class Tower extends SubsystemBase {

  DigitalInput topSensor;
  // DigitalInput middleSensor = new DigitalInput(1);
  // DigitalInput bottomSensor = new DigitalInput(2);
  static Tower instance;
  // need to createbthird input for middle sensor

  private CANSparkMax tower;

  private Tower() {

    topSensor = new DigitalInput(topTowerSensor);

    tower = new CANSparkMax(towerId, MotorType.kBrushless);

    tower.setIdleMode(IdleMode.kBrake);

    tower.setInverted(false);

    tower.setSmartCurrentLimit(20);
  }

  public static Tower getInstance() {
    if (instance == null) {
      instance = new Tower();
    }
    return instance;
  }

  public void runTower(double speed) {
    tower.set(speed);
  }

  public boolean ballNotInTower() {
    System.out.println("topsensor: " + topSensor.get());
    return topSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Tower Temp", tower.getMotorTemperature());
  }
}