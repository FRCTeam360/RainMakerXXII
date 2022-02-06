// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.operatorInterface.XboxCont;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;


import static frc.robot.Constants.CANIds.*;

import java.io.DataInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Feeder extends SubsystemBase {

  // DigitalInput topSensor = new DigitalInput(0);
  // DigitalInput bottomSensor = new DigitalInput(1);
  // DigitalInput middleSensor = new DigitalInput(2);
  // We have to create a third input for the middle sensor



  private CANSparkMax feeder;
  private CANSparkMax tower;

  private static Feeder instance;

  private Feeder() {

    DigitalInput topSensor = new DigitalInput(0);
    DigitalInput bottomSensor = new DigitalInput(1);
    DigitalInput middleSensor = new DigitalInput(2);



    feeder = new CANSparkMax(feederId, MotorType.kBrushless);
    tower = new CANSparkMax(towerId, MotorType.kBrushless);

    feeder.setIdleMode(IdleMode.kCoast);
    tower.setIdleMode(IdleMode.kCoast);

    tower.setInverted(true);
    feeder.setInverted(true);

    feeder.setSmartCurrentLimit(20);
    tower.setSmartCurrentLimit(20);
  }

  /**
   * Gets the Singleton Feeder instance
   * @return the Singleton Feeder instance
   */
  public static Feeder getInstance(){
    if(instance == null){
      instance = new Feeder();
    }
    return instance;
  }

  public void runFeeder (double speed) {
    feeder.set(speed);
  }



  public void runTower (double speed) {
    tower.set(speed);
  }

  public void runBoth (double speed) {
    runFeeder(speed);
    runTower(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}