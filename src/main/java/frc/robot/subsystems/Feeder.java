// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANIds.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Feeder extends SubsystemBase {

  private TalonSRX feeder;

  private static Feeder instance;

  private Feeder() {
    feeder = new TalonSRX(feederId);
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
    feeder.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
