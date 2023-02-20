// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CANIds.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private CANSparkMax myFeeder;
  private static Feeder instance;
  public static Feeder getInstance(){
    if(instance == null) {
      instance = new Feeder();
    }
    return instance;
  }
  private Feeder() {
    this.myFeeder = new CANSparkMax(feederId, MotorType.kBrushless);
   }
  
  public void runFeeder(double speed) {
    myFeeder.set(speed);
  }
  
  public void stopFeeder() {
    myFeeder.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
