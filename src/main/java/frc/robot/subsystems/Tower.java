// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CANIds.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Tower extends SubsystemBase {
  /** Creates a new Tower. */
  private CANSparkMax myTower;
  private static Tower instance;
  public static Tower getInstance(){
    if(instance == null) {
      instance = new Tower();
    }
    return instance;
  }
  private Tower() {
    myTower = new CANSparkMax(towerId, MotorType.kBrushless);
  
  }

  public void runTower(double speed) {
    myTower.set(speed);
  }

  public void stopTower() {
    myTower.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
