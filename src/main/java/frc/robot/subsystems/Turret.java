// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private static Turret instance;
  /** Creates a new Intake. */
  private CANSparkMax myTurret;
  public static Turret getInstance(){
    if(instance == null) {
      instance = new Turret();
    }
    return instance;
  }
  /** Creates a new Turret. */
  private Turret() {
    myTurret = new CANSparkMax(CANIds.turretMotorID, MotorType.kBrushless);
  
  }
  public void runTurret(double speed) {
    myTurret.set(speed);
  }

  public void stopTurret() {
    myTurret.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
