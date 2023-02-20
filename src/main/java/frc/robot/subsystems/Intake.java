// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CANIds.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  private static Intake instance;
  /** Creates a new Intake. */
  private CANSparkMax myIntake;
  public static Intake getInstance(){
    if(instance == null) {
      instance = new Intake();
    }
    return instance;
  }
  private Intake() {
    myIntake = new CANSparkMax(intakeId, MotorType.kBrushless);
  
  }

  public void runIntake(double speed) {
    myIntake.set(speed);
  }

  public void stopIntake() {
    myIntake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
