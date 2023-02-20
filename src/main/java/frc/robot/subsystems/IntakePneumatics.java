// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePneumatics extends SubsystemBase {
  private DoubleSolenoid solenoid;
  private static IntakePneumatics instance;
  
  public static IntakePneumatics getInstance(){
    if(instance == null) {
      instance = new IntakePneumatics();
    }
    return instance;
  }

  /** Creates a new IntakePneumatics. */
  private IntakePneumatics() {
    solenoid = new DoubleSolenoid(20, PneumaticsModuleType.CTREPCM, 0, 1);
    
  }

  public void intakeExtension() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void intakeRectraction() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
