// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.DigitalInputPorts.*;

public class Turret extends SubsystemBase {

  private DigitalInput leftLimitSwitch;
  private DigitalInput middleLimitSwitch;
  private DigitalInput rightLimitSwitch;

  public static final double kP = 0.6;
  public static final double AimMinCmd = 0.01;

  private static Turret instance;

  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  private static CANSparkMax turretMotor;

  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new CANSparkMax(turretMotorID, MotorType.kBrushless);

    turretMotor.restoreFactoryDefaults();

    leftLimitSwitch = new DigitalInput(leftLimitSwitchPort);
    rightLimitSwitch = new DigitalInput(rightLimitSwitchPort);
  }

  public void turnTurret(double aimError) {
    if (leftLimitSwitch.get() || rightLimitSwitch.get()) {
      turretMotor.set(0);
    } else {
      turretMotor.set(aimError);
    }
  }

  public void align(double aimError) {
    double aimAdjust = kP * aimError;
    if (aimError > .2) {
      aimAdjust += AimMinCmd;
    } else if (aimError < -.2) {
      aimAdjust -= AimMinCmd;
    }
    this.turnTurret(-aimAdjust);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
