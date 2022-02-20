// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.CANIds.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private static final String kFF = null;
  private CANSparkMax shooterLead;
  private CANSparkMax shooterFollow;
  private SparkMaxPIDController shooterPidController;
  private RelativeEncoder shooterEncoder;

  private static Shooter instance;

  public boolean shooterReady;

  private double previousVelocity;
  private double integral;

  public double velocityTarget = 2000;
  public boolean isAtSpeed;

  // Old data, need to tune
  public static final int kSlotIdx = 0;
  public static final int kTimeOutMs = 30;
  public static final int kPIDLoopIdx = 0;
  public static final double kP = 0.0009;
  public static final double kI = 0;
  public static final double kD = 0.0005;
  public static final double kF = 5000;
  public static final double kPeakOutput = 1;

  public static final double backupTargetVelocity = 14500; // Constant
  public static double targetVelocity = backupTargetVelocity; // will get changed in the future by limelight
                                                              // subsystem or a command...

  public static final double aVal = 2.697; // Quad Ratic regression values
  public static final double bVal = -52.912;
  public static final double cVal = 14815.146;

  private Shooter() {
    shooterLead = new CANSparkMax(shooterLeadId, MotorType.kBrushless);
    shooterFollow = new CANSparkMax(shooterFollowId, MotorType.kBrushless);

    shooterPidController = shooterLead.getPIDController();

    shooterEncoder = shooterLead.getEncoder();

    shooterLead.restoreFactoryDefaults();
    shooterFollow.restoreFactoryDefaults();

    shooterFollow.follow(shooterLead, true);

    shooterLead.setSmartCurrentLimit(40);
    shooterFollow.setSmartCurrentLimit(40);

    shooterLead.setIdleMode(IdleMode.kCoast);
    shooterFollow.setIdleMode(IdleMode.kCoast);

    shooterLead.setInverted(true);
    // shooterLead.setSensorPhase(true); //the Follower isn't harvested for it's
    // encoder therefor rotation doesn't need to be modified

    // SmartDashboard.putNumber("kP", 0.0);
    // SmartDashboard.putNumber("kI", 0.0);
    // SmartDashboard.putNumber("kD", 0.0);
    // SmartDashboard.putNumber("kF", 0.0);
  }

  /**
   * Gets the Singleton Shooter instance
   * 
   * @return the Singleton Shooter instance
   */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  public double getVelocity() {
    return shooterLead.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    // kP = SmartDashboard.getNumber("kP", 0.0);
    // kI = SmartDashboard.getNumber("kI", 0.0);
    // kD = SmartDashboard.getNumber("kD", 0.0);
    // kF = SmartDashboard.getNumber("kF", 0.0);

    SmartDashboard.putNumber("Shooter Velocity", this.getVelocity());
  }

  /**
   * Sets shooter to percentage output
   * 
   * @param output motor output from -1 to 1
   */
  public void setSpeed(double output) {
    shooterLead.set(output);
  }

  /**
   * Sets shooter to velocity using PID and FeedForward
   * 
   * @param target target velocity in shooter RPMs
   */
  public void setVelocity(double target) {

    velocityTarget = target;

    double error = velocityTarget - this.getVelocity();

    double deriv = velocityTarget - previousVelocity;
    previousVelocity = this.getVelocity();
    integral = integral + error;

    double speed = (velocityTarget / kF) + (error * kP) + (integral * kI) - (deriv * kD);

    // temporary limiting of max output - will probably change
    speed = Math.min(speed, 0.7);
    speed = Math.max(speed, -0.7);

    this.setSpeed(speed);
    SmartDashboard.putNumber("speed set", speed);
  }

  public boolean isAtSpeed() {
    double error = velocityTarget - this.getVelocity();
    return Math.abs(error) <= 100;
  }
}
