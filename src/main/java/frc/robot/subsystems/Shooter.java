// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import static frc.robot.Constants.CANIds.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX shooterLead;
  private WPI_TalonFX shooterFollow;

  private static Shooter instance;

  private final Limelight myLimelight = Limelight.getInstance();

  public boolean shooterReady;

  public double velocityTarget = 2000;
  private double previousTarget;
  private double newTarget;
  public boolean isAtSpeed;

  // public static final double falconEncoderToRotations = 1.0 / 2048.0;
  // public static final double decisecondsPerSeconds = 10.0;
  // public static final double secondsPerMinutes = 60.0;
  // public static final double falconRttnPerShooterRttn = 24.0 / 36.0;
  // public static final double shooterToRPM = falconEncoderToRotations *
  // falconRttnPerShooterRttn * decisecondsPerSeconds
  // * decisecondsPerSeconds;

  public static final double shooterToRPM = (600.0 / 2048.0) * (3.0 / 2.0);
  public static final double shooterToMotorRPM = (600.0 / 2048.0);

  // Old data, need to tune
  public static final int kSlotIdx = 0;
  public static final int kTimeOutMs = 30;
  public static final int kPIDLoopIdx = 0;
  public static double kP = 0.3;
  public static double kI = 0.000083;
  public static double kD = 0;
  public static double kF = 0.0471;
  public static double kIz = 200;
  public static final double kPeakOutput = 1;

  public static final double backupTargetVelocity = 14500; // Constant
  public static double targetVelocity = backupTargetVelocity; // will get changed in the future by limelight
                                                              // subsystem or a command...

  private static final double a = 0.0086455746;
  private static final double b = -0.1318250336;
  private static final double c = 0.7137822277;
  private static final double d = -30.47373396;
  private static final double e = 3327.558816;

  public static final double MAX_SHOOTER_ACCELERATION = 5000;
  private final SlewRateLimiter filter = new SlewRateLimiter(MAX_SHOOTER_ACCELERATION);

  private Shooter() {
    shooterLead = new WPI_TalonFX(shooterLeadId);
    shooterFollow = new WPI_TalonFX(shooterFollowId);

    shooterLead.configFactoryDefault();
    shooterFollow.configFactoryDefault();

    shooterFollow.follow(shooterLead);

    shooterLead.setNeutralMode(NeutralMode.Coast);
    shooterFollow.setNeutralMode(NeutralMode.Coast);

    shooterLead.setInverted(false);
    shooterFollow.setInverted(true);

    shooterLead.config_kF(0, kF, kTimeOutMs);
    shooterLead.config_kP(0, kP, kTimeOutMs);
    shooterLead.config_kI(0, kI, kTimeOutMs);
    shooterLead.config_kD(0, kD, kTimeOutMs);
    shooterLead.config_IntegralZone(0, kIz, kTimeOutMs);
    shooterLead.configNominalOutputForward(0, kTimeOutMs);
    shooterLead.configNominalOutputReverse(0, kTimeOutMs);
    shooterLead.configPeakOutputForward(1, kTimeOutMs);
    shooterLead.configPeakOutputReverse(-1, kTimeOutMs);
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
    return shooterLead.getSelectedSensorVelocity() * shooterToRPM;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Shooter Velocity", this.getVelocity());
    SmartDashboard.putNumber("Shooter Ticks", shooterLead.getSelectedSensorVelocity());
    SmartDashboard.putBoolean("shooter ready", isAtSpeed());
  }

  /**
   * Sets shooter to percentage output
   * 
   * @param output motor output from -1 to 1
   */
  public void setPower(double output) {
    shooterLead.set(output);
  }

  /**
   * Sets shooter to velocity using PID and FeedForward
   * 
   * @param target target velocity in shooter RPMs
   */
  public void setVelocity(double target) {
    if (target == 0) {
      filter.reset(0);
      this.setPower(0);
      velocityTarget = 0;
    } else {

      // target = shootOnMove(target);

      velocityTarget = filter.calculate(target);

      shooterLead.set(ControlMode.Velocity, velocityTarget * 2 / 3 / shooterToMotorRPM);
    }
  }

  private double shootOnMove(double target) {
    double deriv = target - previousTarget;
    previousTarget = target;
    SmartDashboard.putNumber("deriv: ", deriv);

    if (deriv >= 0) {
      newTarget = target + (deriv * 125);
    } else {
      newTarget = target + (deriv * 35);
    }

    // SmartDashboard.putNumber("shootSpeedAdjust", (deriv * 125));
    return newTarget;
  }

  public boolean isAtSpeed() {
    if (velocityTarget == 0) {
      return false;
    }
    double error = velocityTarget - this.getVelocity();
    SmartDashboard.putNumber("error", error);
    return Math.abs(error) <= 45;
  }

  public double getShooterVelocity() {
    return shooterLead.get() / (2 / 3 / shooterToMotorRPM);
  }

  /**
   * gets shoot goal as determined by our quartic regression, using limelight
   * y-value
   * 
   * @return shootGoal
   */
  public double getShootGoal() {
    double limedY = myLimelight.getY();
    return ((a * Math.pow(limedY, 4)) + (b * Math.pow(limedY, 3) + (c * Math.pow(limedY, 2)) + (d * limedY) + e))
        * 1.00;
  }
}
