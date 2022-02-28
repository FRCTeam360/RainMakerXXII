// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CANIds.*;
//import frc.robot.Constants.AutoConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

//import static frc.robot.Constants.DriveTrainConstants.*;
import frc.robot.Constants.AutoConstants;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.kauailabs.navx.frc.AHRS; //If error here check updates: install vendor online use: https://www.kauailabs.com/dist/frc/2021/navx_frc.json
import edu.wpi.first.wpilibj.SPI; //Port NavX is on

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode.*;

public class DriveTrain extends SubsystemBase {
  private static WPI_TalonFX motorLLead;
  private static WPI_TalonFX motorLFollow1;
  private static WPI_TalonFX motorLFollow2;
  private static WPI_TalonFX motorRLead;
  private static WPI_TalonFX motorRFollow1;
  private static WPI_TalonFX motorRFollow2;

  private final DifferentialDrive m_differentialDrive;

  // private double leftVel; // initializes velocities for left and right sides
  // private double rightVel;
  // private double leftNewPos; // initializes new positions for left and right
  // sides
  // private double rightNewPos;

  private AHRS navX;
  private final DifferentialDriveOdometry m_odometry;
  private final MotorControllerGroup leftGroup;
  private final MotorControllerGroup rightGroup;

  public static double ACCELERATION_LIMIT = 2;

  private double pastForwardSpeed = 0;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    motorLLead = new WPI_TalonFX(motorLLeadID);
    motorLFollow1 = new WPI_TalonFX(motorLFollow1ID);
    motorLFollow2 = new WPI_TalonFX(motorLFollow2ID);
    motorRLead = new WPI_TalonFX(motorRLeadID);
    motorRFollow1 = new WPI_TalonFX(motorRFollow1ID);
    motorRFollow2 = new WPI_TalonFX(motorRFollow2ID);

    motorLLead.configFactoryDefault();
    motorLFollow1.configFactoryDefault();
    motorLFollow2.configFactoryDefault();
    motorRLead.configFactoryDefault();
    motorRFollow1.configFactoryDefault();
    motorRFollow2.configFactoryDefault();

    // makes the second motor for left and right sides to follow the primary motor
    // on the left and right
    motorLFollow1.follow(motorLLead);
    motorLFollow2.follow(motorLLead);
    motorRFollow1.follow(motorRLead);
    motorRFollow2.follow(motorRLead);

    // makes one side of the robot reverse direction in order to ensure that the
    // robot goes forward when the joysticks are both forward and backwards when the
    // joysticks are both backwards
    motorLLead.setInverted(TalonFXInvertType.CounterClockwise);
    motorLFollow1.setInverted(TalonFXInvertType.FollowMaster);
    motorLFollow2.setInverted(TalonFXInvertType.FollowMaster);
    motorRLead.setInverted(TalonFXInvertType.Clockwise);
    motorRFollow1.setInverted(TalonFXInvertType.FollowMaster);
    motorRFollow2.setInverted(TalonFXInvertType.FollowMaster);

    navX = new AHRS(SPI.Port.kMXP); // For frc-characterization tool: "SPI.Port.kMXP" of type "NavX"
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    resetEncPos(); // Reset Encoders r navX yaw before m_odometry is defined

    // makes the 3 motor controllers function as 1 motor controller for autos
    leftGroup = new MotorControllerGroup(motorLLead, motorLFollow1, motorLFollow2);
    rightGroup = new MotorControllerGroup(motorRLead, motorRFollow1, motorRFollow2);

    m_differentialDrive = new DifferentialDrive(motorLLead, motorRLead);
    m_differentialDrive.setSafetyEnabled(false); // So it won't stop the motors from moving

    this.brakeMode();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts); // Answer is no //Set to motor groups
    rightGroup.setVoltage(rightVolts); // it's big brain time
    m_differentialDrive.feed(); // Feed the motorsafety class so it doesnt disable the motors
  }

  public void resetEncPos() { // For initialization resets encoder positions, for ramsete
    motorLLead.setSelectedSensorPosition(0);
    motorRLead.setSelectedSensorPosition(0);
    navX.zeroYaw();
    navX.setAngleAdjustment(-navX.getAngle()); // Set angle offset
    m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading())); // Set odomentry to zero
  }

  public double getHeading() {
    return Math.IEEEremainder(navX.getAngle(), 360) * (AutoConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getYaw() {
    return navX.getYaw();
  }

  /**
   * Sets the lead motors to the percentage given.
   * 
   * @param leftMotorPercentage
   * @param rightMotorPercentage
   */
  public void drive(double leftMotorPercentage, double rightMotorPercentage) {
    if (leftMotorPercentage > 1) {
      leftMotorPercentage = 1;
    } else if (leftMotorPercentage < -1) {
      leftMotorPercentage = -1;
    }

    if (rightMotorPercentage > 1) {
      rightMotorPercentage = 1;
    } else if (rightMotorPercentage < -1) {
      rightMotorPercentage = -1;
    }

    motorLLead.set(TalonFXControlMode.PercentOutput, leftMotorPercentage);
    motorRLead.set(TalonFXControlMode.PercentOutput, rightMotorPercentage);

  }

  // public Pose2d getPose() {
  // return m_odometry.getPoseMeters();
  // }

  public void brakeMode() {
    motorLLead.setNeutralMode(NeutralMode.Brake);
    motorLFollow1.setNeutralMode(NeutralMode.Brake);
    motorLFollow2.setNeutralMode(NeutralMode.Brake);
    motorRLead.setNeutralMode(NeutralMode.Brake);
    motorRFollow1.setNeutralMode(NeutralMode.Brake);
    motorRFollow2.setNeutralMode(NeutralMode.Brake);
  }

  public void coastMode() {
    motorLLead.setNeutralMode(NeutralMode.Coast);
    motorLFollow1.setNeutralMode(NeutralMode.Coast);
    motorLFollow2.setNeutralMode(NeutralMode.Coast);
    motorRLead.setNeutralMode(NeutralMode.Coast);
    motorRFollow1.setNeutralMode(NeutralMode.Coast);
    motorRFollow2.setNeutralMode(NeutralMode.Coast);
  }

  public void navxTestingDashboardReadouts() {
    // SmartDashboard.putNumber("N ang", Math.IEEEremainder(navX.getAngle(), 360) );
    SmartDashboard.putNumber("NAV ang", navX.getAngle());
    SmartDashboard.putString("Pos2D", m_odometry.getPoseMeters().toString());
    // System.out.print("NavX angle: " + navX.getAngle());
    // SmartDashboard.putNumber("N pre", navX.getBarometricPressure()); //why this
    // no work cri, just tryna get the pressure
    SmartDashboard.putNumber("N yaw", navX.getYaw());

    // SmartDashboard.putBoolean("NAVC con", navX.isConnected());
    // SmartDashboard.putBoolean("NAV cal", navX.isCalibrating());
  }

  // public double getHighestVelocity () {
  // double leftSpeed =
  // motorLLead.TalonFXSensorCollection.getIntegratedSensorVelocity() *
  // AutoConstants.ticksToMeters;
  // double rightSpeed = motorRLead.getEncoder().getVelocity() *
  // AutoConstants.ticksToMeters;
  // double highSpeed = Math.max( Math.abs(leftSpeed), Math.abs(rightSpeed) );
  // //Make em both positive
  // return highSpeed; //In meters per second
  // 5}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update( // Must be in meters according to internets
        Rotation2d.fromDegrees(getHeading()), motorLLead.getSelectedSensorPosition() * AutoConstants.ticksToMeters,
        motorRLead.getSelectedSensorPosition() * AutoConstants.ticksToMeters);
    navxTestingDashboardReadouts();
  }

  public void positionPrintouts() {
    double leftRawPos = motorLLead.getSelectedSensorPosition();
    SmartDashboard.putNumber("Left Raw Pos", leftRawPos);
    double rightRawPos = motorRLead.getSelectedSensorPosition();
    SmartDashboard.putNumber("Right Raw Pos", rightRawPos);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getForwardSpeed() {
    double forward = (motorLLead.getSelectedSensorVelocity() + motorRLead.getSelectedSensorVelocity()) / 2;
    System.out.println("forward: " + forward);
    return forward;
  }

  public double getAcceleration() {
    double forwardSpeed = this.getForwardSpeed();
    double acceleration = forwardSpeed - this.pastForwardSpeed;
    this.pastForwardSpeed = forwardSpeed;
    return acceleration;
  }

  public Boolean isAccelerating() {
    return (this.getForwardSpeed() > 0 && this.getAcceleration() > 0)
        || (this.getForwardSpeed() < 0 && this.getAcceleration() < 0);
  }
}
