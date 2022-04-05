// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.CANIds.*;
//import frc.robot.Constants.AutoConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

//import static frc.robot.Constants.DriveTrainConstants.*;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  public double offsetAngle = 0;
  private double driveOffset = 0;

  // Conversions for the Falcons
  private static final double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
  public static final double ticksToMeters = ((pi * 0.1524) * ((15.0 / 85.0) * (24.0 / 46.0) / 2048.0));

  private WPI_TalonFX motorLLead = new WPI_TalonFX(motorLLeadID);
  private WPI_TalonFX motorLFollow1 = new WPI_TalonFX(motorLFollow1ID);
  private WPI_TalonFX motorLFollow2 = new WPI_TalonFX(motorLFollow2ID);
  private WPI_TalonFX motorRLead = new WPI_TalonFX(motorRLeadID);
  private WPI_TalonFX motorRFollow1 = new WPI_TalonFX(motorRFollow1ID);
  private WPI_TalonFX motorRFollow2 = new WPI_TalonFX(motorRFollow2ID);

  private static DriveTrain instance;

  public final DifferentialDrive m_differentialDrive;

  public AHRS navX = new AHRS(SPI.Port.kMXP); // For frc-characterization tool: "SPI.Port.kMXP" of type "NavX"
  public final static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
      AutoConstants.kTrackwidthMeters);
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getHeading());
  public final static SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(AutoConstants.ksVolts,
      AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter);
  private Pose2d pose;

  private PIDController leftPidController = new PIDController(AutoConstants.kPDriveVel, 0, 0);
  private PIDController rightPidController = new PIDController(AutoConstants.kPDriveVel, 0, 0);

  private final MotorControllerGroup leftGroup;
  private final MotorControllerGroup rightGroup;

  private static double ACCELERATION_LIMIT = 1.5;

  private double pastForwardSpeed = 0;

  private StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true, 40, 40, 0.01);
  private SlewRateLimiter driveLLimiter = new SlewRateLimiter(1.0);
  private SlewRateLimiter driveRLimiter = new SlewRateLimiter(1.0);

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {

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

    // motorLLead.configStatorCurrentLimit(statorLimit);
    // motorLFollow1.configStatorCurrentLimit(statorLimit);
    // motorLFollow2.configStatorCurrentLimit(statorLimit);
    // motorRLead.configStatorCurrentLimit(statorLimit);
    // motorRFollow1.configStatorCurrentLimit(statorLimit);
    // motorRFollow2.configStatorCurrentLimit(statorLimit);

    resetEncPos(); // Reset Encoders r navX yaw before m_odometry is defined

    // makes the 3 motor controllers function as 1 motor controller for autos
    leftGroup = new MotorControllerGroup(motorLLead, motorLFollow1, motorLFollow2);
    rightGroup = new MotorControllerGroup(motorRLead, motorRFollow1, motorRFollow2);

    m_differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    m_differentialDrive.setSafetyEnabled(false); // So it won't stop the motors from moving

    this.coastMode();
  }

  public static DriveTrain getInstance(){
    if(instance == null){
      instance = new DriveTrain();
    }
    return instance;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    motorLLead.setVoltage(leftVolts); // Answer is no //Set to motor groups
    motorRLead.setVoltage(rightVolts); // it's big brain time
    m_differentialDrive.feed(); // Feed the motorsafety class so it doesnt disable the motors

  }

  public void resetEncPos() { // For initialization resets encoder positions, for ramsete
    motorLLead.setSelectedSensorPosition(0);
    motorRLead.setSelectedSensorPosition(0);
    navX.zeroYaw();
    navX.setAngleAdjustment(0); // Set angle offset
    offsetAngle = 0;
    m_odometry.resetPosition(new Pose2d(), getHeading()); // Set odomentry to zero
    System.out.println("reset");
  }

  public void angleAdjust(double adjust){
    navX.reset();
    navX.setAngleAdjustment(adjust);
    System.out.println("angle set");
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  public void setDriveOffset(double offset){
    driveOffset = offset + 90;
  }

  public void implementOffset(){
    offsetAngle = driveOffset;
    System.out.println("offsetting to: " + offsetAngle);
  }

  public double getHeadingAngle() {
    return Math.IEEEremainder(navX.getAngle(), 360) + offsetAngle;
  }

  public double getYaw() {
    return navX.getYaw();
  }

  public void setAngleOffset(double offset) {
    navX.setAngleAdjustment(offset);
  }

  public void setOffset(){
    navX.setAngleAdjustment(offsetAngle);
  }

  public static DifferentialDriveKinematics getKinematics() {
    return kDriveKinematics;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedForward;
  }

  public PIDController getLeftController() {
    return leftPidController;
  }

  public PIDController getRightController() {
    return rightPidController;
  }

  public double getLeftEncoderMeters() {
    return motorLLead.getSelectedSensorPosition() * ticksToMeters;
  }

  public double getRightEncoderMeters() {
    return motorRLead.getSelectedSensorPosition() * ticksToMeters;
  }

  public double getLeftEncoderMetersPerSec() {
    return motorLLead.getSelectedSensorVelocity() * ticksToMeters;
  }

  public double getRightEncoderMetersPerSec() {
    return motorRLead.getSelectedSensorVelocity() * ticksToMeters;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() { // Must be in meters/second
    // In example: m_leftEncoder.getRate() , m_rightEncoder.getRate() however, they
    // set their rate to inclue their conversions
    return new DifferentialDriveWheelSpeeds(motorLLead.getSelectedSensorVelocity() * ticksToMeters,
        motorRLead.getSelectedSensorVelocity() * ticksToMeters);
  }

  public double getAccelerationLimit() {
    return ACCELERATION_LIMIT;
  }

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
    // SmartDashboard.putString("Pos2D", m_odometry.getPoseMeters().toString());
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
    pose = m_odometry.update( // Must be in meters according to internets
        getHeading(), motorLLead.getSelectedSensorPosition() * ticksToMeters,
        motorRLead.getSelectedSensorPosition() * ticksToMeters);
    navxTestingDashboardReadouts();
    // System.out.println("distance x: " +
    // Units.metersToFeet(m_odometry.getPoseMeters().getX()));
    // System.out.println("distance y: " +
    // Units.metersToFeet(m_odometry.getPoseMeters().getY()));
    // System.out.println("Right encoder: " +
    // Units.metersToFeet(motorLLead.getSelectedSensorPosition() *
    // AutoConstants.ticksToMeters));
    // System.out.println("Left encoder" +
    // Units.metersToFeet(motorRLead.getSelectedSensorPosition() *
    // AutoConstants.ticksToMeters));
    
    SmartDashboard.putNumber("robot heading", getHeadingAngle());

    // SmartDashboard.putNumber("RL current", motorRLead.getSupplyCurrent());
    // SmartDashboard.putNumber("RF1 current", motorRFollow1.getSupplyCurrent());
    // SmartDashboard.putNumber("RF2 current", motorRFollow2.getSupplyCurrent());
    // SmartDashboard.putNumber("LL current", motorLLead.getSupplyCurrent());
    // SmartDashboard.putNumber("LF1 current", motorLFollow1.getSupplyCurrent());
    // SmartDashboard.putNumber("LF2 current", motorLFollow2.getSupplyCurrent());
  }

  public void positionPrintouts() {
    double leftRawPos = motorLLead.getSelectedSensorPosition();
    SmartDashboard.putNumber("Left Raw Pos", leftRawPos);
    double rightRawPos = motorRLead.getSelectedSensorPosition();
    SmartDashboard.putNumber("Right Raw Pos", rightRawPos);
  }

  public void drive(double leftMotorPercentage, double rightMotorPercentage) {
    // if(leftMotorPercentage == 0){
      // motorLLead.stopMotor();
    // } else {
      leftMotorPercentage = driveLLimiter.calculate(leftMotorPercentage);
      motorLLead.set(leftMotorPercentage);
    // }
    // if(rightMotorPercentage == 0){
        // motorRLead.stopMotor();
    // } else {
      rightMotorPercentage = driveRLimiter.calculate(rightMotorPercentage);
      motorRLead.set(rightMotorPercentage);
    // }
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
