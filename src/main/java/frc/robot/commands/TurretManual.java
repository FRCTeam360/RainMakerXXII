// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.OIConstants.*;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.DriveTrain;
import frc.robot.operatorInterface.OperatorControl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class TurretManual extends CommandBase {
  private OperatorControl operatorCont;
  private Turret myTurret;
  private DriveTrain myDriveTrain;

  private enum ControlTypes {
    POWER_CONTROL, POSITION_CONTROL, FIELD_ORIENTED_CONTROL
  };

  private ControlTypes controlTypes;

  /** Creates a new TurretManual. */
  public TurretManual(DriveTrain driveTrain) {

    myDriveTrain = driveTrain;
    myTurret = Turret.getInstance();
    operatorCont = OperatorControl.getInstance();
    addRequirements(myTurret);
    // Use addRequirements() here to declare subsystem dependencies.

    this.controlTypes = ControlTypes.POWER_CONTROL;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(operatorCont.getDPadRight()){
      myTurret.resetEncoderTicks();
    }

    this.changeMode();

    switch (this.controlTypes) {
      case POSITION_CONTROL:
        this.positionControl();
        break;

      case FIELD_ORIENTED_CONTROL:
        this.fieldOrientedControl();
        break;

      case POWER_CONTROL:
      default:
        this.powerControl();
        break;

    }
    double encoderTick = myTurret.getEncoderTick();
    SmartDashboard.putNumber("gettick", encoderTick);
  }

  private void changeMode() {
    if (operatorCont.getDPadLeft()) {
      this.controlTypes = ControlTypes.POSITION_CONTROL;
    } else if (operatorCont.getDPadDown()) {
      this.controlTypes = ControlTypes.POWER_CONTROL;
    }
  }

  private void powerControl() {
    if (Math.abs(operatorCont.getRightX()) > .125) {
      myTurret.turn(operatorCont.getRightX());
    } else {
      myTurret.turn(0);
    }
  }

  private void positionControl() {
    double x = 0;
    double y = 0;
    if (Math.abs(operatorCont.getRightX()) > .125 || Math.abs(operatorCont.getRightY()) > .125) {
      x = -operatorCont.getRightX();
      y = -operatorCont.getRightY();
      double angle = Math.atan(x / y); // Math.abs()
      angle = Math.toDegrees(angle);
      // System.out.println("controller angle: " + angle);

      if (y < 0 && x > 0) {
        angle = 180 + angle; // -
      } else if (y < 0 && x < 0) {
        angle = -180 + angle;
      }

      myTurret.angleTurn(angle);
    } else {
      myTurret.turn(0);
    }

    // else if (y > 0 && x < 0) {
    // angle = -1 * angle;
    // }
    // else if(y>0 && x>0) {

    // }
  }

  private void fieldOrientedControl() {

    double gyroAngle = myDriveTrain.getYaw();

    double x = operatorCont.getRightX();
    double y = operatorCont.getRightY();
    double angle = Math.abs(Math.atan(x / y));
    if (y < 0 && x > 0) {
      angle = 180 - angle;
    } else if (y < 0 && x < 0) {
      angle = -180 + angle;
    } else if (y > 0 && x < 0) {
      angle = -1 * angle;
    }
    // else if(y>0 && x>0) {

    // }
    double turretAngle = angle - gyroAngle;
    myTurret.angleTurn(turretAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myTurret.turn(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
