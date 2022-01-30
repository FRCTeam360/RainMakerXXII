// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.ShooterConstants.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class ShooterJoy extends CommandBase {

    private final Shooter shooter;
    private final DriverControl driverCont;
    private final OperatorControl operatorCont;

public ShooterJoy(Shooter shooter) {
    this.shooter = shooter;
    driverCont = DriverControl.getInstance();
    operatorCont = OperatorControl.getInstance();
    addRequirements(shooter);
}

@Override   // Called when the command is initially scheduled.
public void initialize() {
}

@Override   // Called every time the scheduler runs while the command is scheduled.
public void execute() {
    // if (cont.getXButton()){
        shooter.setSpeed(operatorCont.getLeftY() * 1.0 ); 
    // } else{
    //     shooter.setVelocity(cont.getLeftY() * 5500);
    // }
}

@Override   // Called once the command ends or is interrupted.
public void end(boolean interrupted) {
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}