// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class ShooterJoy extends CommandBase {

    private final Shooter shooter;
    private final OperatorControl operatorCont;

public ShooterJoy(Shooter shooter) {
    this.shooter = shooter;
    operatorCont = OperatorControl.getInstance();
    addRequirements(shooter);
}

@Override   // Called when the command is initially scheduled.
public void initialize() {
}

@Override   // Called every time the scheduler runs while the command is scheduled.
public void execute() {
    // if (cont.getXButton()){
        shooter.setSpeed(operatorCont.getLeftY() * 0.45 ); 
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