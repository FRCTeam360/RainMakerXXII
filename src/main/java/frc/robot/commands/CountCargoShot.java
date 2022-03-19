// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoCounter;

public class CountCargoShot extends CommandBase {
  private final CargoCounter mCargoCounter = CargoCounter.getInstance();
  Integer initialShotCount, cargoToShoot;

  /** Creates a new CountCargoShot. */
  public CountCargoShot(Integer cargoToShoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cargoToShoot = cargoToShoot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initialShotCount = mCargoCounter.getShotCount();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public boolean cargoToShootReached(){
    return this.initialShotCount + this.cargoToShoot >= mCargoCounter.getShotCount();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cargoToShootReached();
  }
}
