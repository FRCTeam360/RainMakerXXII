// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;

//WHEN CALLING THIS METHOD: MoveWithRamsete(trajectory, drivetrain).andThen(() -> drivetrain.tankDriveVolts(0,0));

/** Add your docs here. */
public class MoveWithRamsete extends RamseteCommand {

    public MoveWithRamsete (Trajectory traj, DriveTrain drivetrain) {
        super(
            traj, 
            drivetrain::getPose, 
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(AutoConstants.kPDriveVel, 0, 0),
            new PIDController(AutoConstants.kPDriveVel, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain
        ); 
    }
}
