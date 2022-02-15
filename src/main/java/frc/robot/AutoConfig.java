// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;

/** Add your docs here. */
public class AutoConfig {
    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            AutoConstants.ksVolts, 
            AutoConstants.kvVoltSecondsPerMeter, 
            AutoConstants.kaVoltSecondsSquaredPerMeter
        ), 
        AutoConstants.kDriveKinematics, 
        10
    );
    public static final TrajectoryConfig configFwd = 
        new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
        .setKinematics(AutoConstants.kDriveKinematics) // Add kinematics to ensure max speed is actually obeyed
        .addConstraint(autoVoltageConstraint) // Apply the voltage constraint
        .setReversed(false); //forward

        public static final TrajectoryConfig configRev =
        new TrajectoryConfig(
         AutoConstants.kMaxSpeedMetersPerSecond,
         AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
        .setKinematics(AutoConstants.kDriveKinematics) // Add kinematics to ensure max speed is actually obeyed
        .addConstraint(autoVoltageConstraint) // Apply the voltage constraint
        .setReversed(true); //reversed
}
