// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class AutoConstants {

        private static final double xOffsetTerminal5Ball = -0.13;
        private static final double yOffsetTerminal5Ball = -0.13;

        public static double getXOffsetTerminal5Ball() {
            if(fieldType == FieldType.COMP) {
                return xOffsetTerminal5Ball;
            }
            return 0.0;
        }

        public static double getYOffsetTerminal5Ball() {
            if(fieldType == FieldType.COMP) {
                return yOffsetTerminal5Ball;
            }
            return 0.0;
        }

        //values for Ramsete controller
        public static final double ksVolts = 0.59619;
        public static final double kvVoltSecondsPerMeter = 1.2895;
        public static final double kaVoltSecondsSquaredPerMeter = 0.16441;
        public static final double kPDriveVel = 1.7177;
        public static final double kTrackwidthMeters = 0.641;

        public static final double kMaxSpeedMetersPerSecondHigh = 2.0;
        public static final double kMaxAccelerationMetersPerSecondSquaredHigh = 2.0;

        public static final double kMaxSpeedMetersPerSecondLow = 1.0;
        public static final double kMaxAccelerationMetersPerSecondSquaredLow = 1.0;

        //ramsete values - 2,.7 are default
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;

        public static final boolean kGyroReversed = true; //Characterization says this isn't necessary but it seems to perform better....
    }

    public static final class CANIds{
        public static final int motorLLeadID = 1;
        public static final int motorLFollow1ID = 2;
        public static final int motorLFollow2ID = 3;
        public static final int motorRLeadID = 4;
        public static final int motorRFollow1ID = 5;
        public static final int motorRFollow2ID = 6;
        public static final int turretMotorID = 11;
        public static final int shooterLeadId = 7;
        public static final int shooterFollowId = 9;
        public static final int intakeId = 10;
        public static final int feederId = 12;
        public static final int towerId = 8;
        public static final int climbLeftId = 13;
        public static final int climbRightId = 14;
    }

    public static final class OIConstants {
        public static final int driverContPort = 0;
        public static final int operatorContPort = 1;
        public static final double xboxDeadzone = .125;
    }

    public static final class PneumaticConstants {
        public static final int intakeForwardChannel = 0;
        public static final int intakeReverseChannel = 1;
    }

    public static final class DigitalInputPorts {
        public static final int topTowerSensor = 0;
        public static final int bottomTowerSensor = 2;
        public static final int feederSensor = 3;
        public static final int middleLimitSwitchPort = 1;
    }

    public enum FieldType {
        COMP, PRACTICE
    }

    public static final FieldType fieldType = FieldType.COMP;

}