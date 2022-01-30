// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class LimelightConstants {
        public static final double AimMinCmd = 0.01;
        public static final double kP = 0.6;
    }

    public static final class CANIds{
        public static final int turretMotorID = 2;
        public static final int intakeId = 11;
        public static final int feederId = 10;
        public static final int towerId = 12;
    }

    public static final class OIConstants {
        public static final int driverContPort = 0;
        public static final int operatorContPort = 1;
    }

    public static final class PneumaticConstants {
        public static final int intakeForwardChannel = 0;
        public static final int intakeReverseChannel = 1;
    }

    public static final class DigitalInputPorts {
        public static final int toplimitSwitchPort = 0;
        public static final int bottomlimitSwitchPort = 1;
    }
}


