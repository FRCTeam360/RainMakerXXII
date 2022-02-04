// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final class ShooterConstants {

        // Old data, need to tune
        public static final int kSlotIdx = 0;
        public static final int kTimeOutMs = 30;
        public static final int kPIDLoopIdx = 0;
        // public static final double kP = 0.1;// ((0.05 * 1023.0) / 1001.0) * 7.5 * 2.5; //(0.05 * 1023.0) / 453.0;
        // public static final double kI = 0;
        // public static final double kD = 0.0;
        // public static final double kF = 0.0; //(0.6 * 1023.0) / 15900.0;
        public static final double kPeakOutput = 1;

        public static final double backupTargetVelocity = 14500; // Constant
        public static double targetVelocity = backupTargetVelocity; // will get changed in the future by limelight
                                                                    // subsystem or a command...

        public static final double aVal = 2.697; // Quad Ratic regression values
        public static final double bVal = -52.912;
        public static final double cVal = 14815.146;
    }
    
    public static final class CANIds{
        public static final int shooterLeadId = 7;
        public static final int shooterFollowId = 9;
        public static final int intakeId = 10;
        public static final int feederId = 11;
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
}
