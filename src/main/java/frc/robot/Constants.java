// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        
        // https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/identification-routine.html#running-the-identification-routine
        public static final double kTrackwidthMeters = 0.69; // TODO CHANGE
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    
        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
    
        public static final double ksVolts = 0.22; // TODO CHANGE
        public static final double kvVoltSecondsPerMeter = 1.98; // TODO CHANGE
        public static final double kaVoltSecondsSquaredPerMeter = 0.2; // TODO CHANGE
    
        public static final double kPDriveVel = 8.5; // TODO CHANGE

        private static final double IN_TO_M = .0254;
  
        public static final int MOTOR_ENCODER_CODES_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
        private static final double DIAMETER_INCHES = 6.0; // wheel diameter
        private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
        private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double GEAR_RATIO = 10.71;
        public static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
        public static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;
    }
    
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3; // TODO CHANGE
        public static final double kMaxAccelerationMetersPerSecondSquared = 1; // TODO CHANGE
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object 
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
