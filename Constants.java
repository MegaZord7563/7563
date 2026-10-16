package frc.robot;

public final class Constants {
    public static final class DriveConstants {
        // Left side motor IDs
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;

        // Right side motor IDs
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        // Encoder ports (A, B)
        public static final int[] kLeftEncoderPorts = {0, 1};
        public static final int[] kRightEncoderPorts = {2, 3};

        // Invert right encoder so both read positive when driving forward
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        // Distance per encoder pulse (example: 0.1524 m per pulse)
        public static final double kEncoderDistancePerPulse = 0.1524;

        // Track width in meters – used for odometry
        public static final double kTrackWidthMeters = 0.7;
    }

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
}