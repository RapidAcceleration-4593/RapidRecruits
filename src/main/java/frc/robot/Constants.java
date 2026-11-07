package frc.robot;

public class Constants {

    public static class Drivetrain {
        // 3 meters per second.
        public static final double kMaxSpeed = 3.0; // Meters per second

        // 1/2 rotation per second.
        public static final double kMaxAngularSpeed = Math.PI; // Radians per second

        public static final double kTrackWidth = 0.381 * 2;
        public static final double kWheelRadius = 0.0508;
        public static final double kDriveGearing = 4.0; // Output over input
        public static final int kEncoderResolution = -4096;

        public static final int leftLeaderId = 1;
        public static final int leftFollowerId = 2;
        public static final int rightLeaderId = 3;
        public static final int rightFollowerId = 4;

        public static final int leftEncoderAId = 0;
        public static final int leftEncoderBId = 1;
        public static final int rightEncoderAId = 2;
        public static final int rightEncoderBId = 3;

        public static final int gyroId = 0;

        public static final double kP = 8.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 1.0;
        public static final double kV = 3.0;
    }
}
