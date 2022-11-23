package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Constants {
    public static class DriveConstants {
        public static final String leftFrontName = "fleft";
        public static final String leftBackName = "bleft";
        public static final String rightFrontName = "fright";
        public static final String rightBackName = "bright";

        public static final DcMotor.Direction leftDriveDirection = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction rightDriveDirection = DcMotor.Direction.REVERSE;

        public static final DcMotor.ZeroPowerBehavior driveBrakeOnInit = DcMotor.ZeroPowerBehavior.FLOAT;

        public static class DrivePIDF {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.0;
        }

        // Speed Scaling
        public static double slowScale = 0.2;
        public static double normalScale = 0.4;
        public static double fastScale = 0.75;


        // Meters  (W: 0.13385, L: 0.1225)
        private static final double halfWidth = 0.13385;
        private static final double halfLength = 0.1225;

        public static final Translation2d frontLeftTranslation = new Translation2d(-halfWidth, halfLength);
        public static final Translation2d frontRightTranslation = new Translation2d(halfWidth, halfLength);
        public static final Translation2d backLeftTranslation = new Translation2d(-halfWidth, -halfLength);
        public static final Translation2d backRightTranslation = new Translation2d(halfWidth, -halfLength);

    }

    public static class LiftConstants {
        public static final String leftWinchName = "lWinch";
        public static final DcMotorEx.Direction leftDirection = DcMotorEx.Direction.FORWARD;

        public static final String rightWinchName = "rWinch";
        public static final DcMotorEx.Direction rightDirection = DcMotorEx.Direction.REVERSE;

        public static DcMotorEx.ZeroPowerBehavior zeroPowerBehavior = DcMotorEx.ZeroPowerBehavior.BRAKE;


        public static class LiftController {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.0;
            public static double tolerance = 10.0;
        }

        public static int maximumTravel = 100;
        public static int minimumTravel = 0;

        public static int positionChangeScale = 10;

        public static int downPosition = 0;
        public static int lowPosition = 10;
        public static int mediumPosition = 30;
        public static int highPosition = 50;
    }

    public static class LiftClawConstants {
        public static final String name = "clawServo";
        public static final boolean inverted = false;
        public static final double min = 0.0;
        public static final double max = 1.0;

        public static final double openPosition = 0.0;
        public static final double closedPosition = 0.0;
    }



    public static class IMUConstants {
        public static final String deviceName = "imu";
    }

    public static class ControlConstants {
        public static final double driveDeadBan = 0.2;
        public static final double rotationDeadBan = 0.15;

        public static final double liftDeadBan = 0.2;
    }


    public static final CurrentUnit currentUnit = CurrentUnit.MILLIAMPS;
}
