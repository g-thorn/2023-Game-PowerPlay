package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
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

        public static final PIDFCoefficients drivePIDFCoefficients = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);

        public static class DrivePIDF {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.0;
        }

        // Meters  (W: 0.13385, L: 0.1225)
        private static final double halfWidth = 0.13385;
        private static final double halfLength = 0.1225;

        public static final Translation2d frontLeftTranslation = new Translation2d(-halfWidth, halfLength);
        public static final Translation2d frontRightTranslation = new Translation2d(halfWidth, halfLength);
        public static final Translation2d backLeftTranslation = new Translation2d(-halfWidth, -halfLength);
        public static final Translation2d backRightTranslation = new Translation2d(halfWidth, -halfLength);

    }

    public static class ArmConstants {
        public static final String joint1Name = "armJoint1";
        public static final String joint2Name = "armJoint2";

        public static final DcMotor.Direction joint1Direction = DcMotorSimple.Direction.REVERSE;
        public static final DcMotor.Direction joint2Direction = DcMotorSimple.Direction.FORWARD;

        public static final DcMotor.ZeroPowerBehavior jointZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

        public static final int joint1Max = 300;
        public static final int joint1Min = -50;

        public static class Joint1Controller {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.0;

            public static final int tolerance = 10;
        }

        public static final int joint2Max = 300;
        public static final int joint2Min = -50;

        public static class Joint2Controller {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.0;

            public static final int tolerance = 10;
        }
    }

    public static class ServoConstants {
        public static class ClawServo {
            public static final String deviceName = "clawServo";

            public static final double min = 0.0;
            public static final double max = 0.0;

            public static final Servo.Direction direction = Servo.Direction.FORWARD;
        }

        public static class XServo {
            public static final String deviceName = "xServo";

            public static final double min = 0.0;
            public static final double max = 0.0;

            public static final Servo.Direction direction = Servo.Direction.FORWARD;
        }

        public static class ZServo {
            public static final String deviceName = "zServo";

            public static final double min = 0.0;
            public static final double max = 0.0;

            public static final Servo.Direction direction = Servo.Direction.FORWARD;
        }
    }

    public static class IMUConstants {
        public static final String deviceName = "imu";
    }

    public static class ControlConstants {
        public static double driverDeadzone = 0.2;
        public static double armDeadzone = 0.2;

    }

    public static final CurrentUnit currentUnit = CurrentUnit.MILLIAMPS;
}
