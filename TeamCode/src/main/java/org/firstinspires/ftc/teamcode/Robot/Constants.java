package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Constants {
    public static class DriveConstants {
        public static final String leftFrontName = "leftFrontDrive";
        public static final String leftBackName = "leftBackDrive";
        public static final String rightFrontName = "rightFrontDrive";
        public static final String rightBackName = "rightBackDrive";

        public static final DcMotor.Direction leftDriveDirection = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction rightDriveDirection = DcMotor.Direction.REVERSE;

        public static final DcMotor.ZeroPowerBehavior driveBrakeOnInit = DcMotor.ZeroPowerBehavior.BRAKE;
    }

    public static class ArmConstants {
        public static final String joint1Name = "armJoint1";
        public static final String joint2Name = "armJoint2";

        public static final DcMotor.Direction joint1Direction = DcMotorSimple.Direction.REVERSE;
        public static final DcMotor.Direction joint2Direction = DcMotorSimple.Direction.FORWARD;

        public static final DcMotor.ZeroPowerBehavior jointZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

        public static class Joint1Controller {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.0;

            public static final double tolerance = 5.0;
        }

        public static class Joint2Controller {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.0;

            public static final double tolerance = 5.0;
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
        public static double stickDeadzone = 0.2;
    }

    public static final CurrentUnit currentUnit = CurrentUnit.MILLIAMPS;
}
