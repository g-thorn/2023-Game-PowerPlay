package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants.frontLeftTranslation;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants.frontRightTranslation;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants.backLeftTranslation;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants.backRightTranslation;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Lib.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants.DrivePIDF;

public class Drive extends Subsystem {

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;

    private PIDFController leftFrontController;
    private PIDFController leftBackController;
    private PIDFController rightFrontController;
    private PIDFController rightBackController;

    private MecanumDriveKinematics mDriveKinematics;
    private ChassisSpeeds mChassisSpeeds;

    private boolean isFastMode = false;
    private boolean isSlowMode = false;

    private boolean isOpenLoop = false;

    public Drive(OpMode op) {

        // Left Front Drive
        leftFrontDrive = (DcMotorEx) op.hardwareMap.get(DcMotor.class, DriveConstants.leftFrontName);
        leftFrontDrive.setDirection(DriveConstants.leftDriveDirection);
        leftFrontDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);

        leftFrontController = new PIDFController(DrivePIDF.kP, DrivePIDF.kI, DrivePIDF.kD, DrivePIDF.kI);

        // Left Back Drive
        leftBackDrive = (DcMotorEx) op.hardwareMap.get(DcMotor.class, DriveConstants.leftBackName);
        leftBackDrive.setDirection(DriveConstants.leftDriveDirection);
        leftBackDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);

        leftBackController = new PIDFController(DrivePIDF.kP, DrivePIDF.kI, DrivePIDF.kD, DrivePIDF.kI);


        // Right Front Drive
        rightFrontDrive = (DcMotorEx) op.hardwareMap.get(DcMotor.class, DriveConstants.rightFrontName);
        rightFrontDrive.setDirection(DriveConstants.rightDriveDirection);
        rightFrontDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);

        rightFrontController = new PIDFController(DrivePIDF.kP, DrivePIDF.kI, DrivePIDF.kD, DrivePIDF.kI);

        // Right Back Drive
        rightBackDrive = (DcMotorEx) op.hardwareMap.get(DcMotor.class, DriveConstants.rightBackName);
        rightBackDrive.setDirection(DriveConstants.rightDriveDirection);
        rightBackDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);

        rightBackController = new PIDFController(DrivePIDF.kP, DrivePIDF.kI, DrivePIDF.kD, DrivePIDF.kI);

        // Kinematics
        mDriveKinematics = new MecanumDriveKinematics(frontLeftTranslation, frontRightTranslation, backLeftTranslation, backRightTranslation);
    }

    @Override
    public void initAction() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void enable() {

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.leftFrontPosition = leftFrontDrive.getCurrentPosition();
        mPeriodicIO.leftFrontVelocity = leftFrontDrive.getVelocity();
        mPeriodicIO.leftFrontCurrent = leftFrontDrive.getCurrent(Constants.currentUnit);

        mPeriodicIO.leftBackPosition = leftBackDrive.getCurrentPosition();
        mPeriodicIO.leftBackVelocity = leftFrontDrive.getVelocity();
        mPeriodicIO.leftBackCurrent = leftFrontDrive.getCurrent(Constants.currentUnit);

        mPeriodicIO.rightFrontPosition = rightFrontDrive.getCurrentPosition();
        mPeriodicIO.rightFrontVelocity = leftFrontDrive.getVelocity();
        mPeriodicIO.rightFrontCurrent = leftFrontDrive.getCurrent(Constants.currentUnit);

        mPeriodicIO.rightBackPosition = rightBackDrive.getCurrentPosition();
        mPeriodicIO.rightBackVelocity = leftFrontDrive.getVelocity();
        mPeriodicIO.rightBackCurrent = leftFrontDrive.getCurrent(Constants.currentUnit);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void writePeriodicOutputs() {
        if (isOpenLoop) {
            leftFrontDrive.setPower(mPeriodicIO.leftFrontDemand);
            leftBackDrive.setPower(mPeriodicIO.leftBackDemand);
            rightFrontDrive.setPower(mPeriodicIO.rightFrontDemand);
            rightBackDrive.setPower(mPeriodicIO.rightBackDemand);

        } else {
            leftFrontDrive.setPower(leftFrontController.calculate(mPeriodicIO.leftFrontVelocity, mPeriodicIO.leftFrontDemand));
            leftBackDrive.setPower(leftBackController.calculate(mPeriodicIO.leftBackVelocity, mPeriodicIO.leftFrontDemand));
            rightFrontDrive.setPower(rightFrontController.calculate(mPeriodicIO.rightFrontVelocity, mPeriodicIO.rightFrontDemand));
            rightBackDrive.setPower(rightBackController.calculate(mPeriodicIO.rightBackVelocity, mPeriodicIO.rightBackDemand));
        }
    }

    public void mecanumDrive(Translation2d translation2d, double rotation) {
        isOpenLoop = true;

        double axial    = -translation2d.getY();
        double lateral  = translation2d.getX();
        double yaw      = rotation;

        mPeriodicIO.leftFrontDemand     = axial + lateral + yaw;
        mPeriodicIO.leftBackDemand      = axial - lateral - yaw;
        mPeriodicIO.rightFrontDemand    = axial - lateral + yaw;
        mPeriodicIO.rightBackDemand     = axial + lateral - yaw;

        scaleDriveDemands();
    }


    public void mecanumDriveClosed(Translation2d translation2d, double rotation) {
        isOpenLoop = false;

        double axial    = -translation2d.getY();
        double lateral  = translation2d.getX();
        double yaw      = rotation;

        mPeriodicIO.leftFrontDemand     = axial + lateral + yaw;
        mPeriodicIO.leftBackDemand      = axial - lateral - yaw;
        mPeriodicIO.rightFrontDemand    = axial - lateral + yaw;
        mPeriodicIO.rightBackDemand     = axial + lateral - yaw;

        scaleDriveDemands();
    }

    public void fieldCentricDrive(double headingR, Translation2d driveTranslation, double rightX) {
        isOpenLoop = false;

        double driveX = driveTranslation.getX();
        double driveY = -driveTranslation.getY();

        double axial    = driveY * Math.cos(headingR) + driveX * Math.sin(headingR);
        double lateral  = -driveY * Math.sin(headingR) + driveX * Math.cos(headingR);
        double yaw      = rightX;

        mPeriodicIO.leftFrontDemand     = axial + lateral + yaw;
        mPeriodicIO.leftBackDemand      = axial - lateral - yaw;
        mPeriodicIO.rightFrontDemand    = axial - lateral + yaw;
        mPeriodicIO.rightBackDemand     = axial + lateral - yaw;

        scaleDriveDemands();
    }

    private void scaleDriveDemands() {
        if (isSlowMode) {
            mPeriodicIO.leftFrontDemand     *= DriveConstants.slowScale;
            mPeriodicIO.leftBackDemand      *= DriveConstants.slowScale;
            mPeriodicIO.rightFrontDemand    *= DriveConstants.slowScale;
            mPeriodicIO.rightBackDemand     *= DriveConstants.slowScale;
        } else if (isFastMode) {
            mPeriodicIO.leftFrontDemand     *= DriveConstants.fastScale;
            mPeriodicIO.leftBackDemand      *= DriveConstants.fastScale;
            mPeriodicIO.rightFrontDemand    *= DriveConstants.fastScale;
            mPeriodicIO.rightBackDemand     *= DriveConstants.fastScale;
        } else {
            mPeriodicIO.leftFrontDemand     *= DriveConstants.normalScale;
            mPeriodicIO.leftBackDemand      *= DriveConstants.normalScale;
            mPeriodicIO.rightFrontDemand    *= DriveConstants.normalScale;
            mPeriodicIO.rightBackDemand     *= DriveConstants.normalScale;
        }
    }

    public void setFastMode() {
        isFastMode = true;
        isSlowMode = false;
    }

    public void setNormalMode() {
        isFastMode = false;
        isSlowMode = false;
    }

    public void setSlowMode() {
        isFastMode = false;
        isSlowMode = true;
    }


    @Override
    public void stop() {
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }

    private class PeriodicIO {
        // Inputs
        public int      leftFrontPosition;
        public double   leftFrontVelocity;
        public double   leftFrontCurrent;

        public int      leftBackPosition;
        public double   leftBackVelocity;
        public double   leftBackCurrent;

        public int      rightFrontPosition;
        public double   rightFrontVelocity;
        public double   rightFrontCurrent;

        public int      rightBackPosition;
        public double   rightBackVelocity;
        public double   rightBackCurrent;


        // Outputs
        public double leftFrontDemand;
        public double leftBackDemand;
        public double rightFrontDemand;
        public double rightBackDemand;
    }
}
