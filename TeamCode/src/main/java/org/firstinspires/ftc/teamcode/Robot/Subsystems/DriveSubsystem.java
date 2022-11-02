package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Lib.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class DriveSubsystem extends Subsystem {

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;

    public DriveSubsystem(OpMode op) {
        super();

        // Left Front Drive
        leftFrontDrive = op.hardwareMap.get(DcMotorEx.class, DriveConstants.leftFrontName);
        leftFrontDrive.setDirection(DriveConstants.leftDriveDirection);
        leftFrontDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);
        leftFrontDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);


        // Left Back Drive
        leftBackDrive = op.hardwareMap.get(DcMotorEx.class, DriveConstants.leftBackName);
        leftBackDrive.setDirection(DriveConstants.leftDriveDirection);
        leftBackDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);
        leftBackDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);


        // Right Front Drive
        rightFrontDrive = op.hardwareMap.get(DcMotorEx.class, DriveConstants.rightFrontName);
        rightFrontDrive.setDirection(DriveConstants.rightDriveDirection);
        rightFrontDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);
        rightFrontDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);


        // Right Back Drive
        rightBackDrive = op.hardwareMap.get(DcMotorEx.class, DriveConstants.rightBackName);
        rightBackDrive.setDirection(DriveConstants.rightDriveDirection);
        rightBackDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);
        rightBackDrive.setZeroPowerBehavior(DriveConstants.driveBrakeOnInit);
    }

    @Override
    public void initHardware() {

    }

    @Override
    public void initAction() {

    }


    @Override
    public void enable() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.leftFrontPosition = leftFrontDrive.getCurrentPosition();
        mPeriodicIO.leftFrontVelocity = leftFrontDrive.getVelocity();
        mPeriodicIO.leftFrontCurrent = leftFrontDrive.getCurrent(Constants.currentUnit);

        mPeriodicIO.leftBackPosition = leftBackDrive.getCurrentPosition();
        mPeriodicIO.leftBackVelocity = leftBackDrive.getVelocity();
        mPeriodicIO.leftBackCurrent = leftBackDrive.getCurrent(Constants.currentUnit);

        mPeriodicIO.rightFrontPosition = rightFrontDrive.getCurrentPosition();
        mPeriodicIO.rightFrontVelocity = rightFrontDrive.getVelocity();
        mPeriodicIO.rightFrontCurrent = rightFrontDrive.getCurrent(Constants.currentUnit);

        mPeriodicIO.rightBackPosition = rightBackDrive.getCurrentPosition();
        mPeriodicIO.rightBackVelocity = rightBackDrive.getVelocity();
        mPeriodicIO.rightBackCurrent = rightBackDrive.getCurrent(Constants.currentUnit);
    }

    public void mecanumDrive(Translation2d translation2d, double rotation) {
        double axial    = -translation2d.getY();
        double lateral  = translation2d.getX();
        double yaw      = rotation;

        mPeriodicIO.leftFrontDemand     = axial + lateral + yaw;
        mPeriodicIO.leftBackDemand      = axial - lateral - yaw;
        mPeriodicIO.rightFrontDemand    = axial - lateral + yaw;
        mPeriodicIO.rightBackDemand     = axial + lateral - yaw;
    }

    public void fieldCentricDrive(double heading, double leftX, double leftY, double rightX){

        double axial    = -leftY;
        double lateral  = leftX;
        double yaw      = rightX;

        mPeriodicIO.leftFrontDemand     = axial + lateral + yaw;
        mPeriodicIO.leftBackDemand      = axial - lateral - yaw;
        mPeriodicIO.rightFrontDemand    = axial - lateral + yaw;
        mPeriodicIO.rightBackDemand     = axial + lateral - yaw;
    }

    @Override
    public void writePeriodicOutputs() {
        leftFrontDrive.setPower(mPeriodicIO.leftFrontDemand);
        leftBackDrive.setPower(mPeriodicIO.leftBackDemand);
        rightFrontDrive.setPower(mPeriodicIO.rightFrontDemand);
        rightBackDrive.setPower(mPeriodicIO.rightBackDemand);
    }

    @Override
    public void stop() {

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

        public boolean isBrake;
    }
}
