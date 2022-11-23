package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Lib.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Constants.LiftConstants;
import org.firstinspires.ftc.teamcode.Robot.Constants.LiftConstants.LiftController;
import org.firstinspires.ftc.teamcode.Robot.SubsystemOpMode;

public class Lift extends Subsystem {

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    private DcMotorEx leftWinch;
    private DcMotorEx rightWinch;

    private PIDFController liftController;

    private boolean isOpenLoop = false;

    public enum LiftPosition {
        DOWN, LOW, MEDIUM, HIGH
    }

    public Lift(OpMode op) {
        // Left Winch
        leftWinch = (DcMotorEx) op.hardwareMap.get(LiftConstants.rightWinchName);
        leftWinch.setDirection(LiftConstants.leftDirection);
        leftWinch.setZeroPowerBehavior(LiftConstants.zeroPowerBehavior);

        //Right Winch
        rightWinch = (DcMotorEx) op.hardwareMap.get(LiftConstants.rightWinchName);
        rightWinch.setDirection(LiftConstants.rightDirection);
        rightWinch.setZeroPowerBehavior(LiftConstants.zeroPowerBehavior);

        liftController = new PIDFController(LiftController.kP, LiftController.kI, LiftController.kD, LiftController.kF);
        liftController.setTolerance(LiftController.tolerance);
    }


    @Override
    public void initAction() {
        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void enable() {

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.leftPosition = leftWinch.getCurrentPosition();
        mPeriodicIO.leftCurrent = leftWinch.getCurrent(Constants.currentUnit);
        mPeriodicIO.leftVelocity = leftWinch.getVelocity();

        mPeriodicIO.rightPosition = rightWinch.getCurrentPosition();
        mPeriodicIO.rightCurrent = rightWinch.getCurrent(Constants.currentUnit);
        mPeriodicIO.rightVelocity = rightWinch.getVelocity();
    }

    private int getLiftPosition() {
        return mPeriodicIO.leftPosition;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void writePeriodicOutputs() {
        // Apply Limits
        if (mPeriodicIO.liftDemand > LiftConstants.maximumTravel) {
            mPeriodicIO.liftDemand = LiftConstants.maximumTravel;
        } else if (mPeriodicIO.liftDemand < LiftConstants.minimumTravel) {
            mPeriodicIO.liftDemand = LiftConstants.minimumTravel;
        }

        // Set Power / Position
        if (isOpenLoop) {
            leftWinch.setPower(mPeriodicIO.liftDemand);
            rightWinch.setPower(mPeriodicIO.liftDemand);
        } else {
            double power = liftController.calculate(getLiftPosition(), mPeriodicIO.liftDemand);

            leftWinch.setPower(power);
            rightWinch.setPower(power);
        }
    }

    public void setLiftPosition(LiftPosition position) {
        switch (position) {
            case DOWN:
                setLiftPosition(LiftConstants.downPosition);
                break;
            case LOW:
                setLiftPosition(LiftConstants.lowPosition);
                break;
            case MEDIUM:
                setLiftPosition(LiftConstants.mediumPosition);
                break;
            case HIGH:
                setLiftPosition(LiftConstants.highPosition);
                break;
            default:
                break;
        }
    }

    public void setLiftPosition(int position) {
        isOpenLoop = false;
        mPeriodicIO.liftDemand = position;
    }

    public void changeLiftPosition(double change) {
        isOpenLoop = false;

        change = change * LiftConstants.positionChangeScale;

        mPeriodicIO.liftDemand = getLiftPosition() + change;
    }

    public void setLiftPower(double power) {
        isOpenLoop = true;
        mPeriodicIO.liftDemand = power;
    }

    @Override
    public void stop() {

    }

    private class PeriodicIO {
        public int      leftPosition;
        public double   leftVelocity;
        public double   leftCurrent;

        public int      rightPosition;
        public double   rightVelocity;
        public double   rightCurrent;

        public double liftDemand;
    }
}
