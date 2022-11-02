package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Lib.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.Robot.Constants.ArmConstants.Joint1Controller;
import org.firstinspires.ftc.teamcode.Robot.Constants.ArmConstants.Joint2Controller;


public class ArmSubsystem extends Subsystem {

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private DcMotorEx armJoint1;
    private DcMotorEx armJoint2;

    private PIDFCoefficients joint1Coeffs = new PIDFCoefficients(Joint1Controller.kP, Joint1Controller.kI, Joint1Controller.kD, Joint1Controller.kF);
    private PIDFCoefficients joint2Coeffs = new PIDFCoefficients(Joint2Controller.kP, Joint2Controller.kI, Joint2Controller.kD, Joint2Controller.kF);

    public ArmSubsystem(OpMode op) {

        super();

        // Joint 1
        armJoint1 = op.hardwareMap.get(DcMotorEx.class, ArmConstants.joint1Name);
        armJoint1.setDirection(ArmConstants.joint1Direction);
        armJoint1.setZeroPowerBehavior(ArmConstants.jointZeroPowerBehavior);

        armJoint1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, joint1Coeffs);
        armJoint1.setTargetPositionTolerance(Joint1Controller.tolerance);

        // Joint 2
        armJoint2 = op.hardwareMap.get(DcMotorEx.class, ArmConstants.joint2Name);
        armJoint2.setDirection(ArmConstants.joint2Direction);
        armJoint2.setZeroPowerBehavior(ArmConstants.jointZeroPowerBehavior);

        armJoint2.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, joint2Coeffs);
        armJoint2.setTargetPositionTolerance(Joint2Controller.tolerance);


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
        mPeriodicIO.joint1Position = armJoint1.getCurrentPosition();
        mPeriodicIO.joint1Velocity = armJoint1.getVelocity();
        mPeriodicIO.joint1Current = armJoint1.getCurrent(Constants.currentUnit);

        mPeriodicIO.joint2Position = armJoint2.getCurrentPosition();
        mPeriodicIO.joint2Velocity = armJoint2.getVelocity();
        mPeriodicIO.joint2Current = armJoint2.getCurrent(Constants.currentUnit);
    }

    @Override
    public void writePeriodicOutputs() {
        armJoint1.setTargetPosition(mPeriodicIO.joint1Demand);
        armJoint2.setTargetPosition(mPeriodicIO.joint2Demand);
    }

    public void updateJoint1Setpoint(int change) {
        mPeriodicIO.joint1Demand = mPeriodicIO.joint1Position + change;

        if (mPeriodicIO.joint1Demand > ArmConstants.joint1Max) {
            mPeriodicIO.joint1Demand = ArmConstants.joint1Max;
        } else if (mPeriodicIO.joint1Demand < ArmConstants.joint1Min) {
            mPeriodicIO.joint1Demand = ArmConstants.joint1Min;
        }
    }

    public void updateJoint2Setpoint(int change) {
        mPeriodicIO.joint2Demand = mPeriodicIO.joint2Position + change;

        if (mPeriodicIO.joint2Demand > ArmConstants.joint2Max) {
            mPeriodicIO.joint2Demand = ArmConstants.joint2Max;
        } else if (mPeriodicIO.joint2Demand < ArmConstants.joint2Min) {
            mPeriodicIO.joint2Demand = ArmConstants.joint2Min;
        }
    }

    @Override
    public void stop() {

    }

    private class PeriodicIO {
        int joint1Position;
        double joint1Velocity;
        double joint1Current;

        int joint2Position;
        double joint2Velocity;
        double joint2Current;

        int joint1Demand;
        int joint2Demand;
    }
}
