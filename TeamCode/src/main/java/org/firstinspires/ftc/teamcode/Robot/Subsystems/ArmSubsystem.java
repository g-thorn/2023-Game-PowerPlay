package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Lib.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Constants;
import org.firstinspires.ftc.teamcode.Robot.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.Robot.Constants.ArmConstants.Joint1Controller;
import org.firstinspires.ftc.teamcode.Robot.Constants.ArmConstants.Joint2Controller;


public class ArmSubsystem extends Subsystem {

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private DcMotorEx armJoint1;
    private DcMotorEx armJoint2;

    private PIDFController joint1Controller;
    private PIDFController joint2Controller;


    public ArmSubsystem(OpMode op) {

        super(op);

        // Joint 1
        armJoint1 = op.hardwareMap.get(DcMotorEx.class, ArmConstants.joint1Name);
        armJoint1.setDirection(ArmConstants.joint1Direction);
        armJoint1.setZeroPowerBehavior(ArmConstants.jointZeroPowerBehavior);

        joint1Controller.setPIDF(Joint1Controller.kP, Joint1Controller.kI, Joint1Controller.kD, Joint1Controller.kF);
        joint1Controller.setTolerance(Joint1Controller.tolerance);

        // Joint 2
        armJoint2 = op.hardwareMap.get(DcMotorEx.class, ArmConstants.joint2Name);
        armJoint2.setDirection(ArmConstants.joint2Direction);
        armJoint2.setZeroPowerBehavior(ArmConstants.jointZeroPowerBehavior);

        joint2Controller.setPIDF(Joint2Controller.kP, Joint2Controller.kI, Joint2Controller.kD, Joint2Controller.kF);
        joint2Controller.setTolerance(Joint2Controller.tolerance);

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
