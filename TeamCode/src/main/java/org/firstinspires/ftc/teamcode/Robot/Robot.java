package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.IMU;
//9277 sucks
public class Robot {

    public SubsystemManager mSubsystemManager;

    public Drive mDrive;
    public ArmSubsystem mArmSubsystem;
    public Claw mClaw;

    public IMU mIMU;

    public ControlBoard mControlBoard;

    public Robot(OpMode op) {
        mSubsystemManager = new SubsystemManager();

        mDrive = new Drive(op);
        mArmSubsystem = new ArmSubsystem(op);
        mClaw = new Claw(op);

        mIMU = new IMU(op);

        mSubsystemManager.setSubsystems(
                mDrive,
                mArmSubsystem,
                mClaw,
                mIMU
        );

        mControlBoard = new ControlBoard(op);
    }

    public void initHardware() {

    }

    public void initAction() {

    }

    public void periodic() {
        /** Driver */
        Translation2d driveTranslation = mControlBoard.driveTranslation();
        double rotation = mControlBoard.rotation();

        mDrive.mecanumDrive(driveTranslation, rotation);

        /** Operator */
        int joint1Change = (int) mControlBoard.joint1() * 10;
        mArmSubsystem.updateJoint1Setpoint(joint1Change);

        int joint2Change = (int) mControlBoard.joint2() * 10;
        mArmSubsystem.updateJoint2Setpoint(joint2Change);
    }

    public void stop() {

    }
}
