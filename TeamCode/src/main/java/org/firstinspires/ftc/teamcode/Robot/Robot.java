package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Lib.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.IMU;

public class Robot {

    public SubsystemManager mSubsystemManager;

    public DriveSubsystem mDriveSubsystem;
    public ArmSubsystem mArmSubsystem;
    public ClawSubsystem mClawSubsystem;

    public IMU mIMU;

    public Controlboard mControlboard;

    public Robot(OpMode op) {
        mSubsystemManager = new SubsystemManager();

        mDriveSubsystem = new DriveSubsystem(op);
        mArmSubsystem = new ArmSubsystem(op);
        mClawSubsystem = new ClawSubsystem(op);

        mIMU = new IMU(op);

        mSubsystemManager.setSubsystems(
                mDriveSubsystem,
                mArmSubsystem,
                mClawSubsystem,
                mIMU
        );

        mControlboard = new Controlboard(op);
    }

    public void initHardware() {

    }

    public void initAction() {

    }

    public void periodic() {
        /** Driver */
        Translation2d driveTranslation = mControlboard.drive();
        double rotation = mControlboard.rotation();

        mDriveSubsystem.mecanumDrive(driveTranslation, rotation);

        /** Operator */
        int joint1Change = (int)mControlboard.joint1() * 10;
        mArmSubsystem.updateJoint1Setpoint(joint1Change);

        int joint2Change = (int)mControlboard.joint2() * 10;
        mArmSubsystem.updateJoint2Setpoint(joint2Change);
    }

    public void stop() {

    }
}
