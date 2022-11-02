package org.firstinspires.ftc.teamcode.Robot;

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
    }

    public void initHardware() {

    }

    public void initAction() {

    }

    public void periodic() {

    }

    public void stop() {

    }
}
