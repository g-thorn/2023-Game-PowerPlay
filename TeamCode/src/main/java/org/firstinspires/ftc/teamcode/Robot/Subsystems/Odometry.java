package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Lib.Subsystem;

import static org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants.frontLeftLocation;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants.frontRightLocation;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants.backLeftLocation;
import static org.firstinspires.ftc.teamcode.Robot.Constants.DriveConstants.backRightLoation;

public class Odometry extends Subsystem {

    private MecanumDriveOdometry mOdometry;

    // Kinematics
    private MecanumDriveKinematics mDriveKinematics = new MecanumDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLoation);


    public Odometry(OpMode op, Drive mDrive) {
        mOdometry = new MecanumDriveOdometry(mDriveKinematics, new Rotation2d(0.0), new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()));
    }

    @Override
    public void initAction() {

    }

    @Override
    public void enable() {

    }

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void stop() {

    }
}
