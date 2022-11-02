package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Teleop", group="OpMode")
public class FullTele extends OpMode {

    private Robot mRobot;

    FullTele() {
        mRobot = new Robot(this);

    }

    @Override
    public void init() {
        mRobot.mSubsystemManager.initAllHardware();
        mRobot.mSubsystemManager.initAllAction();
    }

    @Override
    public void loop() {
        mRobot.mSubsystemManager.readAll();
        mRobot.mSubsystemManager.periodicAll();
        mRobot.mSubsystemManager.writeAll();
    }

    @Override
    public void stop() {
        mRobot.mSubsystemManager.stopALl();
    }
}
