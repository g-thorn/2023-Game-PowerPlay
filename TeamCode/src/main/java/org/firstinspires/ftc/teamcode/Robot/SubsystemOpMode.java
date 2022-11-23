package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public abstract class SubsystemOpMode extends OpMode {

    protected SubsystemManager mSubsystemManager;

    public SubsystemOpMode() {
        mSubsystemManager = new SubsystemManager();
    }

    @Override
    public void init() {
        mSubsystemManager.initAllAction();

        opInit();

        telemetry.update();
    }

    public abstract void opInit();

    @Override
    public void loop() {
        mSubsystemManager.readAll();
        mSubsystemManager.periodicAll();
        mSubsystemManager.writeAll();

        opPeriodic();

        telemetryPeriodic();
        telemetry.update();
    }

    public abstract void opPeriodic();
    public abstract void telemetryPeriodic();

    @Override
    public void stop() {
        mSubsystemManager.stopALl();

        opStop();

        telemetry.update();
    }

    public abstract void opStop();
}
