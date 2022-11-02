package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Subsystem {

    public abstract void initHardware();

    public abstract void initAction();

    public abstract void enable();

    public abstract void periodic();

    public abstract void readPeriodicInputs();

    public abstract void writePeriodicOutputs();

    public abstract void stop();

    public abstract class PeriodicIO {};
}
