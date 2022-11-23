package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.SubsystemManager;

public abstract class Subsystem {

    public abstract void initAction();

    public abstract void enable();

    public abstract void readPeriodicInputs();

    public abstract void periodic();

    public abstract void writePeriodicOutputs();

    public abstract void stop();

    public abstract class PeriodicIO {};
}
