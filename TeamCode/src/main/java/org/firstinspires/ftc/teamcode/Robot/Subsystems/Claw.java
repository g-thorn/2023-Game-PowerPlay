package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Lib.Subsystem;

public class Claw extends Subsystem {

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private ServoEx clawServo;

    private final double rangeMax;
    private final double rangeMin;


    public Claw(OpMode op, String name, boolean inverted, double rangeMax, double rangeMin) {

        this.rangeMax = rangeMax;
        this.rangeMin = rangeMin;

        clawServo = (ServoEx) op.hardwareMap.get(Servo.class, name);
        clawServo.setInverted(inverted);
        clawServo.setRange(rangeMin, rangeMax);

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
        mPeriodicIO.clawPosition =  clawServo.getPosition();
    }

    @Override
    public void writePeriodicOutputs() {
        // Apply Limits
        if (mPeriodicIO.clawDemand > rangeMax) {
            mPeriodicIO.clawDemand = rangeMax;
        } else if (mPeriodicIO.clawDemand < rangeMin) {
            mPeriodicIO.clawDemand = rangeMin;
        }

        clawServo.setPosition(mPeriodicIO.clawDemand);
    }

    public void setClawPosition(double position) {
        mPeriodicIO.clawDemand = position;
    }

    public void changeClawPosition(double change) {
        mPeriodicIO.clawDemand = getClawPosition() + change;
    }

    @Override
    public void stop() {

    }

    public double getClawPosition() {
        return mPeriodicIO.clawPosition;
    }

    private class PeriodicIO {
        double clawPosition;

        double clawDemand;
    }
}
