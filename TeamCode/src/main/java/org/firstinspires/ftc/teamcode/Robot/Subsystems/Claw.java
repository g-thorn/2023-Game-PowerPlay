package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Lib.Subsystem;
import org.firstinspires.ftc.teamcode.Robot.Constants.ServoConstants.ClawServo;
import org.firstinspires.ftc.teamcode.Robot.Constants.ServoConstants.ZServo;
import org.firstinspires.ftc.teamcode.Robot.Constants.ServoConstants.XServo;

public class Claw extends Subsystem {

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private Servo clawServo;
    private Servo xServo;
    private Servo zServo;


    private enum ClawState {
        OPEN, WIDE_OPEN, CLOSED, MANUAL
    }
    ClawState clawState;

    private enum XState {
        LEFT_FLIP, MIDDLE, RIGHT_FLIP, MANUAL
    }
    XState xState;

    private enum ZState {
        LEVEL, MANUAL
    }
    ZState zState;

    public Claw(OpMode op) {

        clawServo = op.hardwareMap.get(Servo.class, ClawServo.deviceName);
        clawServo.scaleRange(ClawServo.min, ClawServo.max);
        clawServo.setDirection(ClawServo.direction);

        xServo = op.hardwareMap.get(Servo.class, XServo.deviceName);
        xServo.scaleRange(XServo.min, XServo.max);
        xServo.setDirection(XServo.direction);

        zServo = op.hardwareMap.get(Servo.class, ZServo.deviceName);
        zServo.scaleRange(ZServo.min, ZServo.max);
        zServo.setDirection(ZServo.direction);
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

    public void setClawServoState(ClawState clawState) {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void stop() {

    }

    private class PeriodicIO {
        double clawDemand;
        double xDemand;
        double zDemand;
    }
}
