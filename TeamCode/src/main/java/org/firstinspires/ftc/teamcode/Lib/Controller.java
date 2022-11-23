package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Constants;

public class Controller {

    private Gamepad gamepad;

    private double joystickDeadzone;
    private double triggerDeadzone;

    public enum DeadzoneType {
        CIRCULAR, SQUARE
    }

    private DeadzoneType deadzoneType;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.joystickDeadzone = 0.0;
        this.triggerDeadzone = 0.0;
        this.deadzoneType = DeadzoneType.SQUARE;
    }

    public void setJoystickDeadzone(DeadzoneType deadzoneType, double deadzone) {
        this.deadzoneType = deadzoneType;
        this.joystickDeadzone = deadzone;
    }

    public void setTriggerDeadzone(double deadzone) {
        this.triggerDeadzone = deadzone;
    }

    public double leftX() {
        return gamepad.left_stick_x;
    }

    public double leftY() {
        return gamepad.left_stick_y;
    }

    public boolean leftStickPressed() {
        return gamepad.left_stick_button;
    }

    public double rightX() {
        return gamepad.right_stick_x;
    }

    public double rightY() {
        return gamepad.left_stick_y;
    }

    public boolean rightStickPressed() {
        return gamepad.right_stick_button;
    }

    public double leftTrigger() {
        return gamepad.left_trigger;
    }

    public double rightTrigger() {
        return gamepad.right_trigger;
    }

    public boolean leftBumper() {
        return gamepad.left_bumper;
    }

    public boolean rightBumper() {
        return gamepad.right_bumper;
    }

    public boolean dpadUp(){
        return gamepad.dpad_up;
    }

    public boolean dpadDown(){
        return gamepad.dpad_down;
    }

    public boolean dpadLeft(){
        return gamepad.dpad_left;
    }

    public boolean dpadRight(){
        return gamepad.dpad_right;
    }

    public boolean A(){
        return gamepad.a;
    }

    public boolean B(){
        return gamepad.b;
    }

    public boolean X(){
        return gamepad.x;
    }

    public boolean Y(){
        return gamepad.y;
    }

    public boolean ps() {
        return gamepad.ps;
    }

    public boolean options() {
        return gamepad.options;
    }

}
