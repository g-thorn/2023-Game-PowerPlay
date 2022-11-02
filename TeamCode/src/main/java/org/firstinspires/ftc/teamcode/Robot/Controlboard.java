package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Lib.Controller;
import org.firstinspires.ftc.teamcode.Robot.Constants.ControlConstants;

public class Controlboard {
    private Controller driver;
    private Controller operator;

    public Controlboard(OpMode op) {
        driver = new Controller(op.gamepad1);
        operator = new Controller(op.gamepad2);
    }

    public Translation2d drive() {
        if (Math.abs(driver.leftX()) > ControlConstants.stickDeadzone || Math.abs(driver.leftY()) > ControlConstants.stickDeadzone) {
            double x = driver.leftX() * 0.5;
            double y = driver.leftY() * 0.5;
            return new Translation2d(x, y);
        }
        return new Translation2d(0.0, 0.0);
    }

    public double rotation() {
        if (Math.abs(driver.rightX()) > ControlConstants.stickDeadzone) {
            double rot = driver.rightX();
            return rot;
        }
        return 0.0;
    }

}
