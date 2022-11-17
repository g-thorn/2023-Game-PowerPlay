package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Lib.Controller;
import org.firstinspires.ftc.teamcode.Robot.Constants.ControlConstants;

public class ControlBoard {
    private Controller driver;
    private Controller operator;

    public ControlBoard(OpMode op) {
        driver = new Controller(op.gamepad1);
        operator = new Controller(op.gamepad2);
    }

    /** Driver */
    public Translation2d driveTranslation() {
        if (Math.abs(driver.leftX()) > ControlConstants.driverDeadzone || Math.abs(driver.leftY()) > ControlConstants.driverDeadzone) {
            double x = driver.leftX() * 0.5;
            double y = driver.leftY() * 0.5;
            return new Translation2d(x, y);
        }
        return new Translation2d(0.0, 0.0);
    }

    public double rotation() {
        if (Math.abs(driver.rightX()) > ControlConstants.driverDeadzone) {
            double rot = driver.rightX();
            return rot;
        }
        return 0.0;
    }

    /** Operator */
    public double joint1() {
        if (Math.abs(operator.leftX()) > ControlConstants.armDeadzone) {
            return operator.leftX();
        }
        return 0.0;
    }

    public double joint2() {
        if (Math.abs(operator.leftY()) > ControlConstants.armDeadzone) {
            return operator.leftY();
        }
        return 0.0;
    }
}
