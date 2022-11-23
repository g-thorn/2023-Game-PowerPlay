package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Lib.Controller;
import org.firstinspires.ftc.teamcode.Robot.Constants.ControlConstants;

public class ControlBoard {
    public Controller driver;
    public Controller operator;

    public ControlBoard(OpMode op) {
        driver = new Controller(op.gamepad1);
        operator = new Controller(op.gamepad2);
    }

    /** Driver */
    public Translation2d getDriveTranslation() {
        if (Math.abs(driver.leftX()) > ControlConstants.driveDeadBan || Math.abs(driver.leftY()) > ControlConstants.driveDeadBan) {
            double x = driver.leftX();
            double y = driver.leftY();
            return new Translation2d(x, y);
        }
        return new Translation2d(0.0, 0.0);
    }

    public double getDriveRotation() {
        if (Math.abs(driver.rightX()) > ControlConstants.rotationDeadBan) {
            double rot = driver.rightX();
            return rot;
        }
        return 0.0;
    }

    public boolean getDriveFastMode() {
        return driver.B();
    }

    public boolean getDriveSlowMode() {
        return driver.A();
    }

    /** Operator */
    public boolean getLiftHigh() {
        return operator.Y();
    }

    public boolean getLiftMedium() {
        return operator.X();
    }

    public boolean getLiftLow() {
        return operator.B();
    }

    public boolean getLiftDown() {
        return operator.A();
    }

    public double getLiftChange() {
        if (Math.abs(operator.leftY()) >= ControlConstants.liftDeadBan) {
            return operator.leftY();
        }
        return 0.0;
    }
}
