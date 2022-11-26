package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Constants.LiftClawConstants;
import org.firstinspires.ftc.teamcode.Robot.ControlBoard;
import org.firstinspires.ftc.teamcode.Robot.SubsystemOpMode;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Claw;

@TeleOp(name="ClawTele", group="OpMode")
public class ClawTele extends SubsystemOpMode {

    private ControlBoard mControlBoard;
    private final Claw mLiftClaw;

    private ClawTele() {
        mControlBoard = new ControlBoard(this);
        mLiftClaw = new Claw(this, LiftClawConstants.name, LiftClawConstants.inverted, LiftClawConstants.min, LiftClawConstants.max);
    }

        @Override
    public void opInit() {
        telemetry.addLine("Initialized");
    }

    @Override
    public void opPeriodic() {
        double deadBan = 0.2;
        double step = 0.1;

        double setPoint = mLiftClaw.getClawPosition();

        if (mControlBoard.driver.rightY() > deadBan) {
            setPoint += step;
        } else if (mControlBoard.driver.rightY() < -deadBan) {
            setPoint -= step;
        }

        mLiftClaw.setClawPosition(setPoint);
    }

    @Override
    public void telemetryPeriodic() {
        telemetry.addData("Claw Position: ", mLiftClaw.getClawPosition());
    }

    @Override
    public void opStop() {

    }
}
