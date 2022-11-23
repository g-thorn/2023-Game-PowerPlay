package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Constants.LiftClawConstants;
import org.firstinspires.ftc.teamcode.Robot.ControlBoard;
import org.firstinspires.ftc.teamcode.Robot.SubsystemOpMode;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Lift.LiftPosition;

@TeleOp(name="FullTele", group="OpMode")
public class FullTele extends SubsystemOpMode {

    private ControlBoard mControlBoard;
    private Drive mDrive;
    private IMU mIMU;
    private Lift mLift;
    private Claw mLiftClaw;


    private FullTele() {
        mControlBoard   = new ControlBoard(this);
        mIMU            = new IMU(this);
        mDrive          = new Drive(this);
        mLift           = new Lift(this);
        mLiftClaw       = new Claw(this, LiftClawConstants.name, LiftClawConstants.inverted, LiftClawConstants.min, LiftClawConstants.max);

        mSubsystemManager.setSubsystems(
                mIMU,
                mDrive,
                mLift,
                mLiftClaw
        );
    }

    @Override
    public void opInit() {

    }

    @Override
    public void opPeriodic() {
        // Drive
        if (mControlBoard.getDriveSlowMode()) {
            mDrive.setSlowMode();
        } else if (mControlBoard.getDriveFastMode()) {
            mDrive.setFastMode();
        } else {
            mDrive.setNormalMode();
        }

        mDrive.mecanumDrive(mControlBoard.getDriveTranslation(), mControlBoard.getDriveRotation());

        // Lift
        if (mControlBoard.getLiftDown()) {
            mLift.setLiftPosition(LiftPosition.DOWN);
        } else if (mControlBoard.getLiftLow()) {
            mLift.setLiftPosition(LiftPosition.LOW);
        } else if (mControlBoard.getLiftMedium()) {
            mLift.setLiftPosition(LiftPosition.MEDIUM);
        } else if (mControlBoard.getLiftHigh()) {
            mLift.setLiftPosition(LiftPosition.HIGH);
        }

        mLift.changeLiftPosition(mControlBoard.getLiftChange());

        // Claw


    }

    @Override
    public void telemetryPeriodic() {

    }

    @Override
    public void opStop() {

    }
}
