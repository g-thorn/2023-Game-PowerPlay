package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.ControlBoard;
import org.firstinspires.ftc.teamcode.Robot.SubsystemOpMode;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;

@TeleOp(name="DriveTele", group="OpMode")
public class DriveTele extends SubsystemOpMode {

    private ControlBoard mControlBoard;
    private Drive mDrive;

    private DriveTele() {
        mControlBoard = new ControlBoard(this);

        mDrive = new Drive(this);

        mSubsystemManager.setSubsystems(
                mDrive
        );
    }

    @Override
    public void opInit() {

    }

    @Override
    public void opPeriodic() {

        if (mControlBoard.getDriveFastMode()) {
            mDrive.setFastMode();
        } else if (mControlBoard.getDriveSlowMode()) {
            mDrive.setSlowMode();
        } else {
            mDrive.setNormalMode();
        }

        mDrive.mecanumDrive(mControlBoard.getDriveTranslation(), mControlBoard.getDriveRotation(), true);
    }

    @Override
    public void telemetryPeriodic() {

    }

    @Override
    public void opStop() {

    }


}
