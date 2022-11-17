package org.firstinspires.ftc.teamcode.Tele;

import org.firstinspires.ftc.teamcode.Robot.ControlBoard;
import org.firstinspires.ftc.teamcode.Robot.SubsystemOpMode;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drive;

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
        mDrive.mecanumDrive(mControlBoard.driveTranslation(), mControlBoard.rotation());
    }

    @Override
    public void opStop() {

    }


}
