package org.firstinspires.ftc.teamcode.Robot;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.Lib.Subsystem;

import java.util.Arrays;
import java.util.List;

public class SubsystemManager {
    private List<Subsystem> mAllSubsystems;

    SubsystemManager() {}

    public void setSubsystems(Subsystem... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
    }

    public List<Subsystem> getSubsystems() {
        return mAllSubsystems;
    }

    public void initAllHardware() {
        for (Subsystem s : mAllSubsystems) {
            s.initHardware();
        }
    }

    public void initAllAction() {
        for (Subsystem s : mAllSubsystems) {
            s.initAction();
        }
    }

    public void enableAll() {
        for (Subsystem s : mAllSubsystems) {
            s.enable();
        }
    }

    public void readAll() {
        for (Subsystem s : mAllSubsystems) {
            s.readPeriodicInputs();
        }
    }

    public void periodicAll() {
        for (Subsystem s : mAllSubsystems) {
            s.periodic();
        }
    }

    public void writeAll() {
        for (Subsystem s : mAllSubsystems) {
            s.writePeriodicOutputs();
        }
    }

    public void stopALl() {
        for (Subsystem s : mAllSubsystems) {
            s.stop();
        }
    }

}
