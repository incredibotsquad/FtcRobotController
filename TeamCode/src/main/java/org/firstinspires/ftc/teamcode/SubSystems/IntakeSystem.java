package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.GameColors;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

@Config
public class IntakeSystem extends SubsystemGroup {

    public boolean isOn;

    public boolean enableIntakeAutomation = false;

    public static final IntakeSystem INSTANCE = new IntakeSystem();

    private IntakeSystem() {
        super(
                IntakeColorSensors.INSTANCE,
                IntakeWheels.INSTANCE,
                IntakeLight.INSTANCE
        );
        isOn = false;
    }

    public boolean isFull() {
        return Spindexer.INSTANCE.storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor == GameColors.NONE);
    }

    public boolean hasOneBall(){
        return Spindexer.INSTANCE.storedColors.stream().filter(ballEntry -> ballEntry.ballColor != GameColors.NONE).count() == 1;
    }

    public boolean hasTwoBalls(){
        return Spindexer.INSTANCE.storedColors.stream().filter(ballEntry -> ballEntry.ballColor != GameColors.NONE).count() == 2;
    }

    public boolean isEmpty() {
        return Spindexer.INSTANCE.storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor != GameColors.NONE);
    }

    public void turnOn() {
        IntakeWheels.INSTANCE.turnOn.invoke();
        Spindexer.INSTANCE.moveToNextEmptySlot();
        isOn = true;
    }
//    public Command turnOn = IntakeWheels.INSTANCE.turnOn
//                                .and(Spindexer.INSTANCE.moveToNextEmptySlot)
//                                .and(new InstantCommand(() -> isOn = true))
//                                .named("Intake Group On");

    public void turnOff() {
        IntakeWheels.INSTANCE.turnOff.invoke();
        Spindexer.INSTANCE.moveToNextFullSlot();
        isOn = false;
    }
//    public Command turnOff = IntakeWheels.INSTANCE.turnOff
//                                .and(Spindexer.INSTANCE.moveToNextFullSlot)
//                                .and(new InstantCommand(() -> isOn = false))
//                                .named("Intake Group Off");

//    public Command manualOn = turnOn
//                                .and()
//                                .requires(this);
//
//    public Command manualOff = turnOff
//                                .and()
//                                .requires(this);

    //THIS FUNCTION GETS CALLED EVERY LOOP WHILE WAITING FOR START - ITS FINE TO RUN IT THEN
    //THIS WILL ALSO GET CALLED WHILE THE OPMODE IS RUNNING - THIS IS ALSO WHAT WE NEED
    @Override
    public void periodic() {

        if (!enableIntakeAutomation) return;

        if (isEmpty()) {
            Log.i("INTAKE", "PERIODIC: EMPTY LIGHT");
            IntakeLight.INSTANCE.indicateEmpty.invoke();
        }

        if (hasOneBall()) {
            Log.i("INTAKE", "PERIODIC: ONE BALL LIGHT");
            IntakeLight.INSTANCE.indicateOneBall.invoke();
        }

        if (hasTwoBalls()) {
            Log.i("INTAKE", "PERIODIC: TWO BALLS LIGHT");
            IntakeLight.INSTANCE.indicateTwoBalls.invoke();
        }

        if (isFull()) {
            Log.i("INTAKE", "PERIODIC: FULL LIGHT");
            IntakeLight.INSTANCE.indicateThreeBalls.invoke();
        }
    }
}
