package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.GameColors;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

@Config
public class IntakeSystem extends SubsystemGroup {

    private boolean isOn;

    public boolean enableIntakeAutomation = false;

    public static final IntakeSystem INSTANCE = new IntakeSystem();

    private IntakeSystem() {
        super(
                IntakeColorSensors.INSTANCE,
                IntakeWheels.INSTANCE
        );
        isOn = false;
    }

    public boolean isFull() {
        return Spindexer.INSTANCE.storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor == GameColors.NONE);
    }

    public boolean isEmpty() {
        return Spindexer.INSTANCE.storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor != GameColors.NONE);
    }

    private Command turnOn = IntakeWheels.INSTANCE.turnOn
                                .and(Spindexer.INSTANCE.moveToNextEmptySlot)
                                .and(new InstantCommand(() -> isOn = true))
                                .named("Intake Group On");

    private Command turnOff = IntakeWheels.INSTANCE.turnOff
                                .and(Spindexer.INSTANCE.moveToNextFullSlot)
                                .and(new InstantCommand(() -> isOn = false))
                                .named("Intake Group Off");
//    public Command manualOn = turnOn
//                                .and()
//                                .requires(this);
//
//    public Command manualOff = turnOff
//                                .and()
//                                .requires(this);

    @Override
    public void periodic() {
        if(!enableIntakeAutomation) return;

        //start the intake if we have no stored colors
        if(!isOn && isEmpty()) {
            turnOn.schedule();
            Log.i("INTAKE", "PERIODIC: INTAKE TURNED ON");
            return;
        }

        //stop the intake if we have no more space
        if(isOn && isFull()) {
            turnOff.schedule();
            Log.i("INTAKE", "PERIODIC: INTAKE TURNED OFF");
            return;
        }

        if (isOn && IntakeColorSensors.INSTANCE.detectedColor != GameColors.NONE) {
            Spindexer.INSTANCE.storeCurrentBall(IntakeColorSensors.INSTANCE.detectedColor);
            IntakeColorSensors.INSTANCE.clearDetectedColor();
            Spindexer.INSTANCE.moveToNextEmptySlot.schedule();
            Log.i("INTAKE", "PERIODIC: BALL INDEXED AND MOVED TO NEXT EMPTY SLOT");
            return;
        }
    }
}
