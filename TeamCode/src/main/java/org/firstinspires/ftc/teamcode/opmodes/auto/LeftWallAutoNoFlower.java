package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * LeftWallAuto without the FLOWER: from RED_START_FAR_WALL, wait at RED_LAUNCH_FAR_CELL_2 for the
 * CELL to come up, launch the preloads, then park at RED_PARK_LEFT (or park there straight away if
 * it can't launch). INIT: operator B = red, X = blue. Log tag "LeftWallAutoNoFlower".
 */
@Autonomous(name = "LeftWallAutoNoFlower", group = "Autonomous", preselectTeleOp = "IncredibotsMecanumDrive")
public class LeftWallAutoNoFlower extends LeftWallAuto {
    @Override
    protected boolean intakeAtFlower() {
        return false;
    }
}
