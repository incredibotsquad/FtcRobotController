package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * "RightWall-L-G-L-F-L-P": RightWall-L-G-L-F-P plus a third launch. From RED_START_AUDIENCE_WALL: launch the
 * preloads at RED_LAUNCH_AUDIENCE_CELL_2, GARDEN, back to the launch spot (TIP check, launch if not
 * tipped), RIGHT FLOWER only if it launched, back again (TIP check, launch if not tipped), park at
 * RED_PARK_RIGHT. Any launch visit that doesn't launch with artifacts aboard parks straight away.
 * INIT: operator B = red, X = blue. Log tag "RightWallLGLFLP".
 */
@Autonomous(name = "RightWall-L-G-L-F-L-P", group = "Autonomous", preselectTeleOp = "IncredibotsMecanumDrive")
public class RightWallLGLFLP extends RightWallLGLFP {
    @Override
    protected boolean launchAfterFlower() {
        return true;
    }
}
