package org.firstinspires.ftc.teamcode.common;

import java.util.List;

public class GamePattern {
    public int tagId;
    public List<GameColors> colorSequence;

    public GamePattern(int tagId, List<GameColors> colorSequence) {
        this.tagId = tagId;
        this.colorSequence = colorSequence;
    }
}
