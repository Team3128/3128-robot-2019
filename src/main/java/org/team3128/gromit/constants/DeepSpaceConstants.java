package org.team3128.gromit.constants;

import org.team3128.common.generics.GameConstants;
import org.team3128.common.utility.units.Length;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;

public class DeepSpaceConstants extends GameConstants {
    public static final double AUTOPTIMUS_DISTANCE = 3 * Length.ft;

    public static final double VISION_TARGET_POINT = 1 * Length.ft;
    public static final double VISION_TX_ALIGN_THRESHOLD = 6 * Length.in;

    public static final double DECELERATE_START_DISTANCE = 2.5 * Length.ft;
    public static final double DECELERATE_END_DISTANCE = 0.6666666666666666666666 * Length.ft;

    public static final double LOW_VISION_TARGET_HEIGHT = 28.5 * Length.in;
    public static final double HIGH_VISION_TARGET_HEIGHT = 35 * Length.in;

    public static final double TARGET_WIDTH = 14.5 * Length.in;

    public static double getVisionTargetHeight(GameElement gameElement, ScoreTarget scoreTarget) {
        if (gameElement == GameElement.HATCH_PANEL || scoreTarget == ScoreTarget.CARGO_SHIP) {
            return DeepSpaceConstants.LOW_VISION_TARGET_HEIGHT;
        } else {
            return DeepSpaceConstants.HIGH_VISION_TARGET_HEIGHT;
        }
    }
}