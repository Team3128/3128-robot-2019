package org.team3128.gromit.util;

import org.team3128.common.util.units.Length;
import org.team3128.gromit.main.MainDeepSpaceRobot.GameElement;
import org.team3128.gromit.main.MainDeepSpaceRobot.ScoreTarget;

public class DeepSpaceConstants {
    public static double AUTOPTIMUS_DISTANCE = 3 * Length.ft;

    public static double VISION_TARGET_POINT = 1 * Length.ft;
    public static double VISION_TX_ALIGN_THRESHOLD = 6 * Length.in;

    public static double DECELERATE_START_DISTANCE = 4 * Length.ft;
    public static double DECELERATE_END_DISTANCE = 2 * Length.ft;

    public static double LOW_VISION_TARGET_HEIGHT = 28.5 * Length.in;
    public static double HIGH_VISION_TARGET_HEIGHT = 37.5 * Length.in;

    public static double getVisionTargetHeight(GameElement gameElement, ScoreTarget scoreTarget) {
        if (gameElement == GameElement.HATCH_PANEL || scoreTarget == ScoreTarget.CARGO_SHIP) {
            return DeepSpaceConstants.LOW_VISION_TARGET_HEIGHT;
        }
        else {
            return DeepSpaceConstants.HIGH_VISION_TARGET_HEIGHT;
        }
    }
}