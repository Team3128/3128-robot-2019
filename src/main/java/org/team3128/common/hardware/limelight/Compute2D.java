package org.team3128.common.hardware.limelight;

import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.units.Length;

public class Compute2D {
    public static class Compute2DLocalization {
        public double x, y, alpha, yaw;

        public double xPrime, yPrime;

        public double thetaLeft, thetaRight;
        public double distance, distanceLeft, distanceRight;

        public String toString() {
            return "\n" + "(x, y) = " + (x / Length.in) + "in, " + (y / Length.in) + "in\n" + "(x', y') = "
                    + (xPrime / Length.in) + "in, " + (yPrime / Length.in) + "in\n" + "\n" + "alpha = " + alpha + "\n"
                    + "yaw = " + yaw + "\n" + "(theta_left, theta_right) = " + thetaLeft + "," + thetaRight + "\n"
                    + "\n" + "d = " + (distance / Length.in) + "in\n" + "(dL, dR) = " + (distanceLeft / Length.in)
                    + "in," + (distanceRight / Length.in) + "in";
        }
    }

    public static class Compute2DInput {
        public double horizontalOffsetAngle, verticalOffsetAngle, boundingBoxPixelWidth;
    }

    public static Compute2DInput getInput(Limelight limelight, int numSamples) {
        Compute2DInput input = new Compute2DInput();

        input.horizontalOffsetAngle = limelight.getValue(LimelightKey.HORIZONTAL_OFFSET, numSamples);
        input.verticalOffsetAngle = limelight.getValue(LimelightKey.VERTICAL_OFFSET, numSamples);
        input.boundingBoxPixelWidth = limelight.getValue(LimelightKey.LENGTH_HORIZONTAL, numSamples);

        return input;
    }

    public static Compute2DLocalization compute2D(Limelight limelight, Compute2DInput inputData, double targetHeight) {
        Compute2DLocalization outputData = new Compute2DLocalization();

        double targetHorizontalArcAngle = inputData.boundingBoxPixelWidth * LimelightConstants.HORIZONTAL_FOV
                / LimelightConstants.SCREEN_WIDTH;

        outputData.yPrime = limelight.calculateYPrimeFromTY(inputData.verticalOffsetAngle, targetHeight);
        outputData.xPrime = outputData.yPrime * RobotMath.tan(inputData.horizontalOffsetAngle);

        outputData.distance = outputData.yPrime / RobotMath.cos(inputData.horizontalOffsetAngle);

        double tanBover2 = RobotMath.tan(targetHorizontalArcAngle / 2);
        double alpha = RobotMath.atan(Math.sqrt((2 * RobotMath.square(outputData.distance)
                - 2 * outputData.distance
                        * Math.sqrt(RobotMath.square(tanBover2 * limelight.targetWidth)
                                + RobotMath.square(outputData.distance) + RobotMath.square(limelight.targetWidth))
                + RobotMath.square(limelight.targetWidth)) / (RobotMath.square(tanBover2 * limelight.targetWidth))));

        outputData.alpha = alpha;

        outputData.y = outputData.distance * RobotMath.cos(alpha);
        outputData.x = outputData.y * RobotMath.tan(alpha - targetHorizontalArcAngle / 2) + limelight.targetWidth / 2;

        return outputData;
    }

    public static Compute2DLocalization deadCompute2D(Limelight limelight, Compute2DInput inputData,
            double targetHeight) {
        Compute2DLocalization outputData = new Compute2DLocalization();

        double targetHorizontalArcAngle = inputData.boundingBoxPixelWidth * LimelightConstants.HORIZONTAL_FOV
                / LimelightConstants.SCREEN_WIDTH;

        outputData.thetaLeft = inputData.horizontalOffsetAngle - targetHorizontalArcAngle / 2;
        outputData.thetaRight = inputData.horizontalOffsetAngle + targetHorizontalArcAngle / 2;

        outputData.yPrime = limelight.calculateYPrimeFromTY(inputData.verticalOffsetAngle, targetHeight);
        outputData.xPrime = outputData.yPrime * RobotMath.tan(inputData.horizontalOffsetAngle);

        outputData.distance = outputData.yPrime / RobotMath.cos(inputData.horizontalOffsetAngle);

        double leftDistancePlusMinus = (outputData.thetaLeft > 0) ? -1 : 1;
        double rightDistancePlusMinus = (outputData.thetaRight > 0) ? 1 : -1;

        outputData.distanceLeft = outputData.distance
                * RobotMath.cos(inputData.horizontalOffsetAngle - outputData.thetaLeft)
                + leftDistancePlusMinus * 0.5 * Math.sqrt(RobotMath.square(limelight.targetWidth) - RobotMath.square(2
                        * outputData.distance * RobotMath.sin(inputData.horizontalOffsetAngle - outputData.thetaLeft)));
        outputData.distanceRight = outputData.distance
                * RobotMath.cos(inputData.horizontalOffsetAngle - outputData.thetaRight)
                + rightDistancePlusMinus * 0.5
                        * Math.sqrt(RobotMath.square(limelight.targetWidth) - RobotMath.square(2 * outputData.distance
                                * RobotMath.sin(inputData.horizontalOffsetAngle - outputData.thetaRight)));

        outputData.yaw = RobotMath.asin((outputData.distanceLeft * RobotMath.cos(outputData.thetaLeft)
                - outputData.distanceRight * RobotMath.cos(outputData.thetaRight)) / limelight.targetWidth);

        outputData.x = -outputData.distance * RobotMath.sin(inputData.horizontalOffsetAngle - outputData.yaw);
        outputData.y = -outputData.distance * RobotMath.cos(inputData.horizontalOffsetAngle - outputData.yaw);

        return outputData;
    }

}