package org.team3128.common.narwhalvision;


/**
 * Class which describes a target
 * It is serialized and sent to the robot.
 *
 * NOTE: it cannot reference any classes available on only one side or the other
 */

public class TargetInformation
{
	// NOTE: these are kept public to give the serializer an easier time
	// we also use floats so we don't send an unneccessary amount of precision

	public float area;
	public float boundingRectLeft;
	public float boundingRectTop;
	public float boundingRectRight;
	public float boundingRectBottom;
	
	//NOTE: these are pixel coordinates, so the origin is in the top left of the image

	public float boundingRectHeight, boundingRectWidth;

	public float boundingRectCenterX, boundingRectCenterY;

	public int imageWidth, imageHeight;

	public float horizontalFOV, verticalFOV;
	
	// position in the list of targets found in this round of searching.  A lower number means that this target matched the criteria better. The best target of each round has index 1.
	public int targetRanking;

	/**
	 * Blank constructor for serializer
	*/
	public TargetInformation()
	{
	}

	/**
	 * Computes the estimated distance to a target using the known height of the target off the ground.
	 * Only works if the camera is horizontal.
	 *
	 * @param cameraHeight distance between the center of the camera lens and the ground
	 * @param targetHeight distance between the center of the target and the ground
	 *
	 * @return The estimated distance directly to the target in cm.
	 */
	public float getTargetDistance(float targetHeight, float cameraHeight)
	{
		float deltaHeight = targetHeight - cameraHeight;

		float targetDistance = (float) (deltaHeight / Math.cos(Math.toRadians(getVerticalAngle())));

		return targetDistance;
	}


	/**
	 * Get this particle's offset in degrees from directly in front of the camera.
	 * Note that this may be less accurate at larger angles due to parallax distortion.
	 *        _
	 *       |_|
	 *     \    |    /
	 *      \   |   /
	 *       \  |  /
	 *        \ | /
	 *         \_/
	 *         | |
	 * Can be positive or negative.  If the object is in the center, it returns 0.
	 * @return
	 */
	public float getHorizontalAngle()
	{
		float distanceFromCenter = boundingRectCenterX - imageWidth / 2.0F;

		return (float) Math.toDegrees(Math.atan(2 * distanceFromCenter * Math.tan(Math.toRadians(horizontalFOV / 2.0F)) / imageWidth));
	}

	/**
	 * Get this particle's vertical offset (azimuth) in degrees from directly in front of the camera.
	 * Note that this may be less accurate at larger angles due to parallax distortion.
	 *        _
	 *       |_|
	 *     \    |    /
	 *      \   |   /
	 *       \  |  /
	 *        \ | /
	 *         \_/
	 *         | |
	 * Can be positive or negative.  If the object is in the center, it returns 0.
	 * @return
	 */
	public float getVerticalAngle()
	{
		float distanceFromCenter = boundingRectCenterY - imageHeight / 2.0F;

		return (float) Math.toDegrees(Math.atan(distanceFromCenter * Math.tan(Math.toRadians(horizontalFOV)) / imageWidth));
	}
	
	public String toString()
	{
		return String.format("TargetInformation: height: %.02f, width: %.02f, center coordinates: (%.02f, %.02f)",
				boundingRectHeight, boundingRectWidth, boundingRectCenterX, boundingRectCenterY);
	}
}
