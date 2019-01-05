package org.team3128.common.narwhalvision;

import org.team3128.common.util.Assert;

/**
 * TargetInformation created by averaging the positions of multiple individual targets.
 * Use this if there are multiple bits of reflective tape around a goal.
 * 
 * @author Narwhal
 *
 */
public class AveragedTargetInformation extends TargetInformation
{
	/**
	 * Note: Image attributes (FOV and size) are not averaged.
	 * Make sure that all targets are from the same camera.
	 * @param posOffsetX  Offset to move the center position by when making the average.  Use if your camera is off-center
	 * @param posOffsetY
	 * @param informations
	 */
	public AveragedTargetInformation(int posOffsetX, int posOffsetY, TargetInformation... informations)
	{
		Assert.greaterThan(informations.length, 0);
		
		for(TargetInformation info : informations)
		{
			this.area += info.area;
			this.boundingRectHeight += info.boundingRectHeight;
			this.boundingRectWidth += info.boundingRectWidth;
			this.boundingRectCenterX += info.boundingRectCenterX;
			this.boundingRectCenterY += info.boundingRectCenterY;
		}
		
		this.area /= informations.length;
		this.boundingRectHeight /= informations.length;
		this.boundingRectWidth /= informations.length;
		this.boundingRectCenterX /= informations.length;
		this.boundingRectCenterY /= informations.length;
		
		//copy non-averaged attributes
		this.horizontalFOV = informations[0].horizontalFOV;
		this.verticalFOV = informations[0].verticalFOV;
		this.imageWidth = informations[0].imageWidth;
		this.imageHeight = informations[0].imageHeight;
		
		//apply offset
		this.boundingRectCenterX += posOffsetX;
		this.boundingRectCenterY += posOffsetY;
		
		//calculate dependent attributes
		this.boundingRectBottom = boundingRectCenterY + boundingRectHeight/2;
		this.boundingRectTop = boundingRectCenterY - boundingRectHeight/2;
		this.boundingRectRight = boundingRectCenterX + boundingRectWidth/2;
		this.boundingRectLeft = boundingRectCenterX - boundingRectWidth/2;
		
		
		
	}
}
