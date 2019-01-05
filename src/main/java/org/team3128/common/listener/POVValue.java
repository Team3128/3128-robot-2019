package org.team3128.common.listener;


/**
 * Class to represent the position of POV on a joystick.  It can be in one of 9 positions:
 * The cardinal directions, the four sub-cardinal (diagonal?) directions, or the center position.
 * These are numbered according to the diagram below.
 * 
 * <pre>
 *      8
 *   7     1
 *  
 * 6    0    2
 *    
 *   5     3
 *      4
 *      </pre>
 * 
 * @author Narwhal
 *
 */
public class POVValue
{	
	int directionValue;

	public int getDirectionValue() {
		return directionValue;
	}

	/**
	 * Construct a POV control value from the angle it should match.
	 * 
	 * <pre>
	 *      1
	 *   8     2
	 *  
	 * 7    0    3
	 *    
	 *   6     4
	 *      5
     </pre>
	 * @param directionValue
	 */
	public POVValue(int directionValue) {
		
		if(directionValue < 0 || directionValue > 8)
		{
			throw new IllegalArgumentException("Direction value out of range");
		}
		
		this.directionValue = directionValue;
	}
	
	/**
	 * Creates a POV control from the value returned by Joystick.getPOV()
	 * @param angle
	 */
	public static POVValue fromWPILibAngle(int angle)
	{
		if(angle < 0)
		{
			return new POVValue(0);
		}
		
		int value = 8 - (angle/ 45);
		

		
		return new POVValue(value);
	}
	
	@Override
	public int hashCode()
	{
		return directionValue * 1000 + 19;
	}

	@Override
	public boolean equals(Object object)
	{
		if(object instanceof POVValue)
		{
			POVValue otherPOV = (POVValue)object;
			return directionValue == otherPOV.directionValue;
		}
		
		return false;
	}
	
	@Override
	public String toString()
	{
		return "POVValue: direction: " + directionValue;
	}

}
