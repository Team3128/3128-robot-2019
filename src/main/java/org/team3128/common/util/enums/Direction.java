package org.team3128.common.util.enums;

/**
 * Direction enum used for turns.
 * @author Jamie
 *
 */
public enum Direction
{
	RIGHT("Right"),
	LEFT("Left");
	
	private String name;
	
	private Direction(String name) {
		this.name = name;
	}
	
	public Direction opposite() {
		return (this == Direction.RIGHT) ? Direction.LEFT : Direction.RIGHT;
	}
	
	public String toString() {
		return name;
	}
}
