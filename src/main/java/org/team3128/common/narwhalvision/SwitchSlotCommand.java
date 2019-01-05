package org.team3128.common.narwhalvision;

/**
 * Command which changes the target settings slot in use on the phone
 */

public class SwitchSlotCommand extends PhoneCommand
{
	private int newSlot;

	public SwitchSlotCommand(int newSlot)
	{
		this.newSlot = newSlot;
	}
}
