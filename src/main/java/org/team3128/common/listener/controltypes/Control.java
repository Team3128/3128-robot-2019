package org.team3128.common.listener.controltypes;

/**
 * Superclass for objects that can be used as ControlWatcher controls.
 * 
 * It implements hashCode() and equals(), so it can be used as a HashMap key. 
 *
 * @author Jamie
 *
 */
public abstract class Control
{
	//ID of the control type, a prime number used for hashCode() and equals()
	private int typeid;
	
	//index of the control on the controller (0-indexed).
	private int index;
	
	protected Control(int typeid, int index)
	{
		this.typeid = typeid;
		this.index = index;
	}
	
	public int getIndex()
	{
		return index;
	}
	
	@Override
	public boolean equals(Object object)
	{
		if(object instanceof Control)
		{
			Control other = (Control)object;
			return index == other.index && typeid == other.typeid;
		}
		
		return false;
	}
	
	@Override
	public int hashCode()
	{
		return index * 1000 + typeid;
	}
}
