package org.team3128.common.listener.callbacks;

import org.team3128.common.listener.POVValue;


public interface POVListenerCallback extends IListenerCallback
{
	public void onListener(POVValue pov);
}
