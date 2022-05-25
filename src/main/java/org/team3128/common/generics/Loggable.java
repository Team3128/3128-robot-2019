package org.team3128.common.generics;

import org.team3128.common.util.Log;

public interface Loggable {
    /**
     * Returns the user-specified category that log messages generated using the
     * Loggable-specific {@link Log} methods should come from. Usually the name of
     * the Class.
     * 
     * @return the category of the {@link Log} message
     */
    public String getTag();
}