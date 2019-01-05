package org.team3128.common.generics;

import org.team3128.common.util.Log;

public abstract class TaggedLoggable implements Loggable {
    protected abstract String getTag();

    public void logInfo(String message) {
        Log.info(getTag(), message);
        System.out.println();
    }

    public void logDebug(String message) {
        Log.debug(getTag(), message);
    }

    public void logFatal(String message) {
        Log.fatal(getTag(), message);
    }

    public void logFatalException(String message, Exception exception) {
        Log.fatalException(getTag(), message, exception);
    }

    public void logRecoverable(String message) {
        Log.recoverable(getTag(), message);
    }

    public void logUnusual(String message) {
        Log.unusual(getTag(), message);
    }
}