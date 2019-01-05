package org.team3128.common.generics;

interface Loggable {
    public void logInfo(String message);
    public void logDebug(String message);
    public void logFatal(String message);
    public void logFatalException(String message, Exception exception);
    public void logRecoverable(String message);
    public void logUnusual(String message);
}