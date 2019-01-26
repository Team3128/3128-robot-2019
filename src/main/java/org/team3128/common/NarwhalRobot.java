package org.team3128.common;

import edu.wpi.first.hal.HAL;

import java.util.ArrayList;

import org.team3128.common.listener.ListenerManager;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.util.Assert;
import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.NotifierJNI;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/**
* NarwhalRobot, based on the TimedRobot, implements a specific type of robot program framework, extending the RobotBase
* class.
*/
@SuppressWarnings("PMD.TooManyMethods")
public abstract class NarwhalRobot extends RobotBase {
    /* ----------- Overridable initialization code ----------------- */
    
    /**
    * Robot-wide initialization code should go here.
    *
    * <p>Users should override this method for default Robot-wide initialization which will be called
    * when the robot is first powered on. It will be called exactly one time.
    *
    * <p>Warning: the Driver Station "Robot Code" light and FMS "Robot Ready" indicators will be off
    * until constructHardware() exits. Code in constructHardware() that waits for enable will cause
    * the robot to never indicate that the code is ready, causing the robot to be bypassed in a match.
    */
    protected abstract void constructHardware();

    /**
     * Use this to create all of the teleoperated listeners.
     * 
     * Make sure to add all of your ListenerManagers to this class to be ticked using {@link addListenerManager()}
     */
    protected abstract void setupListeners();

    /**
	 * Construct your autonomous programs and add them to this SendableChooser here.
	 * 
	 * This function will called multiple times.
	 */
	protected void constructAutoPrograms() {}
    
    /**
    * Initialization code for disabled mode should go here.
    *
    * <p>Users should override this method for initialization code which will be called each time the
    * robot enters disabled mode.
    */
    protected void disabledInit() {}
    
    /**
    * Initialization code for autonomous mode should go here.
    *
    * <p>Users should override this method for initialization code which will be called each time the
    * robot enters autonomous mode.
    */
    protected void autonomousInit() {}
    
    /**
    * Initialization code for teleop mode should go here.
    *
    * <p>Users should override this method for initialization code which will be called each time the
    * robot enters teleop mode.
    */
    protected void teleopInit() {}
    
    /**
    * Initialization code for test mode should go here.
    *
    * <p>Users should override this method for initialization code which will be called each time the
    * robot enters test mode.
    */
    @SuppressWarnings("PMD.JUnit4TestShouldUseTestAnnotation")
    protected void testInit() {}
    
    
    /**
    * Periodic code for all robot modes should go here.
    */
    protected void robotPeriodic() {}
        
    /**
    * Periodic code for disabled mode should go here.
    */
    protected void disabledPeriodic() {}
        
    /**
    * Periodic code for autonomous mode should go here.
    */
    protected void autonomousPeriodic() {}
        
    /**
    * Periodic code for teleop mode should go here.
    */
    protected void teleopPeriodic() {}
    
    /**
    * Periodic code for test mode should go here.
    */
    @SuppressWarnings("PMD.JUnit4TestShouldUseTestAnnotation")
    protected void testPeriodic() {}


    /**
	 * Use this function to read and write data from NarwhalDashboard.
	 * It is called asynchronously, no matter what mode the robot is in.
	 */
	protected void updateDashboard() {}

    /* ----------- Overridable periodic code ----------------- */

    protected double m_period;
    public static final double kDefaultPeriod = 0.02;
    
    private enum Mode {
        kNone,
        kDisabled,
        kAutonomous,
        kTeleop,
        kTest
    }
    
    private Mode m_lastMode = Mode.kNone;
    private final Watchdog m_watchdog;
    
    // The C pointer to the notifier object. We don't use it directly, it is
    // just passed to the JNI bindings.
    private final int m_notifier = NotifierJNI.initializeNotifier();
    
    // The absolute expiration time
    private double m_expirationTime;


    final static int dashboardUpdateWavelength = NarwhalDashboard.getUpdateWavelength();
    Thread dashboardUpdateThread;


    ArrayList<ListenerManager> listenerManagers = new ArrayList<ListenerManager>();

    
    /**
    * Constructor for NarwhalRobot
    */
    protected NarwhalRobot() {
        this(kDefaultPeriod);
    }
    
    /**
    * Constructor for NarwhalRobot.
    *
    * @param period Period in seconds.
    */
    protected NarwhalRobot(double period) {
        m_period = period;
        m_watchdog = new Watchdog(period, this::printLoopOverrunMessage);
        
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
    }
    
    /**
    * Provide an alternate "main loop" via startCompetition().
    */
    @Override
    @SuppressWarnings("UnsafeFinalization")
    public void startCompetition() {
        Log.info("NarwhalRobot", "Welcome to the FRC Team 3128 Common Library version 4.1!");
        Log.info("NarwhalRobot", "Initializing Base Robot...");
        
        Assert.setRobot(this);
        NarwhalDashboard.startServer();
        
        // Attempt to construct hardware
        try {
            constructHardware();
        } catch (RuntimeException ex) {
            Log.fatalException("NarwhalRobot", "Exception constructing hardware", ex);
            ex.printStackTrace();
            fail();
        }

        // Attempt to setup ListenerManager
        try
        {
        	setupListeners();
        }
        catch(RuntimeException ex)
        {
        	Log.fatalException("NarwhalRobot", "Exception seting up listeners", ex);
        	ex.printStackTrace();
        	fail();
        }
        
        // Construct auto programs for the first time
        setupAutoChooser();

        Log.info("NarwhalRobot", "Starting Dashboard Update Thread...");
        dashboardUpdateThread = new Thread(this::updateDashboardLoop, "Dashboard Update Thread");
        dashboardUpdateThread.start();
        
        // Tell the DS that the robot is ready to be enabled
        HAL.observeUserProgramStarting();
        
        m_expirationTime = RobotController.getFPGATime() * 1e-6 + m_period;
        updateAlarm();
        
        // Loop forever, calling the appropriate mode-dependent function
        while (true) {
            long curTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
            if (curTime == 0) {
                break;
            }
            
            m_expirationTime += m_period;
            updateAlarm();
            
            loopFunc();
        }
    }
        
    protected void loopFunc() {
        m_watchdog.reset();
        
        // Call the appropriate function depending upon the current robot mode
        if (isDisabled()) {
            // Call DisabledInit() if we are now just entering disabled mode from either a different mode
            // or from power-on.
            if (m_lastMode != Mode.kDisabled) {
                Log.info("NarwhalRobot", "Entering disabled period.");

                LiveWindow.setEnabled(false);
                Shuffleboard.disableActuatorWidgets();
                disabledInit();
                m_watchdog.addEpoch("disabledInit()");

                if (m_lastMode == Mode.kAutonomous) {
                    Log.info("NarwhalRobot", "Re-constructing autonomous sequences");
                    setupAutoChooser();
                }
                zeroOutListeners();

                m_lastMode = Mode.kDisabled;
            }
            
            HAL.observeUserProgramDisabled();
            disabledPeriodic();
            m_watchdog.addEpoch("disablePeriodic()");
        } else if (isAutonomous()) {
            // Call AutonomousInit() if we are now just entering autonomous mode from either a different
            // mode or from power-on.
            if (m_lastMode != Mode.kAutonomous) {
                Log.info("NarwhalRobot", "Entering autonomous period.");

                setupAutoChooser();
                zeroOutListeners();

                autonomousInit();
                runAutoProgram();

                LiveWindow.setEnabled(false);
                Shuffleboard.disableActuatorWidgets();
                autonomousInit();
                m_watchdog.addEpoch("autonomousInit()");
                m_lastMode = Mode.kAutonomous;
            }
            
            HAL.observeUserProgramAutonomous();
            Scheduler.getInstance().run();
            autonomousPeriodic();
            m_watchdog.addEpoch("autonomousPeriodic()");
        } else if (isOperatorControl()) {
            // Call TeleopInit() if we are now just entering teleop mode from either a different mode or
            // from power-on.
            if (m_lastMode != Mode.kTeleop) {
                Log.info("NarwhalRobot", "Entering teleoperated period.");

                LiveWindow.setEnabled(false);
                Shuffleboard.disableActuatorWidgets();

                zeroOutListeners();
                teleopInit();
                recountAllControls();

                m_watchdog.addEpoch("teleopInit()");
                m_lastMode = Mode.kTeleop;
            }
            
            HAL.observeUserProgramTeleop();
            tickListenerManagers();
            teleopPeriodic();
            m_watchdog.addEpoch("teleopPeriodic()");
        } else {
            // Call TestInit() if we are now just entering test mode from either a different mode or from
            // power-on.
            if (m_lastMode != Mode.kTest) {
                Log.info("NarwhalRobot", "Entering test period.");

                LiveWindow.setEnabled(true);
                Shuffleboard.enableActuatorWidgets();
                testInit();
                m_watchdog.addEpoch("testInit()");
                m_lastMode = Mode.kTest;
            }
            
            HAL.observeUserProgramTest();
            testPeriodic();
            m_watchdog.addEpoch("testPeriodic()");
        }
        
        robotPeriodic();
        m_watchdog.addEpoch("robotPeriodic()");
        m_watchdog.disable();
        SmartDashboard.updateValues();
        
        LiveWindow.updateValues();
        Shuffleboard.update();
        
        // Warn on loop time overruns
        if (m_watchdog.isExpired()) {
            m_watchdog.printEpochs();
        }
    }
    
    private void printLoopOverrunMessage() {
        DriverStation.reportWarning("Loop time of " + m_period + "s overrun\n", false);
    }
    
    @Override
    @SuppressWarnings("NoFinalizer")
    protected void finalize() {
        NotifierJNI.stopNotifier(m_notifier);
        NotifierJNI.cleanNotifier(m_notifier);
    }
    
    /**
    * Get time period between calls to Periodic() functions.
    */
    public double getPeriod() {
        return m_period;
    }
    
    /**
    * Update the alarm hardware to reflect the next alarm.
    */
    @SuppressWarnings("UnsafeFinalization")
    private void updateAlarm() {
        NotifierJNI.updateNotifierAlarm(m_notifier, (long) (m_expirationTime * 1e6));
    }
    
    private void setupAutoChooser()
	{
        Log.info("NarwhalRobot", "Setting Up Autonomous Chooser...");
		Scheduler.getInstance().removeAll(); // get rid of any paused commands
		
		NarwhalDashboard.clearAutos();
        constructAutoPrograms();
        
        NarwhalDashboard.pushAutos();
	}
	
	private void runAutoProgram()
	{
		CommandGroup autoProgram = null;

		autoProgram = NarwhalDashboard.getSelectedAuto();

		if(autoProgram == null)
		{
			Log.recoverable("NarwhalRobot", "Can't start autonomous, there is no sequence to run.  "
				+ "You either didn't provide any sequences, or didn't set a default and didn't select one on the dashboard.");
		}
		else
		{
			Log.info("NarwhalRobot", "Running auto sequence \"" + autoProgram.getName() + "\"");
			autoProgram.start();
		}
	}

    protected void fail() {
        Log.fatal("NarwhalRobot", "Critical error. Robot exiting.");
        
        //give the failure message time to get to the driver station
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        System.exit(7);
    }

    /**
     * This function is run in its own thread to call main.updateDashboard()
     */
    private void updateDashboardLoop()
    {
		Log.info("NarwhalRobot", "Dashboard Update Thread starting");
    	while(true)
    	{
    		updateDashboard();
    		
    		try
			{
				Thread.sleep(dashboardUpdateWavelength);
			} 
    		catch (InterruptedException e)
			{
    			Log.info("NarwhalRobot", "Dashboard Update Thread shutting down");
				return;
			}
    	}
    }

    /**
     * Add a listener manager to the list of ones to be ticked in teleopPeriodic().
     * @param manager
     */
    public void addListenerManager(ListenerManager manager)
    {
    	Assert.notNull(manager);
    	listenerManagers.add(manager);
    }

    //works around an annoying (though understandable) WPILib issue:
    //if the DS is not connected, it has no idea how many buttons/axes/POVs there are on a joystick
    //so it seems to just make up a number
    //so when we start teleop we recount them, since the DS must be connected by now.
    private void recountAllControls()
    {
    	for(ListenerManager manager : listenerManagers)
    	{
    		manager.recountControls();
    	}
    }

    // DO YOU REALLY WANT TO MODIFY YOUR SOUL?
    public void tickListenerManagers()
    {        
    	
    	for(ListenerManager manager : listenerManagers)
    	{
    		manager.tick();
    	}
    }

    /**
     * Updates all listeners with zero/unpressed values for all controls. Prevents
     * the robot from immediately moving when enabled.
     */
    public void zeroOutListeners()
    {
    	for(ListenerManager manager : listenerManagers)
    	{
    		manager.zeroOutListeners();
    	}
    }
}
