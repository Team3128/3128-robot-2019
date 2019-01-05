package org.team3128.common.listener;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.locks.ReentrantLock;

import org.team3128.common.listener.callbacks.AxisListenerCallback;
import org.team3128.common.listener.callbacks.POVListenerCallback;
import org.team3128.common.listener.callbacks.TypelessListenerCallback;
import org.team3128.common.listener.controltypes.Axis;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.Control;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.util.Assert;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.datatypes.Pair;
import org.team3128.common.util.datatypes.SynchronizedMultimap;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class combines the functions of XControl and ListenerManager from the
 * old robot code. It is constructed with one or more Joystick objects. 
 * It polls the controller at a set interval, and invokes listeners whenever a
 * control they're attached to has changed (the button listeners are set for
 * either up or down). You may register the same instance of a listener object for as
 * many controls as you like, but it will only be invoked once per polling cycle
 * no matter how many of its registered controls have changed. However, if you
 * register two different instances of the same listener class for two
 * different controls, those listeners will both be invoked if both controls
 * change.
 * 
 * @author Jamie
 *
 */
public class ListenerManager
{

	// when this is locked no one should touch _joystickValues or _buttonValues
	private ReentrantLock _controlValuesMutex;

	// maps the listeners to the control inputs
	private SynchronizedMultimap<Control, AxisListenerCallback> axisListeners = new SynchronizedMultimap<Control, AxisListenerCallback>();
	private SynchronizedMultimap<Control, Pair<TypelessListenerCallback, Boolean>> buttonListeners = new SynchronizedMultimap<Control, Pair<TypelessListenerCallback, Boolean>>();
	private SynchronizedMultimap<Control, POVListenerCallback> povListeners = new SynchronizedMultimap<Control, POVListenerCallback>();
	private SynchronizedMultimap<Control, TypelessListenerCallback> genericListeners = new SynchronizedMultimap<Control, TypelessListenerCallback>();

	
	private HashMap<String, Control> controlNames;

	// wpilib object which represents a controller
	private ArrayList<Joystick> _joysticks;
		
	//joystick threshold.
	private static final double JOYSTICK_DEADZONE = .15;
	
	private static class ControlValues
	{
		public Set<Button> buttonValues;
		
		public HashMap<Axis, Double> joystickValues;
		
		public HashMap<POV, POVValue> povValues;
		ControlValues()
		{
			buttonValues = new HashSet<>();
			joystickValues = new HashMap<Axis, Double>();
			povValues = new HashMap<POV, POVValue>();
		}
	}
	
	private ControlValues currentControls;

	//zero indexed
	private int numAxes, numPOVs;
	
	//one indexed
	private int numButtons;

	private static final String LOG_TAG = "ListenerManager";
	
	/**
	 * Construct a ListenerManager from joysticks and their type
	 * @param controlType
	 * @param joysticks The joystick or joysticks to pull data from.
	 *   If multiple joysticks are provided, their inputs will be combined additively. That is, if one person presses A and the other presses B, both A and B's listeners will be triggered.
	 *   For axes, whichever value is outside the joystick deadzone will be used.  If both axes are in use, the joystick specified first in the arguments will take precedence.
	 *   
	 *   NOTE: All joysticks are expected to be the same physical model.  If they're not, things are not going to work properly.
	 */
	public ListenerManager(Joystick... joysticks)
	{
		if(joysticks == null || joysticks.length < 1)
		{
			throw new IllegalArgumentException("Invalid joystick arguments");
		}
		
		_controlValuesMutex = new ReentrantLock();
		_joysticks = new ArrayList<>();
		controlNames = new HashMap<>();
		Collections.addAll(_joysticks, joysticks);
		
		recountControls();


		//we want to do this, so that button events will still be sent if buttons are held while the robot is booting
		//currentControls = new ControlValues();
	}
	
	/**
	 * Associate a name with the given control  Throws if the name is already in use.
	 * @param control
	 * @param name
	 */
	public void nameControl(Control control, String name)
	{
		if(controlNames.containsKey(control))
		{
			throw new IllegalArgumentException("This control already has a name!");
		}
		
		controlNames.put(name, control);
	}

	/**
	 * Add a no-argument listener for the given listenable. Multiple listeners can be added
	 * for the same listenable.
	 * 
	 * @param name
	 * @param listener
	 */
	public void addListener(String name, TypelessListenerCallback listener)
	{
		checkControlName(name, null);

		genericListeners.put(controlNames.get(name), listener);
	}
	
	/**
	 * Add a listener for the named POV.
	 */
	public void addListener(String name, POVListenerCallback listener)
	{
		checkControlName(name, POV.class);

		povListeners.put(controlNames.get(name), listener);
	}
	
	/**
	 * Add a button listener for the named button, which will be fired when it is pressed.
	 */
	public void addButtonDownListener(String name, TypelessListenerCallback listener)
	{
		checkControlName(name, Button.class);

		buttonListeners.put(controlNames.get(name), new Pair<>(listener, true));
	}
	
	/**
	 * Add a button listener for the named button, which will be fired when it is released.
	 */
	public void addButtonUpListener(String name, TypelessListenerCallback listener)
	{
		checkControlName(name, Button.class);

		buttonListeners.put(controlNames.get(name), new Pair<>(listener, false));
	}
	
	
	/**
	 * Add a listener for the named axis.
	 * 
	 * Will be called whenever the value changes, except if the change is inside the threshold
	 */
	public void addListener(String name, AxisListenerCallback listener)
	{
		checkControlName(name, Axis.class);

		axisListeners.put(controlNames.get(name), listener);
	}
	
	/**
	 * Fail if the control name is invalid.
	 * 
	 * We can assert here, because this mistake is a coding problem and needs to be fixed right away..
	 */
	private void checkControlName(String name, Class<? extends Control> controlType)
	{
		Assert.notNull(name);
		
		Assert.that(controlNames.containsKey(name), "Unknown control name \"" + name +'\"');
		
		Control namedControl = controlNames.get(name);
		
		if(!(controlType == null || controlType.isInstance(namedControl)))
		{
			Assert.fail("Tried to use \"" + name + "\" as an " + controlType.getSimpleName() + ", but that name is registered to a(n) " + namedControl.getClass().getSimpleName()); 
		}
	}
	
	/**
	 * Add a listener for the given list of control names.
	 * 
	 * @param names
	 * @param listener
	 */
	public void addMultiListener(TypelessListenerCallback listener, String... names)
	{
		Assert.greaterThan(names.length, 0);
		
		for(String controlName : names)
		{
			checkControlName(controlName, null);
			
			genericListeners.put(controlNames.get(controlName), listener);

		}
	}

	/**
	 * Remove all listeners set for the given name.
	 */
	public void removeAllListenersForControl(String name)
	{
		Control control = controlNames.get(name);
		
		genericListeners.removeAll(control);
		
		if(control instanceof Button)
		{
			buttonListeners.removeAll(control);
		}
		else if(control instanceof Axis)
		{
			axisListeners.removeAll(control);
		}
		else if(control instanceof POV)
		{
			povListeners.removeAll(control);
		}
	}

	//
	//
	//

	/**
	 * Returns the boolean value of a button by name.
	 * 
	 * This function is thread-safe, and can be called at the same time as tick().
	 */
	public boolean getButton(String name)
	{
		checkControlName(name, Button.class);
		
		_controlValuesMutex.lock();
		
		boolean retval;

		retval = currentControls.buttonValues.contains(controlNames.get(name));
		_controlValuesMutex.unlock();
		
		return retval;
	}

	/**
	 * Get the value of an axis.
	 * 
	 * This value is automatically thresheld to JOYSTICK_DEADZONE.
	 * 
	 * This function is thread-safe, and can be called at the same time as tick().
	 * 
	 * @param axis
	 * @return
	 */
	public double getAxis(String name)
	{
		checkControlName(name, Axis.class);
		
		_controlValuesMutex.lock();
		double retval = 0.0;
		
		Control axis = controlNames.get(name);
		
		if(currentControls.joystickValues.containsKey(axis))
		{
			retval = currentControls.joystickValues.get(axis);
		}
		
		_controlValuesMutex.unlock();
		return retval;
	}
	
	/**
	 * Get the value of a POV.
	 * 
	 * This function is thread-safe, and can be called at the same time as tick().
	 * 
	 * @param axis
	 * @return
	 */
	public POVValue getPOV(String name)
	{
		checkControlName(name, POV.class);
		
		_controlValuesMutex.lock();
		POVValue retval = null;

		retval = currentControls.povValues.get(controlNames.get(name));
		
		_controlValuesMutex.unlock();
		return retval;
	}
	
	/**
	 * Collect control information from all joysticks.
	 * @return
	 */
	ControlValues pollAllJoysticks()
	{
		ControlValues newControls = new ControlValues();

		
		_controlValuesMutex.lock();

		for(int index = _joysticks.size() - 1; index >= 0; --index)
		{
			Joystick currentJoystick = _joysticks.get(index);			
			// read button values
			for (int counter = 1; counter <= numButtons; counter++)
			{
				boolean buttonValue = currentJoystick.getRawButton(counter);

				if(buttonValue)
				{
					newControls.buttonValues.add(new Button(counter));
				}
			}
			
			//Log.debug(LOG_TAG, "Buttons: " + newControls.buttonValues.toString());

			// read joystick values
			for (int counter = 0; counter <= numAxes; counter++)
			{
				Axis currAxis = new Axis(counter);
				double thisJoystickValue = RobotMath.thresh(currentJoystick.getRawAxis(counter), JOYSTICK_DEADZONE);
				if((!newControls.joystickValues.containsKey(currAxis)) || Math.abs(thisJoystickValue) > JOYSTICK_DEADZONE)
				{
					newControls.joystickValues.put(currAxis, thisJoystickValue);
				}
			}
			
			// read POV values
			for (int counter = 0; counter <= numPOVs; counter++)
			{
				POVValue value = POVValue.fromWPILibAngle(currentJoystick.getPOV(counter));
				if(newControls.povValues.size() - 1 <= counter)
				{
					POVValue oldValue = newControls.povValues.get(new POV(counter));
					if(oldValue == null || oldValue.getDirectionValue() == 0)
					{
						newControls.povValues.put(new POV(counter), value);
					}
					
					//else, use the preexisting value
				}
				else
				{
					//add a new value
					newControls.povValues.put(new POV(counter), value);

				}
			
			}
		}
		_controlValuesMutex.unlock();
				
		return newControls;

	}
	
	private void addTypelessListenersForControl(Set<TypelessListenerCallback> listeners, Control control)
	{
		// get all its registered listeners
		Collection<TypelessListenerCallback> foundListeners = genericListeners.get(control);

		if (foundListeners != null && !foundListeners.isEmpty())
		{
			// loop through them
			for (TypelessListenerCallback callback : foundListeners)
			{
				listeners.add(callback);
			}
		}

	}

	/**
	 * Read controls and invoke listeners. Usually called by the robot main class.
	 */
	public void tick()
	{
		ControlValues newControls = pollAllJoysticks();
			
		//don't need to lock _controlValuesMutex here because we know it's not going to be modified, because we're the only ones modifying it

		//save the old controls and swap in the new ones, so that if/when listeners check they will get the new data
		ControlValues oldControls = currentControls;
		
		// update class variables to match new data
		{
			_controlValuesMutex.lock();
			currentControls = newControls;
			_controlValuesMutex.unlock();
		}
		
		invokeListeners(oldControls, newControls);

	}
	
	/**
	 * fire all the listeners for changed control values, based on the provided old and new values.
	 * 
	 * Does exception handling.
	 * @param oldControls
	 * @param newControls
	 */
	private void invokeListeners(ControlValues oldControls, ControlValues newControls)
	{
		try
		{
			//if the same generic listener is registered for multiple types of control, we need to execute it only once
			//so we collect all of the generic listeners to execute in here to de-duplicate them.
			Set<TypelessListenerCallback> genericListenersToInvoke = new HashSet<TypelessListenerCallback>();
			
			//buttons
			//--------------------------------------------------------------------------------------------------------------------------------------------------
	
			//check for pressed buttons
			for(Button button : newControls.buttonValues)
			{
				if(!oldControls.buttonValues.contains(button))
				{
					addTypelessListenersForControl(genericListenersToInvoke, button);
					
					// get all its registered listeners
					HashSet<Pair<TypelessListenerCallback, Boolean>> foundListeners = buttonListeners.get(button);
	
					if (foundListeners != null && !foundListeners.isEmpty())
					{
						// loop through them
						for (Pair<TypelessListenerCallback, Boolean> callbackPair : foundListeners)
						{
							//button-press listener
							if(callbackPair.right)
							{
								callbackPair.left.onListener();
							}
						}
					}
				}
			}
			
			//check for unpressed buttons
			for(Button button : oldControls.buttonValues)
			{
				if(!newControls.buttonValues.contains(button))
				{
					addTypelessListenersForControl(genericListenersToInvoke, button);
					
					// get all its registered listeners
					HashSet<Pair<TypelessListenerCallback, Boolean>> foundListeners = buttonListeners.get(button);
	
					if (foundListeners != null && !foundListeners.isEmpty())
					{
						// loop through them
						for (Pair<TypelessListenerCallback, Boolean> callbackPair : foundListeners)
						{
							//button-release listener
							if(!callbackPair.right)
							{
								callbackPair.left.onListener();
							}
						}
					}
				}
			}

		// loop through joystick values
		for (Axis axis : newControls.joystickValues.keySet())
		{
			if(currentControls.joystickValues.containsKey(axis))
			{
				// has this particular value changed?
				if(oldControls.joystickValues.containsKey(axis)) //sanity check to prevent NPE if the axes have changed
				{
					double newValue = newControls.joystickValues.get(axis);
					
					if (Math.abs(oldControls.joystickValues.get(axis) - newValue) > .0001) 
					{
						addTypelessListenersForControl(genericListenersToInvoke, axis);
						
						// get all its registered listeners
						HashSet<AxisListenerCallback> foundListeners = axisListeners.get(axis);
		
						if (foundListeners != null && !foundListeners.isEmpty())
						{
							// loop through them
							for (AxisListenerCallback callback : foundListeners)
							{
								
								//Log.debug("ListenerManager", "Invoking listener for axis " + axis.getIndex() + " with value " + newValue);

								callback.onListener(newValue);
							}
						}
		
					}
				}
				}
			}
			
			//POVs
			//--------------------------------------------------------------------------------------------------------------------------------------------------
						
			for(Map.Entry<POV, POVValue> newPOVEntry : newControls.povValues.entrySet())
			{
				POVValue oldPOVValue = oldControls.povValues.get(newPOVEntry.getKey());
				
				//Log.debug("ListenerManager", "old POV: " + String.valueOf(oldPOVValue) + " new POV: " + newPOVEntry.toString());

				
				if(oldPOVValue != null && !oldPOVValue.equals(newPOVEntry.getValue()))
				{
					addTypelessListenersForControl(genericListenersToInvoke, newPOVEntry.getKey());
					
					// get all its registered listeners
					HashSet<POVListenerCallback> foundListeners = povListeners.get(newPOVEntry.getKey());
	
					if (foundListeners != null && !foundListeners.isEmpty())
					{
						// loop through them
						for (POVListenerCallback callback : foundListeners)
						{
							callback.onListener(newPOVEntry.getValue());
						}
					}
				}
			}
	
	
	
			// invoke generic handlers, once they've been merged.
			for (TypelessListenerCallback listener : genericListenersToInvoke)
			{
				listener.onListener();
			}
		}
		
		//we invoke the listeners at the bottom of each of those nested loops, andit's impractical to repeat the catch block 5 times
		//so we just have to put the whole thing in the catch block.
		catch (RuntimeException error) 
		{
			Log.recoverable(
					"ControlWatcher",
					"Caught a " + error.getClass().getSimpleName()
							+ " from a control listener: "
							+ error.getMessage());
			error.printStackTrace();
		}
	}
	
	/**
	 * Set the joystick(s) used by the listener manager.  Replaces the current set of joysticks.
	 * @param joysticks
	 */
	public void setJoysticks(Joystick...joysticks)
	{
		if(joysticks.length < 1)
		{
			throw new IllegalArgumentException("No joysticks provided!");
		}
		_controlValuesMutex.lock();
		
		_joysticks.clear();
		Collections.addAll(_joysticks, joysticks);
		
		_controlValuesMutex.unlock();
	}
	
	/**
	 * Under certain conditions, such as when the roborio first boots up, Joystick.getNumButtons() (and possibly the other two such functions)
	 * can return bad data because WPILib has NO FRIGGIN' IDEA how many buttons are on the joystick until it connects to the Driver Station.
	 * Call this function at a later time after the robot has a connection to re-get the correct values.
	 * 
	 * (called automatically by NarwhalRobot)
	 */
	public void recountControls()
	{
		_controlValuesMutex.lock();
		
		Joystick joyToTest = _joysticks.get(0); //all joysticks are assumed to have the same number of buttons
		numButtons = joyToTest.getButtonCount();
		numAxes = joyToTest.getAxisCount() - 1;

		numPOVs = joyToTest.getPOVCount() - 1;
		_controlValuesMutex.unlock();
		
		//remake the controls arrays with the correct length
		currentControls = pollAllJoysticks();
		
		Log.info(LOG_TAG, String.format("Joystick: %d buttons, %d axes, %d POVs",  numButtons, numAxes + 1, numPOVs + 1));
	}
	
	/**
	 * Update all listeners, and the stored control values, as if all of the controls had gone back to their zeroed / rest state.
	 * 
	 * Button-unpressed listeners are fired, and axis and POV listeners are updated that their control has gone to zero.
	 */
	public void zeroOutListeners()
	{
		ControlValues allZeroValues = new ControlValues();
		
		for(int axisIndex = 0; axisIndex <= numAxes; ++axisIndex)
		{
			allZeroValues.joystickValues.put(new Axis(axisIndex), 0.0);
		}
		
		for(int povIndex = 0; povIndex <= numPOVs; ++povIndex)
		{
			allZeroValues.povValues.put(new POV(povIndex), new POVValue(0));
		}
		
		//save the old controls and swap in the new ones, so that if/when listeners check they will get the new data
		ControlValues oldControls = currentControls;
		
		{
			_controlValuesMutex.lock();
			currentControls = allZeroValues;
			_controlValuesMutex.unlock();
		}
		
		invokeListeners(oldControls, allZeroValues);
	}

}
