package org.team3128.common.narwhalvision;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import org.team3128.common.util.Assert;
import org.team3128.common.util.Log;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.ByteBufferInput;
import com.esotericsoftware.kryo.io.ByteBufferOutput;

public class NarwhalVisionReceiver
{
	final static String TAG = "NarwhalVisionReceiver";

	private DatagramSocket visionDataSocket;
	private Thread internalThread;
	
	private Kryo kryo;
	private ByteBufferInput packetReader;
	private ByteBufferOutput packetWriter;
	
	private long lastPacketRecvTime = 0;
	private String coprocessorIPAddress = "10.31.28.xxx";
	
	//make sure to only access through synchronized functions, updated asynchronously
	private ArrayList<TargetInformation> mostRecentTargets;
	
	public NarwhalVisionReceiver()
	{
		try
		{
			visionDataSocket = new DatagramSocket(3128);
		}
		catch(SocketException e)
		{
			e.printStackTrace();
		}
		
		kryo = new Kryo();
		kryo.register(TargetInformation.class, 0);
		kryo.register(SwitchSlotCommand.class, 1);
		packetReader = new ByteBufferInput();
		packetWriter = new ByteBufferOutput();
		mostRecentTargets = new ArrayList<>();
		
		internalThread = new Thread(this::receiveLoop);
		internalThread.start();
	}
	
	// static buffer used to hold packets/serialized objects
	// no, as far as I can tell, there's no way to not have a fixed size buffer
	final static int SERIALIZATION_BUFFER_SIZE=1024;
	
	private void receiveLoop()
	{
		byte[] recvBuffer = new byte[SERIALIZATION_BUFFER_SIZE];
		
		DatagramPacket visionPacket = new DatagramPacket(recvBuffer, SERIALIZATION_BUFFER_SIZE);
		
		while(true)
		{	
			try
			{
				Log.info(TAG, "Trying to recieve packet.");
				visionDataSocket.receive(visionPacket);
			}
			catch(IOException e)
			{
				Log.recoverable(TAG, "Failed to receive vision packet: " + e.getMessage());
				e.printStackTrace();
				
				setCoprocessorIP("Not Connected...");
				
				continue;
			}
			
			setCoprocessorIP(visionPacket.getAddress().toString());
			
			Log.info(TAG, "Setting buffer...");
			packetReader.setBuffer(recvBuffer);
			
			try
			{
				Log.info(TAG, "Trying to decode a packet.");
				TargetInformation targetInfo = kryo.readObject(packetReader, TargetInformation.class);
				onTargetInfoReceived(targetInfo);
				setLastPacketReceivedTime(System.currentTimeMillis());
				
				Log.debug("NarwhalVisionReceiver", "Got a target information packet: " + targetInfo);
			}
			catch(ClassCastException ex)
			{
				Log.recoverable(TAG, "Received the wrong class from the phone: " + ex.getMessage());
				ex.printStackTrace();
			}
			catch(RuntimeException ex)
			{
				Log.recoverable(TAG, "Error deserializing: " + ex.getMessage());
				ex.getMessage();
			}
			
		}
	}
	
	/**
	 * Sends a command to the phone
	 */
	protected void sendCommand(PhoneCommand command)
	{
		ByteBuffer serializedBytes = ByteBuffer.allocate(SERIALIZATION_BUFFER_SIZE);
		packetWriter.setBuffer(serializedBytes);

		kryo.writeClassAndObject(packetWriter, command);

		packetWriter.flush();

		DatagramPacket packet = new DatagramPacket(serializedBytes.array(), SERIALIZATION_BUFFER_SIZE);

		try
		{
			visionDataSocket.send(packet);
		}
		catch (IOException e)
		{
			Log.recoverable(TAG, "Failed to send command to phone: " + e.getMessage());
			e.printStackTrace();
		}
	}
	
	/**
	 * Change the target data slot in use on the phone
	 * @param slot
	 */
	public void setActiveSlot(int slot)
	{
		Assert.inRange(slot, 1, 4);
		sendCommand(new SwitchSlotCommand(slot));
	}
	
	//these functions are synchronized because they return data set by the thread
	
	/**
	 * Gets the UNIX timestamp of when the last packet was received.  If the value is 0, no packet has ever been received.
	 * @return
	 */
	public synchronized long getLastPacketReceivedTime()
	{
		return lastPacketRecvTime;
	}
	
	private synchronized void setLastPacketReceivedTime(long time)
	{
		lastPacketRecvTime = time;
	}
	
	/**
	 * Gets the String literal of the IP Address that the packet was recieved from. If an iteration of the reviever loop has occured without recieving a packet, the IP will return as "Not Connected...".
	 * @return
	 */
	public synchronized String getCoprocessorIP() {
		return coprocessorIPAddress;
	}
	
	private synchronized void setCoprocessorIP(String ipLiteral) {
		coprocessorIPAddress = ipLiteral;
	}
	
	/**
	 * Get the most recent target information sent by the phone.  Targets are indexed by how closely they fit the criteria; the 0th target is the best match
	 * @return
	 */
	public synchronized TargetInformation[] getMostRecentTargets()
	{
		//make a shallow copy so that when the list is next cleared, it doesn't affect the return value of this method.
		return mostRecentTargets.toArray(new TargetInformation[mostRecentTargets.size()]);
	}
	
	private synchronized void onTargetInfoReceived(TargetInformation target)
	{
		//if the index is 1, clear the list and start over because we are getting data from the next frame
		if(target.targetRanking == 1)
		{
			mostRecentTargets.clear();
			mostRecentTargets.add(target);
		}
		else
		{
			mostRecentTargets.add(target);
		}
	}
	
}
