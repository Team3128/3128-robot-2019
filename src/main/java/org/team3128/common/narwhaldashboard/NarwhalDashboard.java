package org.team3128.common.narwhaldashboard;

import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.LinkedHashMap;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class NarwhalDashboard extends WebSocketServer {
    private static final int PORT = 5800;
    private final static int UPDATE_WAVELENGTH = 100;
    public static int getUpdateWavelength() {
        return UPDATE_WAVELENGTH;
    }

    private static HashMap<String, String> data = new HashMap<String, String>();
    private static LinkedHashMap<String, CommandGroup> autoPrograms = new LinkedHashMap<String, CommandGroup>();

    private static HashMap<String, DashButtonCallback> buttons = new HashMap<String, DashButtonCallback>();

    private static String selectedAuto = null;
    private static boolean autosPushed = false;

    public NarwhalDashboard(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
    }

    public NarwhalDashboard(InetSocketAddress address) {
        super(address);
    }

    /**
     * Publishes a numerical value to NarwhalDashboard
     */
    public static void put(String key, double value) {
        put(key, Double.toString(value));
    }

    /**
     * Publishes a boolean value to NarwhalDashboard
     */
    public static void put(String key, boolean value) {
        put(key, Boolean.toString(value));
    }

    /**
     * Publishes a string value to NarwhalDashboard
     */
    public static void put(String key, String value) {
        data.put(key, value);
    }

    public static void addButton(String key, DashButtonCallback callback) {
        buttons.put(key, callback);
    }

    /**
     * Clears the list of autonomous programs.
     */
    public static void clearAutos() {
        autoPrograms = new LinkedHashMap<String, CommandGroup>();
    }

    /**
     * Adds an autonomous program to NarwhalDashboard's auto picker
     * 
     * @param name - The human-readable name of the autonomous program
     * @param program - The auto program to run if this element is chosen
     */
    public static void addAuto(String name, CommandGroup program) {
        autoPrograms.put(name, program);
    }

    /**
     * Sends new set of autonomous programs to NarwhalDashboard.
     */
    public static void pushAutos() {
        autosPushed = false;
    }

    /**
     * Returns the currently selected auto program
     */
    public static CommandGroup getSelectedAuto() {
        if (selectedAuto == null) return null;

        if (!autoPrograms.containsKey(selectedAuto)) {
            Log.recoverable("NarwhalDashboard", "Auto program \"" + selectedAuto + "\" does not exist. Perhaps it was deleted between its selection and the beginning of the autonomous period?");
        }

		return autoPrograms.get(selectedAuto);
	}

    /**
     * Starts the NarwhalDashboard server. This opens it up to be able to be
     * connected to by client devices (the DS Laptop, a tablet controller, etc)
     * and begins streaming data. 
     */
    public static void startServer() {
        NarwhalDashboard s;
		try {
            s = new NarwhalDashboard(PORT);
            s.setReuseAddr(true);
            s.start();
            
            Log.info("NarwhalDashboard", "Server has started on port " + PORT);
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}  
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        Log.info("NarwhalDashboard", conn.getRemoteSocketAddress().getHostName() + " has opened a connection.");

        (new Thread(() -> {
            while (conn.isOpen()) {
                String jsonString = "{";

                for (String key : data.keySet()) {
                    jsonString += "\"" + key + "\":\"" + data.get(key) + "\",";
                }

                jsonString += "\"selected_auto\":\"" + selectedAuto + "\",";

                // jsonString += "\"buttons\":[";
                // for (String buttonName : buttons.keySet()) {
                //     jsonString += "\"" + buttonName +  "\",";
                // }
                // if (!buttons.isEmpty())
                //     jsonString = jsonString.substring(0, jsonString.length() - 1);
                // jsonString += "],";
                
                if (!autosPushed) {
                    jsonString += "\"auto_programs\":[";
                    for (String autoName : autoPrograms.keySet()) {
                        jsonString += "\"" + autoName +  "\",";
                    }
                    if (!autoPrograms.isEmpty())
                        jsonString = jsonString.substring(0, jsonString.length() - 1);
                    jsonString += "]";

                    autosPushed = true;
                }
                
                jsonString += "}";

                conn.send(jsonString);

                try {
                    Thread.sleep(UPDATE_WAVELENGTH);
                }
                catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        })).start();
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        Log.info("NarwhalDashboard", conn.getRemoteSocketAddress().getHostName() + " has closed its connection.");
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        Log.info("NarwhalDashboard", message);
        String[] parts = message.split(":");

        if (parts[0].equals("selectAuto")) {
            String programName = parts[1];

            if (programName.equals("null")) {
                selectedAuto = null;
            }
            else if (autoPrograms.containsKey(programName)) {
                selectedAuto = programName;
                Log.info("NarwhalDashboard", "Selected auto program: \"" + selectedAuto + "\"");
            }
            else {
                Log.recoverable("NarwhalDashboard", "Auto program \"" + programName + "\" does not exist.");
            }
            
        }
        else if (parts[0].equals("numData")) {
            String key = parts[1];
            double numericalData = Double.parseDouble(parts[2]);
        }
        else if (parts[0].equals("button")) {
            String key = parts[1];
            boolean down = parts[2].equals("down");

            if (buttons.containsKey(key)) {
                buttons.get(key).action(down);
            }
            else {
                Log.recoverable("NarwhalDashboard", "Button \"" + parts[1] + "\" was never added.");
            }
        }
        else {
            Log.info("NarwhalDashboard", "Message recieved: " + message);
        }
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        ex.printStackTrace();
    }

    @Override
    public void onStart() {

    }
}
