package mil.navy.nrl.ncarai.iss.mds.logging.CAN;
//Extended by Parser2 and PanelParser

import java.awt.*;
import java.io.*;
import java.util.*;
import java.lang.Float;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

import javax.swing.*;

public class CommonParser
{
	public static boolean fillBuffer = false;
	public static boolean isTrajectoryNotRunning = false;
	public static int count = 0;
	public static int other = 0;
	public static int fbcount, itnrcount;
	public static ArrayList<Integer> lines = new ArrayList<Integer>();
	
	private static final long serialVersionUID = 1L;

                public static Hashtable<String, String> response = createTable("ResponseHex.txt", 10);
                public static Hashtable<String, String> command = createTable("CommandHex.txt", 20);
                public static Hashtable<String, String> argument = createTable("ArgumentHex.txt", 46);
                public static Hashtable<String, String> node = createTable("NodeHex.txt", 43);

	
	public static void update()
	{
		if(fillBuffer == true && isTrajectoryNotRunning == true && (itnrcount-fbcount == 1))
		{
			fillBuffer = false; 
			isTrajectoryNotRunning = false;
			count++;
			lines.add(itnrcount);
		}
		else
			other++;
	}
	
	public static Hashtable<String, String> createTable(String filename, int numitems)
	{
                Hashtable<String, String> h = new Hashtable<String, String>(numitems);

		try {
			String path = "src/mil/navy/nrl/ncarai/iss/mds/logging/CAN";
			
		Scanner infile = new Scanner(new File(path + "/" + filename));
		
		while(infile.hasNextLine())
		{
			String s = infile.nextLine();
			String[] array = s.split(" ");
			h.put(array[0], array[1]);
		}
		}
		catch (Exception e) {
			System.err.println(e);
		}
		finally{
			return h;
		}
	}
	
	public static void fillTable(Map<String, Integer> h, Object[] a, String type)
	{
		String str = ""; 
		if(type.equals("IN"))
		{
			str = str + "IN\t";
			for(int x = 1; x < 10; x++)
				str = str + a[x] + "\t"; 
		}
		else
		{
			str = str + "OUT\t";
			for(int x = 12; x < 22; x++)
				str = str + a[x] + "\t"; 
		}
			
		if(h.get(str) == null)
			h.put(str, 1);
		else
			h.put(str, h.get(str)+1);
	}
	
	public static JTable getTable(Map<String, Integer> h)
	{		
		JTable t = new JTable(h.size(), 12);
	
        ArrayList<Map.Entry<String, Integer>> list = new ArrayList<Map.Entry<String, Integer>>(h.entrySet());
        
        Collections.sort(list, new Comparator<Map.Entry<String, Integer>>() 
        {
            public int compare(Map.Entry<String, Integer> e1, Map.Entry<String, Integer> e2) 
            {
                Integer i1 = (Integer) e1.getValue();
                Integer i2 = (Integer) e2.getValue();
                return i2.compareTo(i1);
            }
        });
        
        int currentRow = 0;
        
        for(Map.Entry<String, Integer> e : list)
        {
        	String[] array = e.getKey().split("\t");
        	for(int x = 0; x < array.length; x++)
        		t.setValueAt(array[x], currentRow, x);
        	t.setValueAt(e.getValue(), currentRow, 11);
        	currentRow++; 
        }
        
        return t; 
	}
	
	public static String getBufferAndTrajectory()
	{
		String s = "Instances where FillBuffer response is followed by isTrajectoryRunning argument with answer \"No\": " + count + "\nLines: ";
		Iterator<Integer> i = lines.iterator();
		for(int x = 0; x < lines.size(); x++)
			s = s + i.next() + " ";
		return s + "\n";
	}

	public static void displayWithPane(Component c)
	{
		JScrollPane scrollPane = new JScrollPane(c);		
		scrollPane.setPreferredSize(new Dimension(800, 200));
		JOptionPane.showMessageDialog(null, scrollPane);
	}
	
	public static Object[] parseLine(String s, int length, int current, String type)
	{
		//IN -- Response, Argument, Receiver x 2, Data x 4
		//OUT -- Command, Argument, Data x 6
	
		System.out.print(".");
	
		Object[] output = new Object[length];
		
		Object[] array = s.split("\t"); //Time DEBUG : In or Out, Sender or Receiver, Data 
		String time = (String)((String)array[0]).split(" ")[0]; // Time
		String[] hexArray = ((String) array[3]).split(" "); //Hex Codes
		
		output[10] = "...";
		
		if(type.equals("in"))
		{
			fillBuffer = false; 
			
			output[0] = new Integer(current); //Line #
			output[1] = time; //Time
			
			if(node.get(array[2]) != null) //Receiver
				output[2] = node.get(array[2]);
			else
				output[2] = array[2]; 
			
			String receiver = hexArray[2] + hexArray[3]; //Sender
			if(node.get(receiver) != null)
				output[5] = node.get(receiver);
			else
				output[5] = receiver; 
	
			if(response.get(hexArray[0]) != null) //Response
				output[3] = response.get(hexArray[0]);
			else
				output[3] = hexArray[0];
			
			for(int x = 4; x < 8; x++) //Data x 4
				output[x+2] = hexArray[x];
			
			if(hexArray[0].equals("08")) //FillBuffer
			{
				fillBuffer = true; 
				fbcount = other+count; 
				short pointsRequested = Short.valueOf(hexArray[1], 16).shortValue();
				String reversed = hexArray[7] + hexArray[6];
				short pointsInBuffer = Short.valueOf(reversed, 16).shortValue();
				output[4] = "  ";
				for(int x = 6; x < 8; x++) //Data x 2
			    	output[x] = hexArray[x]; 
				output[8] = pointsRequested;
				output[9] = pointsInBuffer;
				update(); 
				return output;
			}
			
			
			if(argument.get(hexArray[1]) != null) //Argument
			{
				output[4] = argument.get(hexArray[1]);
				if(hexArray[1].equals("21")) //Is Trajectory Running? 
				{
					for(int x = 6; x < 9; x++)
						output[x] = "  ";
					if(hexArray[7].equals("00")) //Trajectory is not running
						output[9] = "No";
					else //Trajectory is running
						output[9] = "Yes";
					update(); 
					return output;
				}
				else if(hexArray[1].equals("04") || hexArray[1].equals("05")) //State Desired Position and State Actual Position
				{

                                        for(int x = 6; x < 9; x++)
                                                output[x] = "  ";

					output[9] = parseOutEncoderCount(hexArray);
					update(); 
					return output; 
				}		
			}
			else
				output[4] = hexArray[1]; //Hex Argument

		}
		else  // if (type.equals("out"))
		{		
			isTrajectoryNotRunning = false; 
			output[11] = new Integer(current);
			output[12] = time;
			
			if(node.get(array[2]) != null) //Receiver
				output[13] = node.get(array[2]);
			else
				output[13] = array[2]; 
			
			if(command.get(hexArray[0]) != null) //Command
				output[14] = command.get(hexArray[0]);
			else
				output[14] = hexArray[0];
		
                        for(int x = 2; x < hexArray.length; x++) //Data x 6
                        	output[x+14] = hexArray[x];
	
			if(argument.get(hexArray[1]) != null) //Argument
			{
				output[15] = argument.get(hexArray[1]);
				
				if(hexArray[1].equals("30")) // NextBufferPoint
				{
					int x = 2;
                    for(; x < hexArray.length; x++)
                            output[x+14] = "  ";

					output[(x - 1) + 14] = parseOutEncoderCount(hexArray);
					update();
					return output;
				}		
			}
			else
				output[15] = hexArray[1];
			
		}
		update(); 
		return output;
	}

	public static float parseOutEncoderCount(String[] hexArray) {
                                       	int len = 4;
                                        byte[] arr = new byte[len];

                                  
                                        for(int x = 4, cnt = 0; cnt < len; x++, cnt++) {
                                                arr[cnt] = (byte)Integer.valueOf(hexArray[x], 16).intValue();
                                        }

                                        ByteBuffer bbuffer = ByteBuffer.wrap(arr);
                                        FloatBuffer result = bbuffer.asFloatBuffer();
                                        return result.get(0);
	}
}
