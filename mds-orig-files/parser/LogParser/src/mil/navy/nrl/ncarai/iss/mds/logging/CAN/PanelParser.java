package mil.navy.nrl.ncarai.iss.mds.logging.CAN;
import java.util.*;
import java.io.*;
import javax.swing.*;
import javax.swing.table.*;
import java.awt.*;
import java.awt.event.*;

public class PanelParser extends JPanel implements ActionListener, ItemListener
{

	private static final long serialVersionUID = 1L;
	public File f;
	public JTable table; 
	public XTableColumnModel colModel; 
	public JCheckBox[] boxes; 
	public JPanel sidePanel, bottomPanel; 
	public Map<String, Integer> frequency;
	public ArrayList<Integer> colsRemoved;
	public Object[][] data;
	public JScrollPane scrollPane;
	public String[] cols = {"Line #", "Time", "Receiver", "Response","Argument", "Sender", "Data", "Data", "Data", "Data", "...", "Line #", "Time", "Receiver", "Command","Argument", "Data", "Data", "Data", "Data", "Data", "Data"};

	public PanelParser() throws FileNotFoundException, ClassNotFoundException
	{
		//input file name

		f = new File("");

		while(f.exists() == false)
		{

			final JFileChooser fc = new JFileChooser();
			int returnVal = fc.showOpenDialog(this);

        	if (returnVal == JFileChooser.APPROVE_OPTION) {
            		f = fc.getSelectedFile();
            		//log.append("Opening: " + file.getName() + "." + newline);
        	} 
		}
		
		//create side panel

		sidePanel = new JPanel(); 
		sidePanel.setLayout(new GridLayout(25,1));

		JLabel incoming = new JLabel("Incoming:");
		JLabel outgoing = new JLabel("Outgoing:");
		JLabel divider = new JLabel("Divider:");

		sidePanel.add(incoming);

		boxes = new JCheckBox[22];

		for(int x = 0; x < boxes.length; x++)
		{
			if(x == 10)
				sidePanel.add(divider);
			if(x == 11)
				sidePanel.add(outgoing);
			boxes[x] = new JCheckBox(cols[x], true); 
			boxes[x].setActionCommand(x + ""); //real column number
			boxes[x].addItemListener(this);
			sidePanel.add(boxes[x]);
		}

		//create bottom panel

		bottomPanel = new JPanel();
		bottomPanel.setLayout(new FlowLayout());

		JPanel instructions = new JPanel(); 
		instructions.setLayout(new GridLayout(4, 1));
		JLabel intro1 = new JLabel("To display a column, check corresponding box on the right.");
		JLabel intro2 = new JLabel("To sort a column, click the column header at the top.");
		JLabel intro3 = new JLabel("To learn about an analyzed piece of data, place your cursor over it.");
		JLabel intro4 = new JLabel("To understand a bottom row button's function, place your cursor over the button.");
		instructions.add(intro1);
		instructions.add(intro2);
		instructions.add(intro3);
		instructions.add(intro4);

		JButton FTPane = new JButton("Frequency Table");
		FTPane.setActionCommand("FT");
		FTPane.addActionListener(this);
		FTPane.setToolTipText("This button displays a sorted frequency table of the parsed lines in the output log using GUI.");

		JButton BTPane = new JButton("FillBuffer & IsTrajectoryRunning");
		BTPane.setActionCommand("BT");
		BTPane.addActionListener(this);
		BTPane.setToolTipText("This button displays the number of instances the FillBuffer response is followed by the IsTrajectoryRunning argument with an answer of \"No\", along with the lines on which this occurs, using GUI.");
		
		JButton normal = new JButton("Default Display");
		normal.setActionCommand("default");
		normal.addActionListener(this);
		normal.setToolTipText("This button displays the default, unsorted table.");

		bottomPanel.add(instructions);
		bottomPanel.add(FTPane);
		bottomPanel.add(BTPane);
		bottomPanel.add(normal);

		//create main panel

		setLayout(new BorderLayout());

		Scanner infile = new Scanner(f);

		frequency = new HashMap<String, Integer>(); //Count of the times the same message is sent multiple times during the same time stamp.

		ArrayList<Object[]> array = new ArrayList<Object[]>(); 

		int current = 0;
		while(infile.hasNextLine())
		{
			String s = infile.nextLine();

			Object[] a = null;
			if(s.matches("(?i).*in.*"))
			{	
				a = CommonParser.parseLine(s, 22, current+1, "in");
				CommonParser.fillTable(frequency, a, "IN");
			}
			else
			{
				a = CommonParser.parseLine(s, 22, current+1, "out");
				CommonParser.fillTable(frequency, a, "OUT");
			}

			if (a != null) array.add(a);
			current++;
		}

		table = originalTable(array.toArray(new Object[0][0]), cols);
		scrollPane = new JScrollPane(table);

		add(scrollPane, BorderLayout.CENTER);
		add(sidePanel, BorderLayout.EAST);
		add(bottomPanel, BorderLayout.SOUTH);
	}
	
	public JTable originalTable(Object[][] data, String[] cols)
	{
		JTable table = new JTable(data, cols)
				{
				private static final long serialVersionUID = 1L;

				public TableCellRenderer getCellRenderer(int row, int column)
				{
					return new MyRenderer(); 
				}

				public String getToolTipText(MouseEvent e) 
				{
					String tip = null;
					java.awt.Point p = e.getPoint();
					int row = rowAtPoint(p);
					int col = columnAtPoint(p);
					int realCol = convertColumnIndexToModel(col);
					TableModel model = getModel();
					String response = (String)model.getValueAt(row, 3);
					String argument = (String)model.getValueAt(row, 4);
					String argument2 = (String)model.getValueAt(row, 15);
					if(response != null)
					{
						if(response.equals("FillBuffer") && realCol == 8)
							tip = "Number of points requested.";
						else if(response.equals("FillBuffer") && realCol == 9)
							tip = "Number of points returned.";
					}
					if((argument != null && ((argument.equals("State_Desired_Position") || argument.equals("State_Actual_Position")) && realCol == 9)) || (argument2 != null && ((argument2.equals("State_Desired_Position") || argument2.equals("State_Actual_Position")) && realCol == 21)))
						tip = "Float describing position.";
					return tip;
				}

			};
			
			TableRowSorter<TableModel> sorter = new TableRowSorter<TableModel>(table.getModel());

			Comparator<Integer> comparer = new Comparator<Integer>()
			{
				public int compare(Integer i, Integer ii)
				{
					return i-ii; 
				}
			};

			sorter.setComparator(0, comparer); //Line #
			sorter.setComparator(11, comparer); //Line #

			colModel = new XTableColumnModel(); 
			table.setColumnModel(colModel);
			table.createDefaultColumnsFromModel();

			for(int x = 0; x < colModel.getColumnCount(); x++)
			{
				TableColumn col = colModel.getColumn(x);
				if(x == 1 || x == 12)
					col.setPreferredWidth(95);
				else if(x == 3 || x == 14)
					col.setPreferredWidth(80);
				else if(x == 4 || x == 15)
					col.setPreferredWidth(205);
				else if(x == 5 || x == 13)
					col.setPreferredWidth(115);
				else if(x == 9)
					col.setPreferredWidth(120);
				else if(x == 10)
					col.setPreferredWidth(12);
				else 
					col.setPreferredWidth(44);
			}

			table.setRowSorter(sorter);
			table.setPreferredScrollableViewportSize(new Dimension(500, 70));
			table.setFillsViewportHeight(true);
			
			return table; 
	}
	
	public void actionPerformed(ActionEvent e) 
	{
		String s = null;
		JTable t = null;
		if(e.getActionCommand().equals("FT"))
		{
			t = CommonParser.getTable(frequency);
			CommonParser.displayWithPane(t);
		}
		else if(e.getActionCommand().equals("BT"))
		{
			s = CommonParser.getBufferAndTrajectory();
			JTextArea ta = new JTextArea(s); 
			CommonParser.displayWithPane(ta);
		}	
		else if(e.getActionCommand().equals("default"))
		{
			JTable orig = originalTable(data, cols);
			scrollPane.setViewportView(orig);
		}

	}

	public void itemStateChanged(ItemEvent e) 
	{
		JCheckBox source = (JCheckBox)e.getItemSelectable();
		int c = Integer.parseInt(source.getActionCommand()); 
		TableColumn column = colModel.getColumnByModelIndex(c);
		colModel.setColumnVisible(column, true);
		if (e.getStateChange() == ItemEvent.DESELECTED)
			colModel.setColumnVisible(column, false);		
	}
}
