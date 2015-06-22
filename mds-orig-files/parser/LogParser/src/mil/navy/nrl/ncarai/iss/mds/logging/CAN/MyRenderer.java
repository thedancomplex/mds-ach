package mil.navy.nrl.ncarai.iss.mds.logging.CAN;
import java.awt.*;
import javax.swing.*;
import javax.swing.table.DefaultTableCellRenderer;

public class MyRenderer extends DefaultTableCellRenderer 
{

	private static final long serialVersionUID = 1L;

public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int col)  
   {
	Component c = super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, col);
	if(value != null)
	{
		if (value.toString().equals("Warning") || (value.toString().length() >= 5 && value.toString().substring(0,5).equals("Error")))
			c.setForeground(Color.red);
		else if(value.toString().equals("Yes"))
			c.setForeground(Color.blue);
		else if(value.toString().equals("No"))
			c.setForeground(Color.green.darker()); 
		else if(value.toString().equals("FillBuffer"))
			c.setForeground(Color.orange.darker());
	    else if(value.toString().equals("..."))
	    	c.setBackground(Color.black);
	}
     return c; 
   }

}