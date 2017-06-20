package org.aslab.om.ros;

import org.aslab.om.ecl.action.ActionFeedback;
import org.aslab.om.ecl.action.ActionResult;
import OMRosDrivers_py.MetaActionResult;


public class RosFeedback extends ActionFeedback {

	public RosFeedback(MetaActionResult message){
		if ( message.getResult().equals("SUCCEEDED") )
			result = ActionResult.SUCCEEDED;
		else if (message.getResult().equals("FAILED") )
			result = ActionResult.FAILED;
		else
			return; // manage exception when the message is corrupted??
		
		actionID = message.getActionID();
		//log = message.log; TODO add log to the action feedback
	}
}
