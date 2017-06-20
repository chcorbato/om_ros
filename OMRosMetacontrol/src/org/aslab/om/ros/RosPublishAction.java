package org.aslab.om.ros;

import org.aslab.om.ecl.action.Action;
import org.aslab.om.ecl.action.ActionFeedback;
import org.aslab.om.ecl.action.ActionResult;
import geometry_msgs.PoseWithCovarianceStamped;

public class RosPublishAction extends Action {
	
	PoseWithCovarianceStamped pose_msg;
	
	public RosPublishAction(PoseWithCovarianceStamped a){
		super();
		pose_msg = a;
	}

	@Override
	protected short addAction() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public ActionResult processResult(ActionFeedback feedback) {
		
		if ( feedback.result == ActionResult.SUCCEEDED )
			setSuccess();	// this method also updates the dependencies between actions
		else if (feedback.result == ActionResult.FAILED)
			setFailure();	// this method also updates the dependencies between actions
		
		return feedback.result;
	}

	@Override
	protected boolean timeOut() {
		// TODO Auto-generated method stub
		return false;
	}
			
	

}
