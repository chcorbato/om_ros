package org.aslab.om.ros;

import org.aslab.om.ecl.action.Action;
import org.aslab.om.ecl.action.ActionFeedback;
import org.aslab.om.ecl.action.ActionResult;
import OMRosDrivers_py.MetaAction;
import geometry_msgs.PoseWithCovarianceStamped;


/**
 * @author chcorbato
 * 
 * object to encapsulate information about actions at the Ros framework level
 * those actions that the OMRosDrivers_py/meta_actuator.py can execute by making use of the rospy library
 */
public class RosAction extends Action {

	/**
	 * the Ros message for this action
	 */
	private Object command;
		
	/** 
	 * <!-- begin-UML-doc -->
	 * number of ever issued action_list (this way a new number is assigned to everyone upon creation)
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	private static short num_of_actions = 0;
	
	@Override
	protected short addAction(){
		return num_of_actions++;
	}
	
	public RosAction(MetaAction a){
		super();
		command = a;
		((MetaAction) command).setActionID(id);
	}
	
	public RosAction(PoseWithCovarianceStamped a){
		command = a;
	}
			
	/**
	 * method to execute a Ros action. It:
	 * - updates the information about the state of this action and other associated
	 * 	@see org.aslab.om.ecl.action.Action#execute()
	 * - retrieves the command to send to the actuators (in this case a Ros message)
	 * @return the Ros message for this action, it is of the type MetaAction defined in package
	 * OMRosDrivers_py
	 */
	public Object executeRosAction(){
		execute();
		return command;
	}
	
	/** 
	 * @see org.aslab.om.ecl.action.Action#processResult(org.aslab.om.ecl.action.ActionFeedback)
	 */
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
