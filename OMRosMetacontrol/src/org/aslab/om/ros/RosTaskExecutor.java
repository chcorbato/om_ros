/**
 * 
 */
package org.aslab.om.ros;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import org.aslab.om.ecl.action.ActionFeedback;
import org.aslab.om.ecl.action.ActionResult;
import org.aslab.om.ecl.action.ActionStatus;
import org.aslab.om.metacontrol.action.ReconfigurationCommand;
import org.ros.message.*;
import org.ros.node.ConnectedNode;

import OMRosDrivers_py.MetaAction;
import OMRosDrivers_py.MetaActionResult;
import geometry_msgs.PoseWithCovariance;
import geometry_msgs.PoseWithCovarianceStamped;
import std_msgs.Header;



/** 
 * <!-- begin-UML-doc -->
 * A RosTaskExecutor manages the Ros actions required for one of the ReconfigurationCommands commanded by the COmponentsECL
 * <!-- end-UML-doc -->
 * @author chcorbato
 * @!generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
 */
public class RosTaskExecutor {
	
	/**
	 * the reconfiguration command this executor  has to accomplish
	 */
	private ReconfigurationCommand task;
	
	/** 
	 * <!-- begin-UML-doc -->
	 * the list of Ros actions to realise the command assigned to this executor
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	private Map<Short,RosAction> ros_actions = new HashMap<Short, RosAction>();

	/**
	 * access to the manager of all the action_list
	 */
	private ReconfigurationManager manager;
	
	private MessageFactory msg_factory;

	
	/** 
	 * <!-- begin-UML-doc -->
	 * constructor of the executor. It also realizes and initial computation of Ros actions for the incoming
	 * 
	 * <!-- end-UML-doc -->
	 * @param t one of the reconfiguration commands from the ComponentsECL
	 * @param m a reference to the ReconfigurationManager that coordinates the different executors,
	 * each one addressing on of the commands in the Reconfiguration
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	public RosTaskExecutor(ReconfigurationCommand t, ReconfigurationManager m, ConnectedNode node) {
		msg_factory = node.getTopicMessageFactory();
		manager = m;		
		task = t;		
		computeRosActions(task);
		processActions();	// immediately execute pending actions (actions that can already be executed)
	}
	
	public short getTaskID(){
		return task.getId();
	}
	
	
	
	/**
	 * converts a ComponentAction into Ros commands (MetaAction instances) managed with instances
	 * of RosAction
	 * @param task
	 */
	private void computeRosActions(ReconfigurationCommand task) {				
		switch (task.type) {		
			case KILL:				
				MetaAction action_msg = msg_factory.newFromType("OMRosDrivers_py/MetaAction");
				action_msg.setHeader( (Header) msg_factory.newFromType("Header") );
				action_msg.setActionID( (short) task.getId() );
				action_msg.setActionName("KILL");
				action_msg.setNodeName(task.target);
				newAction(action_msg);
				break;
				
			case LAUNCH:
				action_msg = msg_factory.newFromType("OMRosDrivers_py/MetaAction");
				action_msg.setActionName("LAUNCH");
				action_msg.setPckg( (String) task.arguments.get("rospackage") );
				action_msg.setLaunchfile( (String) task.arguments.get("launchfile") );
				RosAction action = newAction(action_msg);
				
				// TODO: this is ad-hoc for Higgs amcl supposing the other action to publish the state from where to restart
				// is sent upon processing of feedback for kill
				if( action_msg.getLaunchfile().contains("amcl") ){
					PoseWithCovarianceStamped pose_msg = msg_factory.newFromType("geometry_msgs/PoseWithCovarianceStamped");
					pose_msg.setPose( (PoseWithCovariance) manager.internal_states.get("/amcl_pose") );
					RosAction action2 = newAction(pose_msg);
					action2.addDependency(action);	
				}
				
				break;
				
			case RECONFIGURE: // TODO : for the moment we kill and relaunch for every reconfiguration
				/**
				 * ad-hoc method that supposes that a launchfile is given with a configuration that provides this reconfig
				 * the launchfile is given by the argument "launchfile" of the ReconfigurationCommand
				 */
				if (task.arguments.get("launchfile") != null) {
					action_msg = msg_factory.newFromType("OMRosDrivers_py/MetaAction");
					action_msg.setActionName("KILL");
					action_msg.setNodeName(task.target);
					RosAction action1 = newAction(action_msg);
			
					action_msg = msg_factory.newFromType("OMRosDrivers_py/MetaAction");
					action_msg.setActionName("LAUNCH");
					action_msg.setPckg( (String) task.arguments.get("rospackage") );
					action_msg.setLaunchfile( (String) task.arguments.get("launchfile") );
					RosAction action2 = newAction(action_msg);
					action2.addDependency(action1);
				}
				break;
				
			case RESTART:
				action_msg = msg_factory.newFromType("OMRosDrivers_py/MetaAction");
				action_msg.setHeader( (Header) msg_factory.newFromType("Header") );
				action_msg.setActionID( (short) task.getId() );
				action_msg.setActionName("KILL");
				action_msg.setNodeName(task.target);
				RosAction action1 = newAction(action_msg);
				
				action_msg = msg_factory.newFromType("OMRosDrivers_py/MetaAction");
				action_msg.setActionName("LAUNCH");
				action_msg.setPckg( (String) task.arguments.get("rospackage") );
				action_msg.setLaunchfile( (String) task.arguments.get("launchfile") );
				RosAction action2 = newAction(action_msg);
				action2.addDependency(action1);
				
				// TODO: this is ad-hoc for Higgs amcl supposing the other action to publish the state from where to restart
				// is sent upon processing of feedback for kill
				if( action_msg.getLaunchfile().contains("amcl") ){
					PoseWithCovarianceStamped pose_msg = msg_factory.newFromType("geometry_msgs/PoseWithCovarianceStamped");
					pose_msg.setPose( (PoseWithCovariance) manager.internal_states.get("/amcl_pose") );
					RosAction action3 = newAction(pose_msg);
					action3.addDependency(action2);	
				}
				
				break;
		}
	}

	
	/**
	 * @param message the Ros message with the result of a Ros command
	 * @return the feedback information about the task of the executor if it completed (either as a
	 * success or a failure) with the Ros command. If it did not complete (more actions with
	 * associated Ros commands still pending, then returns <code>null</code>)
	 */
	public ActionFeedback processFeedback(MetaActionResult message) {
		
		ActionFeedback internal_feedback = new RosFeedback(message);
		
		RosAction action = ros_actions.get(new Short(message.getActionID()));
		
		ActionResult result = action.processResult(internal_feedback);
		
		/**
		 * this encodes the model of Ros feedback
		 */
		if (result == ActionResult.FAILED){	// if any action fails, the executor task fails
			// more processing  of the failure at the executor level could be done here if possible, like re-plan
			// remove actions for this failed task from the reconfiguration manager?
			task.status = ActionStatus.FAILURE;
			return new ActionFeedback(getTaskID(), ActionResult.FAILED, internal_feedback.log);	// for the moment we just return a feedback concerning task failure
		}
		
		else if (result == ActionResult.SUCCEEDED){  // if action succeeded execute next actions that were waiting because of this one
			// execute all pending actions
			if ( processActions() ){
				task.status = ActionStatus.SUCCESS;
				return new ActionFeedback(getTaskID(), ActionResult.SUCCEEDED, internal_feedback.log);	// for the moment we just return a feedback concerning task failure

			}
			else	// if the task is not finished no feedback is returned
				return null;
		}

		// TODO exception instead of if else?
		else
			return null;
		
	}
	
	
	/**
	 * executes any pending actions required for the completion of the task
	 * @return true if the task is finished, false otherwise
	 */
	private boolean processActions() {
		boolean no_actions_remaining = true;
		
		for (Iterator<RosAction> iter = ros_actions.values().iterator(); iter.hasNext();){
			RosAction action = iter.next();
			if ( action.isPending() ){
				no_actions_remaining = false;
				manager.executeAction(action);
			}
		}
		
		
		return no_actions_remaining;
	}

	private RosAction newAction(MetaAction msg){
		RosAction action = manager.newAction(this, msg);
		ros_actions.put(action.getId(), action);
		return action;
	}
	
	private RosAction newAction(PoseWithCovarianceStamped msg){
		RosAction action = manager.newAction(this, msg);
		ros_actions.put(action.getId(), action);
		return action;
	}


}