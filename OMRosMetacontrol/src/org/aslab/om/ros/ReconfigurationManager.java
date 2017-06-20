package org.aslab.om.ros;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;


import org.aslab.om.ecl.action.ActionFeedback;
import org.aslab.om.ecl.action.ActionResult;
import org.aslab.om.metacontrol.action.ReconfigurationCommand;
import OMRosDrivers_py.MetaAction;
import OMRosDrivers_py.MetaActionResult;
import geometry_msgs.*;

import org.ros.node.topic.Publisher;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;


/**
 * @author chcorbato
 * This class encapsulates all the methods and objects that are needed in the action part of OMRosAPI 
 */
public class ReconfigurationManager {
	
	/**
	 * the reconfiguration of components that this manager handles
	 */
	private Set<ReconfigurationCommand> reconfiguration;
		
	/** 
	 * the Ros publisher to command the Ros actions computed by the executors
	 */
	private Publisher<MetaAction> action_publisher;
	
	private Publisher<PoseWithCovarianceStamped> pose_publisher;	// ad-hoc for Higgs: publisher for amcl internalstate

	private Publisher<MetaActionResult> feedback_publisher;
	
	
	public Map<String, Object> internal_states = new HashMap<String, Object>();
	
	private ConnectedNode node;
	
	/**
	 * the set of executors to accomplish the reconfiguration, each one responsible for a piece
	 * (ReconfigurationCommand) of it
	 */
	private ArrayList<RosTaskExecutor> executors = new ArrayList<RosTaskExecutor>();
	
	
	/**
	 * to keep record of each reconfiguration command (with its id) and its executor
	 */
	private Map<Short, RosTaskExecutor> actionsTable = new HashMap<Short, RosTaskExecutor>();

	
	public ReconfigurationManager(ConnectedNode n, Map<String, Object> info) {
		node = n;
		action_publisher = node.newPublisher("/meta_action",
		"OMRosDrivers_py/MetaAction");
		
		feedback_publisher = node.newPublisher("/meta_action_result", "OMRosDrivers_py/MetaActionResult");

		// TODO this is completely Ad-hoc for Higgs amcl
		pose_publisher = node.newPublisher("/initialpose", "geometry_msgs/PoseWithCovarianceStamped");
		
		internal_states = info;
	}
	
	/**
	 * method to set a new reconfiguration task (goal?) for the manager. It creates an executor for each command
	 * @param a the set of commands for the reconfiguration
	 */
	public void setReconfigurationTask(Set<ReconfigurationCommand> a){
		reconfiguration = a;		
		// create an executor for each command
		for(Iterator<ReconfigurationCommand> i = reconfiguration.iterator(); i.hasNext();){
			RosTaskExecutor exec = new RosTaskExecutor(i.next(), this, node);
			executors.add(exec);
		}
	}
	
	
	public RosAction newAction(RosTaskExecutor executor, MetaAction msg){
		RosAction action = new RosAction(msg);
		actionsTable.put(action.getId(), executor);
		return action;
	}

	public RosAction newAction(RosTaskExecutor executor, PoseWithCovarianceStamped msg){
		RosAction action = new RosAction(msg);
		actionsTable.put(action.getId(), executor);
		return action;
	}	
	
	/**
	 * method to actually command an action (used by the executors)
	 * @param command
	 */
	public void executeAction(RosAction action){
		Object command = action.executeRosAction();
		
		if( command instanceof MetaAction )
			action_publisher.publish((MetaAction) command);
		
		else if( command instanceof PoseWithCovarianceStamped ){	// Ad-hoc for Higgs - amcl
			pose_publisher.publish((PoseWithCovarianceStamped) command);
			// this type of action is realised here, not sent to the metaactuator, so the feedback 
			// is generated here too
			MetaActionResult result = feedback_publisher.newMessage();
			result.setActionID(action.getId());
			result.setResult("SUCCEEDED");
			feedback_publisher.publish(result);
		}
		
	}
	
	
	/**
	 * Redirects the feedback to be processed by the corresponding executor
	 * where the processing of the feedback may cause new actions to be executed
	 * @param message
	 * @return the feedback produced by the ros msg. It can be null if none
	 */
	public synchronized ActionFeedback processFeedback(MetaActionResult message) {
		if( message == null )
			throw new NullPointerException("no message to process");
		
		// get the executor
		RosTaskExecutor executor = actionsTable.get(new Short(message.getActionID()));
		
		ActionFeedback feedback = executor.processFeedback(message);
		
		if (feedback != null){
			if ( feedback.result == ActionResult.FAILED )
				// if there was possible re-planning for the action, it should be called here
				// for the moment if there is a feedback it is scaled directly up to the commander
				// (ComponentsECL) and the executor is eliminated
				executors.remove(executor); // TODO remove
		}
		return feedback;
	}
	
	
	/**
	 * auxiliar method used by RosTaskExecutor to send a feedback for actions that are not commanded to
	 * meta_actuator_node
	 */
	public void simulateFeedback(){
		feedback_publisher.publish(null);
	}


}
