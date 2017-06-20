package org.aslab.om.ros;

import java.io.UnsupportedEncodingException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.aslab.om.components_functions_metamodel.components.static_knowledge.Directionality;
import org.aslab.om.ecl.action.ActionFeedback;
import org.aslab.om.metacontrol.PlantAPI;
import org.aslab.om.metacontrol.action.ReconfigurationCommand;
import org.aslab.om.metacontrol.knowledge.components.ComponentStatus;
import org.aslab.om.metacontrol.knowledge.components.OMcomponentsKB;
import org.aslab.om.metacontrol.perception.components.ComponentSingularity;
import org.aslab.om.metacontrol.perception.components.PortSingularity;


import org.ros.message.*;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import OMRosDrivers_py.ROSParameter;
import OMRosDrivers_py.ROSTopic;
import OMRosDrivers_py.RosSingularity;
import OMRosDrivers_py.ComponentReferent;
import OMRosDrivers_py.ComponentReferents;
import OMRosDrivers_py.MetaActionResult;
import OMRosDrivers_py.OMTypes;
import OMRosDrivers_py.PerceptionFlow;
import geometry_msgs.*;


/** 
 * <!-- begin-UML-doc -->
 * <p>provides a concrete implementation for the OMJava.ecl.metacontrol.PlantAPI for the ROS platform</p>
 * <p>Manages the interaction with the actual monitoring and reconfiguration infrastructure for the
 *  metacontroller, which consists of two different Ros nodes</p>
 *  <p>action issues are delegated to the manager @see ReconfigurationManager</p>
 * <!-- end-UML-doc -->
 * @author chcorbato
 */
public class OMRosAPI extends PlantAPI {

	/** 
	 * <!-- begin-UML-doc -->
	 * the Ros node to receive messages for sensing components and send messages to act upon them
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	private ConnectedNode node;

	/** 
	 * <!-- begin-UML-doc -->
	 * Ros publisher to configure the referents for the Ros sensor (OMRosDrivers_py/meta_sensor.py)
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	private Publisher<ComponentReferents> referents_publisher;

	/** 
	 * <!-- begin-UML-doc -->
	 * the buffer with the sensory information about components
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	private Set<ComponentSingularity> singularityBuffer = new HashSet<ComponentSingularity>();
	
	
	private Map<String, Object> internal_states = new HashMap<String, Object>();
	
	
	/**
	 * the buffer with feedback information about commanded actions
	 */
	private Map<Short,ActionFeedback> feedbackBuffer = new HashMap<Short,ActionFeedback>();
	
	
	/** 
	 * <!-- begin-UML-doc -->
	 * contains the current manager of Ros action_list for achieving the reconfiguration of the components structure of the plant.
	 * It is an ordered set (list)
	 * this attribute should go in an hypothetical action or control object if that part is isolated from the rest of the ECL
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	private ReconfigurationManager manager;

	/** 
	 * <!-- begin-UML-doc -->
	 * access to the knowledge database
	 * <!-- end-UML-doc -->
	 * @!generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	@SuppressWarnings("unused")
	private OMcomponentsKB kb;
	

	/** 
	 * <!-- begin-UML-doc -->
	 * Constructor of the org.aslab.om.metacontrol node. Initializes the subscribers and topis for the I/O with
	 * the Ros system, and also creates the ReconfigurationManager, which needs access to the publisher of actions
	 * <!-- end-UML-doc -->
	 * @param n the ROS node
	 * @param k
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	public OMRosAPI(Node n, OMcomponentsKB k) {
		// begin-user-code
		node = (ConnectedNode) n;
		kb = k;
		
		// initialize internal_states (TODO: it is Ad-hoc for Higgs)
		internal_states.put("/amcl_pose", null);
		
		// -- sensory stuff -------------------------------------

		// referents publisher
		referents_publisher = node.newPublisher("/meta_referents",
				"OMRosDrivers_py/ComponentReferents");
		
		// subscription to sensory flow: it updates the sensory buffer each time a msg is received
		Subscriber<OMRosDrivers_py.PerceptionFlow> subs_perception= node.newSubscriber("/meta_singularities","OMRosDrivers_py/PerceptionFlow");
		subs_perception.addMessageListener(new MessageListener<PerceptionFlow>() {

			@Override
			public void onNewMessage(PerceptionFlow message) {
				// node.getLog().info("New data");
				singularityBuffer.clear();
				readSensing(message);
			}

		});
		
		// subscription to amcl pose -- TODO: this is Ad-hoc for Higgs
		Subscriber<PoseWithCovarianceStamped> subs_pose= node.newSubscriber("/amcl_pose",
				"geometry_msgs/PoseWithCovarianceStamped");
		subs_pose.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {

			@Override
			public void onNewMessage(PoseWithCovarianceStamped message) {
				if(message.getPose() != null){
					internal_states.put("/amcl_pose", message.getPose());
				}
			}
		});
		

		// -- action stuff -------------------------------------	    
		manager =  new ReconfigurationManager(node, internal_states);

		// action publisher
		
		

		// subscription to action result
		Subscriber<OMRosDrivers_py.MetaActionResult> subs_maction_result = node.newSubscriber("/meta_action_result",
				"OMRosDrivers_py/MetaActionResult");
		subs_maction_result.addMessageListener(new MessageListener<MetaActionResult>() {
				
			@Override
			public void onNewMessage(MetaActionResult message) {				
				processFeedback(message);
			}
		});
		// end-user-code
	}

	/** 
	 * <!-- begin-UML-doc -->
	 * retrieves the sensing incoming in the PerceptionFlow message and stores it in the buffer
	 * <!-- end-UML-doc -->
	 * @param message
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	private synchronized void readSensing(PerceptionFlow message) {
		// begin-user-code
		for (Iterator<RosSingularity> i = message.getSingularities().iterator(); i
				.hasNext();) {
				singularityBuffer.add(rosSing2compSing(i.next()));
		}

		// end-user-code
	}

	/** 
	 * <!-- begin-UML-doc -->
	 * method&nbsp;that&nbsp;converts&nbsp;information&nbsp;from&nbsp;a&nbsp;RosSingularity&nbsp;into&nbsp;an&nbsp;OMComponent<br>@param&nbsp;sing&nbsp;the&nbsp;perceptual&nbsp;singularity&nbsp;given<br>@return&nbsp;an&nbsp;OMComponent&nbsp;object
	 * <!-- end-UML-doc -->
	 * @param sing
	 * @return
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */

	private ComponentSingularity rosSing2compSing(RosSingularity sing) {
		// begin-user-code
		ComponentSingularity csing = new ComponentSingularity();

		csing.name = sing.getNodeName();

		/* from Ros information we can only know if the component is in contact with the master
		 * (we consider that as deployed) or not (we consider it as active, with no further 
		 * knowledge about errors here)
		 */		
		if (sing.getStatus() == 0){
			csing.status = ComponentStatus.MISSING;
			return csing;
		}
		else
			csing.status = ComponentStatus.ACTIVE;	// we assume ACTIVE by default

		for (Iterator<ROSParameter> i = sing.getParams().iterator(); i.hasNext();) {
			ROSParameter p = i.next();
			Object value;
			ByteBuffer buffer = p.getValue().toByteBuffer(); // TODO check if this works (I've changed it to work in fuerte)
			buffer.order(ByteOrder.LITTLE_ENDIAN);
			
			if (p.getType().equals(OMTypes.INTEGER)){
				value = buffer.getInt();
			}
			
			else if(p.getType().equals(OMTypes.DOUBLE)){
				value = buffer.getDouble();
			}

			
			else if(p.getType().equals(OMTypes.STRING)) {				
				final byte[] bytes = new byte[buffer.remaining()];			 
				buffer.duplicate().get(bytes);
				int length = bytes[0];				    
				value = new String(Arrays.copyOfRange(bytes, 1, length+1));				
			}					
			else
				value = null;
			
			csing.parameters.put(p.getName(), value);
				
		}
		
		// map published topics
		for (Iterator<ROSTopic> i = sing.getPubs().iterator(); i.hasNext();) {
			ROSTopic t = i.next();
			if(t.getName().equals("/rosout"))	// filter out /rosout topic	//TODO move to more generic
				continue;
			PortSingularity p = new PortSingularity(Directionality.OUT, t.getType(), t.getName());			
			csing.ports.put(t.getName(), p);
		}
		// map subscribed topics
		for (Iterator<ROSTopic> i = sing.getSubs().iterator(); i.hasNext();) {
			ROSTopic t = i.next();
			PortSingularity p = new PortSingularity(Directionality.IN, t.getType(), t.getName());			
			csing.ports.put(t.getName(), p);
		}
		// For debugging
		/*if(sing.node_name.equals("/fake_kinect_scan_node")){
			for( String key : csing.ports.keySet()){
				System.out.println(key);
			}
		}*/

		// incorporate log info
		if (!sing.getErrors().isEmpty())
			csing.internalState.put("log", sing.getErrors());
		
		/* internal state is merged when the sing for the component is processed (timing should be
		 * checked so that both sources of sensing are close in time)
		 */
		
		// if this is the amcl, store the last internal_state in the /amcl_pose entry for internalState
		if( csing.name.contains("amcl") ){	// TODO: too ad-hoc for Higgs
			csing.internalState.put("/amcl_pose", internal_states.get("/amcl_pose"));
		}
		

		return csing;
		// end-user-code
	}

	/** 
	 * <!-- begin-UML-doc -->
	 * method that returns the sensing buffer and empties it so as not to access it twice.
	 * This method is intended for a single client (a ComponentsECL)
	 * <!-- end-UML-doc -->
	 * @return the sensing in the buffer
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	@Override
	protected synchronized Set<ComponentSingularity> getComponentSensing() { // TODO : eliminate? buffer is clear in the corresponding getting method
		// begin-user-code
		Set<ComponentSingularity> aux = new HashSet<ComponentSingularity>();
		aux.addAll(singularityBuffer);
		singularityBuffer.clear();
		return aux;
		// end-user-code
	}

	/** 
	 * <!-- begin-UML-doc -->
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	@Override
	public synchronized void clearBuffer() {
		// begin-user-code
		singularityBuffer.clear();
		// end-user-code
	}

	
	/**
	 * retrieves the current buffer of feedback about the tasks (each one is a #ComponentAction)
	 * and empties it
	 * @return a map with the action ID and the corresponding feedback, for those actions that there is
	 */
	public synchronized Map<Short,ActionFeedback> getFeedback(){
		if( feedbackBuffer.isEmpty() )
			return Collections.emptyMap();
		else {
			Map<Short,ActionFeedback> aux = new HashMap<Short,ActionFeedback>(feedbackBuffer);
			feedbackBuffer.clear();
			return aux;
		}
	}
	
	
	/**
	 * commands the manager to process the incoming ros message, and adds the result (if any) to
	 * the feedback buffer
	 * @param message
	 */
	private synchronized void processFeedback(MetaActionResult message) {
		ActionFeedback feedback = manager.processFeedback(message);
		if( feedback == null )
			return;
		
		feedbackBuffer.put(feedback.actionID, feedback);
	}


	/** 
	 * <!-- begin-UML-doc -->
	 * sends the reconfiguration action to the manager to execute (the manager distributes the
	 * commands for the reconfiguration among several executors)
	 * <!-- end-UML-doc -->
	 * @param action the reconfiguration of components action that is to be addressed
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	@Override
	protected void executeReconfiguration(Set<ReconfigurationCommand> action) {
		manager.setReconfigurationTask(action);
	}
	
	
	/**
	 * @see org.aslab.om.ecl.perception.Sensor#configureFilter(java.util.Set, java.util.Set)
	 */
	@Override
	public void configureFilter(Set<String> observe, Set<String> ignore){
		final MessageFactory msg_factory = node.getTopicMessageFactory();
		
		// initial submission of referents (so as not to perceive them)	    
		ComponentReferents refs = referents_publisher.newMessage();
		
		ComponentReferent ref;
		
		for (Iterator<String> iter = observe.iterator(); iter.hasNext();) {
			String n = iter.next();
			if( n != null ){
				ref = msg_factory.newFromType("OMRosDrivers_py/ComponentReferent");
				ref.setName(n);
				ref.setType("OBSERVE");
				refs.getStatus().add(ref);
			}
		}

		for (Iterator<String> iter = ignore.iterator(); iter.hasNext();) {
			String n = iter.next();
			if( n != null ){
				ref = msg_factory.newFromType("OMRosDrivers_py/ComponentReferent");
				ref.setName(n);
				ref.setType("IGNORE");
				refs.getStatus().add(ref);
			}
		}
		
		referents_publisher.publish(refs);
	}
}
