package org.aslab.om.ros;

import java.util.Scanner;

import org.aslab.om.metacontrol.OM_KDB;
import org.aslab.om.metacontrol.OMmetacontroller;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.parameter.*;


/** 
 * <!-- begin-UML-doc -->
 * <p>@author carlos.hernandez@upm.es (Carlos Hernandez)</p><p>implements a ros node in java which is an OM metacontroller. For that it uses an instance of the OMmetacontroller class and an instance of OMRosAPI, which implements the I/O interface to the plant (the ros running system) of the metacontroller</p><p>preconditions: It assumes an external roscore is already running.</p><p></p>
 * <!-- end-UML-doc -->
 * @author chcorbato
 * @!generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
 */
public class OMMetacontrollerNode extends AbstractNodeMain {

	/** 
	 * <!-- begin-UML-doc -->
	 * object that encapsulates I/O to the running Ros platform
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	private OMRosAPI plant;

	/** 
	 * <!-- begin-UML-doc -->
	 * the OM metacontroller
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	private OMmetacontroller metacontrol;
	

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("meta_controller");
	}

	/** 
	 * <!-- begin-UML-doc -->
	 * Main function:
	 * 	- initializes the connection to the Ros master (node object)
	 *  - initializes the meta-I/O (plant object)
	 *  - initalizes the metacontroller (metacontrol object)
	 * <!-- end-UML-doc -->
	 * @param configuration
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */

	@Override
	public void onStart(final ConnectedNode node) {

		node.getLog().info("Initializing meta_controller node");
					
		// Load the KD depending on the Ros parameter /test
		String test_name;

		ParameterTree params = node.getParameterTree();
		try{
			test_name = params.getString("/test");
		}
		catch (Exception e) {
			test_name = "Testbed2a";
		}
		
		OM_KDB kdb;
		try{
			kdb = (OM_KDB) Class.forName("om_testbed2." + test_name + "_KDB").newInstance();
			node.getLog().info(test_name);
		} catch (Exception e){
			node.getLog().error(test_name + "_KDB class not loaded because it was nor found, terminating org.aslab.om.metacontrol node...\n");
			onShutdown(node);
			System.out.println("\t ... Ros node org.aslab.om.metacontrol terminated");
			System.exit(1);
			return;
		}
		
		// TODO : improve here creation and management of ECL and everything
		plant = new OMRosAPI(node, kdb.ckb);
		metacontrol = new OMmetacontroller(kdb.fg, kdb, plant);
		
		metacontrol.start();
		
		System.out.println("k - kill metacontroller\n"
				+ "p - pause execution of metacontroller\n"
				+ "s - resume execution of the metacontroller\n"
				+ "c - print components estimated state\n");
		

		node.executeCancellableLoop(new CancellableLoop() {
			@Override
		      protected void loop() throws InterruptedException {
				final Scanner input = new Scanner(System.in);
				switch (input.next().charAt(0)) {
				case 'c':
					System.out.println(metacontrol.getCompEstimatedState());
					break;
				case 'k':
					System.out.println("Metacontrol ended");
					metacontrol.stop();
					return;
				case 'p':
					metacontrol.disable();
					System.out.println("org.aslab.om.metacontrol paused");
					break;
				case 's':
					metacontrol.enable();
					System.out.println("org.aslab.om.metacontrol resumed");
					break;
				}
				Thread.sleep(1000);
			}
		});


		// end-user-code
	}

	/** 
	 * <!-- begin-UML-doc -->
	 * <!-- end-UML-doc -->
	 * @generated "UML to Java (com.ibm.xtools.transform.uml2.java5.internal.UML2JavaTransform)"
	 */
	@Override
	public void onShutdown( Node node) {
		// begin-user-code
		node.shutdown();
		node = null;
		// end-user-code
	}
	
	
	void user_input_loop(){
		
		System.out.println("k - kill metacontroller\n"
				+ "p - pause execution of metacontroller\n"
				+ "s - resume execution of the metacontroller\n"
				+ "c - print components estimated state\n");
		Scanner input = new Scanner(System.in);
		
		while (true) {
			switch (input.next().charAt(0)) {
			case 'c':
				System.out.println(metacontrol.getCompEstimatedState());
				break;
			case 'k':
				System.out.println("Metacontrol ended");
				metacontrol.stop();
				return;
			case 'p':
				metacontrol.disable();
				System.out.println("org.aslab.om.metacontrol paused");
				break;
			case 's':
				metacontrol.enable();
				System.out.println("org.aslab.om.metacontrol resumed");
				break;
			}
		}
	}

	@Override
	public void onError(Node arg0, Throwable arg1) {
		// TODO Auto-generated method stub
		
	}


	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}


}
