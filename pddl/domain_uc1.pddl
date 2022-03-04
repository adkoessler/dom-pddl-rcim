(define (domain mrod)
    (:requirements :strips :typing :action-costs :adl)
	(:types
		cell actuator actuatorType actuatorSet perceptor perceptorType perceptorSet do doState stateValue variable variableRole variableType tool toolType grasp graspType graspSet mdl mdlDescType mdlDescBehav mdlDescMethod
	)
	(:constants
		spatialRobot planarRobot rotationAxis - actuatorType
		jointPositionSensor jointVelocitySensor tcpPositionSensor tcpVelocitySensor tcpForceSensor toolTactileSensor Vision2dSensor Vision3dSensor - perceptorType
		unidim bidim tridim cloth origami - doDescType
		isometric elastic elastoplastic noflex generic - doDescBehav
		textured textureless - doDescTex
		statePosition stateShape stateStress - doState
		initialValue targetValue validValue unknownValue - stateValue
		measured control target - variableRole
		jointPosition jointVelocity jointTorque tcpPosition tcpVelocity tcpForce tcpHybridFP objectPosition objectShape objectTension objectStress object2DImage objectPointCloud objectPartialPointCloud - variableType
		twoFingerGripper threeFingerGripper magneticGripper suctionGripper - toolType
		pinchGrasp scissorGrasp magnetGrasp suctionGrasp - graspType
		learned geometric constitutive - mdlDescMethod
	)
	(:predicates
		;actuators
		(activeA ?a - actuator)
		(hasTypeA ?a - actuator ?at - actuatorType)
		(belongsA ?a - actuator ?as - actuatorSet)
		(hasMountedT ?a - actuator ?t - tool)
		(canMountT ?a - actuator ?tt - toolType)
		(hasMountedP ?a - actuator ?p - perceptor)
		(canMountP ?a - actuator ?pt - perceptorType)
		(hasControlledV ?a - actuator ?v - variable)
		(canControlV ?a - actuator ?vt - variableType)
		;(canHandleO ?a - actuator ?dt - doDim)
		;(hasHandledO ?a - actuator ?do - do)
		;perceptors
		(activeP ?p - perceptor)
		(hasTypeP ?p - perceptor ?pt - perceptorType)
		(belongsP ?p - perceptor ?ps - perceptorSet)
		(canPerceiveO ?p - perceptor ?do - do)
		(canMeasureV ?p - perceptor ?vt - variableType)
		(hasMeasuredOV ?do - do ?v - variable) ;for measuring object values
		(hasMeasuredAV ?a - actuator ?v - variable) ;for measuring actuator values
		(hasMountedCP ?c - cell ?p - perceptor)
		;objects
		(hasTypeO ?do - do ?dt - doDescType)
		(hasBehavO ?do - do ?db - doDescBehav)
		(hasTexO ?do - do ?dx - doDescTex)
		(hasStateValue ?do - do ?ds - doState ?sv - stateValue)
		;variables
		(isStreamedV ?v - variable)
		(hasTypeV ?v - variable ?vt - variableType)
		(hasRoleV ?v - variable ?vr - variableRole)
		;tools
		(hasTypeT ?t - tool ?tt - toolType)
		(canHandleT ?tt - toolType ?dt - do)
		(canPerformGrasp ?tt - toolType ?gt - graspType)
		;grasping
		(activeG ?g - grasp)
		(hasTypeG ?g - grasp ?gt - graspType)
		(canHandleG ?gt - graspType ?do - do)
		(belongsG ?g - grasp ?gs - graspSet)
		(hasGrasped ?t - tool ?dt - do ?a - actuator ?g - grasp)
		;model
		(hasTypeM ?m - mdl ?dt - doDescType)
		(hasBehavM ?m - mdl ?db - doDescBehav)
		(hasMethodM ?m - mdl ?mm - mdlDescMethod)
	)
	(:functions
		(perceptorTypeCount ?ps - perceptorSet ?pt - perceptorType) - number
		(actuatorTypeCount ?as - actuatorSet ?at - actuatorType) - number
		(graspCount ?do - do) - number
		;(actuationcount ?as - actuationset) - number
		;(innerloopcount ?as - actuationset ?i - innerloop) - number
		;(validgrasp ?do - do) - number
		;(validshape ?do - do) - number
		(total-cost) - number
	)
	
	; Nicolas controller
	;(:action controlUnrollingVelocityTension1D
	;	:parameters
	;		(?ar ?ab - actuator ?as - actuatorSet ?do - do ?svp ?svh ?svs - stateValue ?vin1 ?vin2 ?vin3 ?vout - variable)
	;	:precondition
	;		(and
	;			(hasDimO ?do linear) (hasHandledO ?ar ?do) (hasHandledO ?ab ?do) ;1st condition: object handled is linear
	;			(belongsA ?ar ?as) (hasTypeA ?ar spatialRobot)  ;2nd condition: one actuator is a robot with a simpleGripper tool
	;			(belongsA ?ab ?as) (hasTypeA ?ab rotationAxis) (hasControlledV ?ab ?vout) (hasTypeV ?vout jointVelocity) ;3rd condition: one actuator is an axis with velocity control
	;			(hasTypeV ?vin1 objectStress) (hasMeasuredOV ?do ?vin1) ;4th condition: a measurement of object stress is available 
	;			(hasTypeV ?vin2 objectShape) (hasMeasuredOV ?do ?vin2) ;5th condition: a measurement of object shape is available
	;			(hasTypeV ?vin3 jointVelocity) (hasMeasuredAV ?ab ?vin3) ;6th condition: coil has velocity proprioception
	;		)
	;	:effect
	;		(and
	;			(not(hasStateValue ?do statePosition ?svp)) ;delete initial stateValues
	;			(not(hasStateValue ?do stateShape ?svh))
	;			(not(hasStateValue ?do stateStress ?svs))
	;			(hasStateValue ?do statePosition targetValue) ;replace with outcomes
	;			(hasStateValue ?do stateShape validValue)
	;			(hasStateValue ?do stateStress validValue)
	;			(increase (total-cost) 100)
	;		)
	;)

	; Forward catenary model: get shape from tension and grasp point coordinates
	(:action catenaryShapeEstimator
		:parameters
			(?as - actuatorSet ?gs - graspSet ?do - do ?mdl - mdl ?pose1 ?pose2 ?tension ?shape - variable)
		:precondition
			(and
				(exists(?g1 ?g2 - grasp) ;1/grasping configuration is not degenerate
					(and
						(belongsG ?g1 ?gs)(belongsG ?g2 ?gs)(not (= ?g1 ?g2))
					)
				)
				(exists(?a1 ?a2 - actuator) ;2/pose of both grasping points from different actuators can be retrieved
					(and
						(exists (?t - tool ?g1 - grasp)(hasGrasped ?t ?do ?a1 ?g1))
						(hasMeasuredAV ?a1 ?pose1)
						(hasTypeV ?pose1 tcpPosition)
						(exists (?t - tool ?g2 - grasp)(hasGrasped ?t ?do ?a2 ?g2))
						(hasMeasuredAV ?a2 ?pose2)
						(hasTypeV ?pose2 tcpPosition)
						(belongsA ?a1 ?as)(belongsA ?a2 ?as)(not (= ?a1 ?a2))
						(hasMeasuredAV ?a1 ?tension)(hasTypeV ?tension tcpForce) ;3/force from one actuator can be retrieved
					)
				)
				(hasTypeM ?mdl unidim)(hasBehavM ?mdl isometric)(hasBehavM ?mdl noflex) ;4/model is conform to catenary
				(hasTypeO ?do unidim)(hasBehavO ?do isometric)(hasBehavO ?do noflex) ;5/object is conform to catenary
				(hasTypeV ?shape objectShape) ;6/output variable has the correct type
			)
		:effect
			(and
				(hasMeasuredOV ?do ?shape) ;shape of manipulated object is known
				(increase (total-cost) 0)
			)
	)
	
	; Inverse catenary model: get tension from shape and grasp point coordinates
	(:action catenaryTensionEstimator
		:parameters
			(?as - actuatorSet ?gs - graspSet ?do - do ?mdl - mdl ?pose1 ?pose2 ?tension ?shape - variable)
		:precondition
			(and
				(exists(?g1 ?g2 - grasp) ;1/grasping configuration is not degenerate
					(and
						(belongsG ?g1 ?gs)(belongsG ?g2 ?gs)(not (= ?g1 ?g2))
					)
				)
				(exists(?a1 ?a2 - actuator) ;2/pose of both grasping points from different actuators can be retrieved
					(and
						(exists (?t - tool ?g1 - grasp)(hasGrasped ?t ?do ?a1 ?g1))
						(hasMeasuredAV ?a1 ?pose1)
						(hasTypeV ?pose1 tcpPosition)
						(exists (?t - tool ?g2 - grasp)(hasGrasped ?t ?do ?a2 ?g2))
						(hasMeasuredAV ?a2 ?pose2)
						(hasTypeV ?pose2 tcpPosition)
						(belongsA ?a1 ?as)(belongsA ?a2 ?as)(not (= ?a1 ?a2))
						(hasMeasuredOV ?do ?shape)(hasTypeV ?shape objectShape) ;3/object shape can be retrieved
					)
				)
				(hasTypeM ?mdl unidim)(hasBehavM ?mdl isometric)(hasBehavM ?mdl noflex) ;4/model is conform to catenary
				(hasTypeO ?do unidim)(hasBehavO ?do isometric)(hasBehavO ?do noflex)  ;5/object is conform to catenary
				(hasTypeV ?tension objectTension) ;6/output variable has the correct type
			)
		:effect
			(and
				(hasMeasuredOV ?do ?tension) ;tension of manipulated object is known
				(increase (total-cost) 0)
			)
	)

  ; Catenary model: get tension from force measurement at an extremity
	; Does not involve a catenary model since horizontal tension is constant
	;(:action catenaryTensionSensing
	;	:parameters
	;		(?as - actuatorSet ?gs - graspSet ?do - do ?f1 ?tension - variable)
	;	:precondition
	;		(and
	;			(exists(?g1 ?g2 - grasp) ;1/grasping configuration is not degenerate
	;				(and
	;					(belongsG ?g1 ?gs)(belongsG ?g2 ?gs)(not (= ?g1 ?g2))
	;				)
	;			)
	;			(exists(?a1 - actuator ?g1 - grasp) ;2/force measurement at one actuator can be retrieved
	;				(and
	;					(exists (?t - tool)(hasGrasped ?t ?do ?a1 ?g1))
	;					(hasMeasuredAV ?a1 ?f1)
	;					(hasTypeV ?f1 tcpForce)
	;				)
	;			)
	;			(hasTypeO ?do unidim)(hasBehavO ?do isometric)(hasBehavO ?do noflex)  ;3/no model, but object shoudl still conform to catenary
	;			(hasTypeV ?tension objectTension) ;4/output variable has the correct type
	;		)
	;	:effect
	;		(and
	;			(hasMeasuredOV ?do ?tension) ;tension of manipulated object is known
	;			(increase (total-cost) 0)
	;		)
	;)
	
	; Catenary curve extrapolator from RGB-D vision
	(:action catenaryPclExtrapolator
		:parameters
			(?do - do ?mdl - mdl ?shape ?pcl - variable)
		:precondition
			(and
				(hasMeasuredOV ?do ?pcl)(hasTypeV ?pcl objectPartialPointCloud) ;1/object PCL can be retrieved
				(hasTypeM ?mdl unidim)(hasBehavM ?mdl isometric) ;2/model is conform to catenary
				(hasTypeO ?do unidim)(hasBehavO ?do isometric) ;3/object is conform to catenary
				(hasTypeV ?shape objectShape) ;4/output variable has the correct type
			)
		:effect
			(and
				(hasMeasuredOV ?do ?shape) ;shape of manipulated object is known
				(increase (total-cost) 0)
			)
	)
	
	; Coordinated catenary tension servoing with PI controller
	(:action catenaryCoordPropTensionServoing
		:parameters
			(?as - actuatorSet ?a1 ?a2 - actuator ?gs - graspSet ?g1 ?g2 - grasp ?do - do ?svp ?svh ?svs - stateValue ?tension ?a1vel ?a2vel ?a1prop ?a2prop - variable)
		:precondition
			(and
				(belongsA ?a1 ?as)(hasTypeA ?a1 spatialRobot)(hasControlledV ?a1 ?a1vel)(hasTypeV ?a1vel jointVelocity) ;1/actuator 1 is velocity controlled
				(belongsA ?a2 ?as)(hasTypeA ?a2 spatialRobot)(hasControlledV ?a2 ?a2vel)(hasTypeV ?a2vel jointVelocity) ;2/actuator 2 is velocity controlled
				(belongsG ?g1 ?gs)(exists (?t - tool)(hasGrasped ?t ?do ?a1 ?g1)) ;3/object is actuated through grasp 1
				(belongsG ?g2 ?gs)(exists (?t - tool)(hasGrasped ?t ?do ?a2 ?g2)) ;4/object is actuated through grasp 2
				(not (= ?g1 ?g2)) ;5/grasping configuration is not degenerate
				(hasTypeV ?tension objectTension) (hasMeasuredOV ?do ?tension) ;4/object tension can be retrieved
				(hasTypeV ?a1prop jointVelocity) (hasMeasuredAV ?a1 ?a1prop) ;5/actuator 1 has velocity proprioception to enable servoing
				(hasTypeV ?a2prop jointVelocity) (hasMeasuredAV ?a2 ?a2prop) ;6/actuator 2 has velocity proprioception to enable servoing
			)
		:effect
			(and
				(not(hasStateValue ?do statePosition ?svp)) ;delete initial stateValues
				(not(hasStateValue ?do stateShape ?svh))
				(not(hasStateValue ?do stateStress ?svs))
				(hasStateValue ?do statePosition targetValue) ;replace with outcomes
				(hasStateValue ?do stateShape validValue)
				(hasStateValue ?do stateStress targetValue)
				(increase (total-cost) 100)
			)
	)

	; Coordinated catenary tension servoing with HFP controller (Flixeder2017)
	;(:action catenaryCoordHfpTensionServoing
	;	:parameters
	;		(?as - actuatorSet ?a1 ?a2 - actuator ?gs - graspSet ?g1 ?g2 - grasp ?do - do ?svp ?svh ?svs - stateValue ?tension ?shape ?a1pos ?a2hfp ?a1prop ?a2prop - variable)
	;	:precondition
	;		(and
	;			(belongsA ?a1 ?as)(hasTypeA ?a1 spatialRobot)(hasControlledV ?a1 ?a1pos)(hasTypeV ?a1pos jointPosition) ;1/actuator 1 is position controlled
	;			(belongsA ?a2 ?as)(hasTypeA ?a2 spatialRobot)(hasControlledV ?a2 ?a2hfp)(hasTypeV ?a2hfp tcpHybridFP) ;2/actuator 2 is hybrid force/position controlled
	;			(belongsG ?g1 ?gs)(exists (?t - tool)(hasGrasped ?t ?do ?a1 ?g1)) ;3/object is actuated through grasp 1
	;			(belongsG ?g2 ?gs)(exists (?t - tool)(hasGrasped ?t ?do ?a2 ?g2)) ;4/object is actuated through grasp 2
	;			(not (= ?g1 ?g2)) ;5/grasping configuration is not degenerate
	;			(hasTypeV ?tension objectTension) (hasMeasuredOV ?do ?tension) ;4/object tension can be retrieved
	;			(hasTypeV ?shape objectShape) (hasMeasuredOV ?do ?shape) ;4/object shape can be retrieved (to position desired point precisely)
	;			(hasTypeV ?a1prop jointPosition) (hasMeasuredAV ?a1 ?a1prop) ;5/actuator 1 has position proprioception to enable servoing
	;			(hasTypeV ?a2prop tcpHybridFP) (hasMeasuredAV ?a2 ?a2prop) ;6/actuator 2 has HFP proprioception to enable servoing
	;		)
	;	:effect
	;		(and
	;			(not(hasStateValue ?do statePosition ?svp)) ;delete initial stateValues
	;			(not(hasStateValue ?do stateShape ?svh))
	;			(not(hasStateValue ?do stateStress ?svs))
	;			(hasStateValue ?do statePosition targetValue) ;replace with outcomes
	;			(hasStateValue ?do stateShape validValue)
	;			(hasStateValue ?do stateStress targetValue)
	;			(increase (total-cost) 100)
	;		)
	;)
	
	;=== Grasping ==================================================================================================
	
	(:action activateG
		:parameters
			(?g - grasp ?gt - graspType ?t - tool ?tt - toolType ?a - actuator ?do - do)
		:precondition
			(and
				(not(activeG ?g)) ;1/grasping has not yet been performed
				(hasMountedT ?a ?t) ;2/tool is mounted on actuator
				(hasTypeT ?t ?tt)(canPerformGrasp ?tt ?gt) ;3/tool can perform a certain grasp type...
				(canHandleG ?gt ?do) ;4/... that is suitable to object
			)
		:effect
			(and
				(hasGrasped ?t ?do ?a ?g)
				(hasTypeG ?g ?gt)
				(activeG ?g)
				(increase (graspCount ?do) 1)
			)
	)
	
	(:action deactivateG
		:parameters
			(?g - grasp ?gt - graspType ?t - tool ?tt - toolType ?a - actuator ?do - do)
		:precondition
			(and
				(activeG ?g) ;1/grasping is performed
				(hasGrasped ?t ?do ?a ?g) ;2/grasping involves an actuator and a tool
				(hasTypeG ?g ?gt)
				(>= (graspCount ?do) 1)
			)
		:effect
			(and
				(not (hasGrasped ?t ?do ?a ?g))
				(not (activeG ?g))
				(not (hasTypeG ?g ?gt))
				(decrease (graspCount ?do) 1)
			)
	)
	
	;=== Sets Management ==================================================================================================
	
	(:action activateA
		:parameters
			(?a - actuator)
		:precondition
			(not(activeA ?a))
		:effect
			(and
				(activeA ?a)
				(forall (?p - perceptor)
					(when
						(hasMountedP ?a ?p)
						(activeP ?p)
					)
				)
			)
	)

	(:action deactivateA
		:parameters
			(?a - actuator)
		:precondition
			(activeA ?a)
		:effect
			(and
				(not(activeA ?a))
				(forall (?p - perceptor)
					(when
						(hasMountedP ?a ?p)
						(not(activeP ?p))
					)
				)
			)
	)

	(:action activateP
		:parameters
			(?p - perceptor)
		:precondition
			(and
				(not(activeP ?p))
				(not
					(exists (?a - actuator)
						(hasMountedP ?a ?p)
					)
				)
			)
		:effect
			(and
				(activeP ?p)
			)
	)
	  
	(:action deactivateP
		:parameters
			(?p - perceptor)
		:precondition
			(activeP ?P)
		:effect
			(and
				(not(activeP ?p))
			)
	)

	(:action addASet
		:parameters
			(?a - actuator ?as - actuatorSet)
		:precondition
			(and
				(activeA ?a)
				(not(belongsA ?a ?as))
				(not
					(exists(?as2 - actuatorSet)
						(belongsA ?a ?as2)
					)
				)
			)
		:effect
			(belongsA ?a ?as)
	)

	(:action removeASet
		:parameters
			(?a - actuator ?as - actuatorSet)
		:precondition
			(and
				(activeA ?a)
				(belongsA ?a ?as)
			)
		:effect
			(not(belongsA ?a ?as))
	)

	(:action addPSet
		:parameters
			(?p - perceptor ?ps - perceptorSet)
		:precondition
			(and
				(activeP ?p)
				(not(belongsP ?p ?ps))
			)
		:effect
			(belongsP ?p ?ps)
	)

	(:action removePSet
		:parameters
			(?p - perceptor ?ps - perceptorSet)
		:precondition
			(and
				(activeP ?p)
				(belongsP ?p ?ps)
			)
		:effect
			(not(belongsP ?p ?ps))
	)
	
	(:action addGSet
		:parameters
			(?g - grasp ?gs - graspSet)
		:precondition
			(and
				(activeG ?g)
				(not(belongsG ?g ?gs))
			)
		:effect
			(belongsG ?g ?gs)
	)

	(:action removeGSet
		:parameters
			(?g - grasp ?gs - graspSet)
		:precondition
			(and
				(activeG ?g)
				(belongsG ?g ?gs)
			)
		:effect
			(not(belongsG ?g ?gs))
	)
	
	;=== Tool and innerloop management ==================================================================================================
	
	; Change actuator innerloop
	(:action controlV
		:parameters
			(?a - actuator ?v - variable ?vt - variableType)
		:precondition
			(and
				(not(hasControlledV ?a ?v)) ;not already controlled by ?v
				(not
					(exists (?v2 - variable)
						(hasControlledV ?a ?v2) ;not already controlled by another var
					)
				)
				(not
					(exists (?a2 - actuator)
						(hasControlledV ?a2 ?v) ;var not already controlling another actuator
					)
				)
				(hasTypeV ?v ?vt)
				(canControlV ?a ?vt)
				(hasRoleV ?v control)
			)
		:effect
			(hasControlledV ?a ?v)
	)

	; Change actuator innerloop
	(:action uncontrolV
		:parameters
			(?a - actuator ?v - variable ?vt - variableType)
		:precondition
			(and
				(hasControlledV ?a ?v) ;already controlled by ?v
				(hasTypeV ?v ?vt)
				(canControlV ?a ?vt)
				(hasRoleV ?v control)
			)
		:effect
			(not(hasControlledV ?a ?v))
	)

	; Change tool
	(:action mountT
		:parameters
			(?a - actuator ?t - tool ?tt - toolType)
		:precondition
			(and
				(not(hasMountedT ?a ?t)) ;tool not already mounted on ?a
				(not
					(exists (?a2 - actuator)
						(hasMountedT ?a2 ?t) ;tool not already mounted on another actuator
					)
				)
				(not
					(exists (?t2 - tool)
						(hasMountedT ?a ?t2) ;no other tool already mounted on ?a
					)
				)
				(hasTypeT ?t ?tt)
				(canMountT ?a ?tt)
			)
		:effect
			(and
				(hasMountedT ?a ?t)
				(increase (total-cost) 10)
			)
	)

	; Change tool
	(:action unmountT
		:parameters
			(?a - actuator ?t - tool ?tt - toolType)
		:precondition
			(and
				(hasMountedT ?a ?t)
				(hasTypeT ?t ?tt)
				(canMountT ?a ?tt)
			)
		:effect
			(and
				(not(hasMountedT ?a ?t))
				(increase (total-cost) 10)
			)
	)
	
	;=== Sensor management ==================================================================================================
	
	; Get object data from sensors
	(:action getExteroSensorData
		:parameters
			(?ps - perceptorSet ?p - perceptor ?v - variable ?vt - variableType ?do - do)
		:precondition
			(and
				(not(hasMeasuredOV ?do ?v)) ;data is not already gotten
				(not
					(exists (?do2 - do)
						(hasMeasuredOV ?do2 ?v) ;var not already used by another exteroceptor
					)
				)
				(activeP ?p) ;sensor is active
				(belongsP ?p ?ps) ;sensor is in perceptorSet
				(canMeasureV ?p ?vt) ;sensor can measure variableType
				(hasTypeV ?v ?vt) ;variable has appropriate type
				(canPerceiveO ?p ?do) ;sensor can percieve object
				(hasRoleV ?v measured) ;variable is made for measurement
			)
		:effect
			(and
				(hasMeasuredOV ?do ?v)
			)
	)

	; Get actuator data from sensors
	(:action getProprioSensorData
		:parameters
			(?ps - perceptorSet ?p - perceptor ?v - variable ?vt - variableType ?a - actuator ?as - actuatorSet)
		:precondition
			(and
				(not(hasMeasuredAV ?a ?v)) ;data is not already gotten
				(not
					(exists (?a2 - actuator)
						(hasMeasuredAV ?a2 ?v) ;var not already used by another proprioceptor
					)
				)
				(activeP ?p) ;sensor is active
				(belongsP ?p ?ps) ;sensor is in perceptorSet
				(activeA ?a) ;actuator is active
				(belongsA ?a ?as) ;actuator is in actuatorSet
				(canMeasureV ?p ?vt) ;sensor can measure variableType
				(hasTypeV ?v ?vt) ;variable has appropriate type
				(hasMountedP ?a ?p) ;sensor can percieve actuator
				(hasRoleV ?v measured) ;variable is made for measurement
			)
		:effect
			(and
				(hasMeasuredAV ?a ?v)
			)
	)
	
	;=== Kinematics ==================================================================================================
	
	; Transform jointPosition variable into toolPosition variable
	(:action enableFPK
		:parameters
			(?a - actuator ?vin ?vout - variable)
		:precondition
			(and
				(hasMeasuredAV ?a ?vin) ;input varible is measured by proprioception
				(hasTypeV ?vin jointPosition) ;input variable type is jointPosition
				(hasTypeV ?vout tcpPosition) ;output variable type is toolPosition
				(not
					(exists(?a2 - actuator)
						(hasMeasuredAV ?a2 ?vout) ;output variable is not used by another proprioceptor
					)	
				)
				(exists(?vr2 - variableRole)
					(and
						(hasRoleV ?vin ?vr2) ;input and output variable have the same role
						(hasRoleV ?vout ?vr2)
					)
				)
			)
		:effect
			(hasMeasuredAV ?a ?vout)
	)

	; Transform toolPosition variable into jointPosition variable
	(:action enableIPK
		:parameters
			(?a - actuator ?vin ?vout - variable)
		:precondition
			(and
				(hasMeasuredAV ?a ?vin)
				(hasTypeV ?vin tcpPosition)
				(hasTypeV ?vout jointPosition)
				(not
					(exists(?a2 - actuator)
						(hasMeasuredAV ?a2 ?vout) ;output variable is not used by another proprioceptor
					)
				)
				(exists(?vr2 - variableRole)
					(and
						(hasRoleV ?vin ?vr2) ;input and output variable have the same role
						(hasRoleV ?vout ?vr2)
					)
				)
			)
		:effect
			(hasMeasuredAV ?a ?vout)
	)
	
	;=== Actions ==================================================================================================
	
	; Flixeder 2017 controller for coordinated transportation of 1D isometric objects
	;(:action coordTransportation1DFlixeder2017
	;	:parameters
	;		(?ar1 ?ar2 - actuator ?as - actuatorSet ?do - do ?svp ?svh ?svs - stateValue ?vin_st ?vin_pr1 ?vin_pr2 ?vin_fr2 ?vout_r1 ?vout_r2 - variable)
	;	:precondition
	;		(and
	;			(not(= ?ar1 ?ar2))
	;			(hasDimO ?do linear) (hasBehavO ?do isometric) (hasHandledO ?ar1 ?do) (hasHandledO ?ar2 ?do) ;1st condition: object handled by the two robots is linear and isometric
	;			(belongsA ?ar1 ?as) (hasTypeA ?ar1 spatialRobot) (hasControlledV ?ar1 ?vout_r1) (hasTypeV ?vout_r1 jointPosition) ;2nd condition: master actuator is position controlled
	;			(belongsA ?ar2 ?as) (hasTypeA ?ar2 spatialRobot) (hasControlledV ?ar2 ?vout_r2) (hasTypeV ?vout_r2 toolHybridFP) ;3rd condition: slave actuator is hybrid FP controlled
	;			(hasTypeV ?vin_st objectStress) (hasMeasuredOV ?do ?vin_st) ;4th condition: a measurement of object stress at slave robot grasp is available 
	;			(hasTypeV ?vin_pr1 jointPosition) (hasMeasuredAV ?ar1 ?vin_pr1) ;5th condition: a measurement of master robot tool pose is available
	;			(hasTypeV ?vin_pr2 jointPosition) (hasMeasuredAV ?ar2 ?vin_pr2) (hasTypeV ?vin_fr2 toolForce) (hasMeasuredAV ?ar2 ?vin_fr2) ;6th condition: a measurement of slave robot tool pose and force is avaliable
	;		)
	;	:effect
	;		(and
	;			(not(hasStateValue ?do statePosition ?svp)) ;delete initial stateValues
	;			(not(hasStateValue ?do stateShape ?svh))
	;			(not(hasStateValue ?do stateStress ?svs))
	;			(hasStateValue ?do statePosition targetValue) ;replace with outcomes
	;			(hasStateValue ?do stateShape unknownValue)
	;			(hasStateValue ?do stateStress targetValue)
	;			(increase (total-cost) 100)
	;		)
	;)
	
	; Aranda 2020: coordinated shape servoing of planar isometric objects
	;(:action coordShapeServo2DAranda2020
	;	:parameters
	;		(?ar1 ?ar2 - actuator ?as - actuatorSet ?do - do ?svp ?svh ?svs - stateValue ?vin_pr1 ?vin_pr2 ?vin_os ?vout_r1 ?vout_r2 - variable)
	;	:precondition
	;		(and
	;			(not(= ?ar1 ?ar2)) ;cannot use the same actuator twice
	;			(hasDimO ?do planar) (hasBehavO ?do isometric) (hasHandledO ?ar1 ?do) (hasHandledO ?ar2 ?do) ;1st condition: object handled by the two robots is linear and isometric
	;			(belongsA ?ar1 ?as) (hasTypeA ?ar1 spatialRobot) (hasControlledV ?ar1 ?vout_r1) (hasTypeV ?vout_r1 jointPosition) ;2nd condition: master actuator is position controlled
	;			(belongsA ?ar2 ?as) (hasTypeA ?ar2 spatialRobot) (hasControlledV ?ar2 ?vout_r2) (hasTypeV ?vout_r2 jointPosition) ;3rd condition: slave actuator is hybrid FP controlled
	;			(hasTypeV ?vin_os objectShape) (hasMeasuredOV ?do ?vin_os) ;4th condition: a measurement of object shape is available
	;			(hasTypeV ?vin_pr1 jointPosition) (hasMeasuredAV ?ar1 ?vin_pr1) ;5th condition: a measurement of master robot tool pose is available
	;			(hasTypeV ?vin_pr2 jointPosition) (hasMeasuredAV ?ar2 ?vin_pr2) ;6th condition: a measurement of slave robot tool pose is avaliable
	;		)
	;	:effect
	;		(and
	;			(not(hasStateValue ?do statePosition ?svp)) ;delete initial stateValues
	;			(not(hasStateValue ?do stateShape ?svh))
	;			(not(hasStateValue ?do stateStress ?svs))
	;			(hasStateValue ?do statePosition targetValue) ;object pose is controlled (see IV.C.)
	;			(hasStateValue ?do stateShape targetValue) ;object shape is controlled
	;			(hasStateValue ?do stateStress unknownValue) ;tensions remain unknown
	;			(increase (total-cost) 100)
	;		)
	;)
	
	;Use SfT to deduce pointCloud from 2DImage
	;(:action enableShapeFromTemplate
	;	:parameters
	;		(?vin ?vout - variable ?do - do)
	;	:precondition
	;		(and
	;			(hasDimO ?do planar) ;SfT only for 2D objects
	;			(hasTexO ?do textured) ;SfT only for textured objects
	;			(hasMeasuredOV ?do ?vin) ;initial var is already measured
	;			(hasTypeV ?vin object2DImage) ;initial var is a 2DImage
	;			(hasTypeV ?vout objectShape) ;final var is a pointCloud
	;		)
	;	:effect
	;		(and
	;			(hasMeasuredOV ?do ?vout)
	;			(increase (total-cost) 10)
	;		)
	;)
	
	; Transform partialPointCloud variable into shape variable
	;(:action getShapeFromPartialPointCloud
	;	:parameters
	;		(?do - do ?vin ?vout - variable)
	;	:precondition
	;		(and
	;			(hasMeasuredOV ?do ?vin)
	;			(hastypeV ?vin objectPartialPointCloud)
	;			(hastypeV ?vout objectShape)
	;		)
	;	:effect
	;		(and
	;			(hasMeasuredOV ?do ?vout)
	;			(increase (total-cost) 100)
	;		)
	;)

	; Transform pointCloud variable into shape variable
	;(:action getShapeFromPointCloud
	;	:parameters
	;		(?do - do ?vin ?vout - variable)
	;	:precondition
	;		(and
	;			(hasMeasuredOV ?do ?vin)
	;			(hastypeV ?vin objectPointCloud)
	;			(hastypeV ?vout objectShape)
	;		)
	;	:effect
	;		(and
	;			(hasMeasuredOV ?do ?vout)
	;		)
	;)

	; Transform pointCloud variable into position variable
	;(:action getPositionFromPointCloud
	;	:parameters
	;		(?do - do ?vin ?vout - variable)
	;	:precondition
	;		(and
	;			(hasMeasuredOV ?do ?vin)
	;			(hastypeV ?vin objectPointCloud)
	;			(hastypeV ?vout objectPosition)
	;		)
	;	:effect
	;		(and
	;			(hasMeasuredOV ?do ?vout)
	;		)
	;)

	; Give ability to vision sensor to measure object tension
	;(:action enable1DTensionFromShape
	;	:parameters
	;		(?vin ?vout - variable ?do - do)
	;	:precondition
	;		(and
	;			(hasDimO ?do linear) ;only for linear objects
	;			(hasMeasuredOV ?do ?vin) ;initial var is already measured
	;			(hasTypeV ?vin objectShape) ;initial var is a shape
	;			(hasTypeV ?vout objectStress) ;final var is a stress
	;		)
	;	:effect
	;		(and
	;			(hasMeasuredOV ?do ?vout)
	;			(increase (total-cost) 10)
	;		)
	;)

	; Give ability to force sensor to measure object tension
	;(:action enable1DTensionFromForce 
	;	:parameters
	;		(?vin ?vout - variable ?do - do ?a - actuator)
	;	:precondition
	;		(and
	;			(hasDimO ?do linear) ;only for linear objects
	;			(hasHandledO ?a ?do) ;object is handled by actuator
	;			(hasMeasuredAV ?a ?vin) ;initial var is already measured
	;			(hasTypeV ?vin toolForce) ;initial var is a force
	;			(hasTypeV ?vout objectStress) ; final var is a stress
	;		)
	;	:effect
	;		(and
	;			(hasMeasuredOV ?do ?vout)
	;			(increase (total-cost) 100)
	;		)
	;)
	
	; Handle object
	;(:action handleO
	;	:parameters
	;		(?a - actuator ?do - do ?dt - doDim ?t - tool ?tt - toolType)
	;	:precondition
	;		(and
	;			(not(hasHandledO ?a ?do))
	;			(hasMountedT ?a ?t)
	;			(hasTypeT ?t ?tt)
	;			(hasDimO ?do ?dt)
	;			(canHandleOT ?tt ?do)
	;		)
	;	:effect
	;		(hasHandledO ?a ?do)
	;)

	; Handle object
	;(:action unhandleO
	;	:parameters
	;		(?a - actuator ?do - do ?dt - doDim ?t - tool ?tt - toolType)
	;	:precondition
	;		(and
	;			(hasHandledO ?a ?do)
	;			(hasMountedT ?a ?t)
	;			(hasTypeT ?t ?tt)
	;			(hasDimO ?do ?dt)
	;			(canHandleOT ?tt ?do)
	;		)
	;	:effect
	;		(not(hasHandledO ?a ?do))
	;)
)

