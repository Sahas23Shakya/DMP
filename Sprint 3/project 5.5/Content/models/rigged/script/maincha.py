# AdvancedSkeleton To ControlRig
#	Copyright (C) Animation Studios
# email: support@animationstudios.com.au
# exported using AdvancedSkeleton version:6.525
import unreal
import re

engineVersion = unreal.SystemLibrary.get_engine_version()
asExportVersion = 6.525
asExportTemplate = '5.4'
print ('AdvancedSkeleton To ControlRig (Unreal:'+engineVersion+') (AsExport:'+str(asExportVersion)+') (Template:'+asExportTemplate+')')
utilityBase = unreal.GlobalEditorUtilityBase.get_default_object()
selectedAssets = utilityBase.get_selected_assets()
if len(selectedAssets)<1:
	raise Exception('Nothing selected, you must select a ControlRig')
selectedAsset = selectedAssets[0]
if selectedAsset.get_class().get_name() != 'ControlRigBlueprint':
	raise Exception('Selected object is not a ControlRigBlueprint, you must select a ControlRigBlueprint')
blueprint = selectedAsset
RigGraphDisplaySettings = blueprint.get_editor_property('rig_graph_display_settings')
RigGraphDisplaySettings.set_editor_property('node_run_limit',256)
library = blueprint.get_local_function_library()
library_controller = blueprint.get_controller(library)
hierarchy = blueprint.hierarchy
hierarchy_controller = hierarchy.get_controller()
RigVMController = blueprint.get_controller() #UE5
PreviousArrayInfo = dict()
global ASCtrlNr
global PreviousEndPlug
global PreviousEndPlugInv
global PreviousYInv
global sp
global nonTransformFaceCtrlNr
PreviousYInv = 0
ASCtrlNr = 0
nonTransformFaceCtrlNr = -1
sp = '/Script/ControlRig.RigUnit_'
PreviousEndPlug = 'RigUnit_BeginExecution.ExecuteContext'
PreviousEndPlugInv = 'RigUnit_InverseExecution.ExecuteContext'

def asAddCtrl (name, parent, joint, type, arrayInfo, gizmoName, ws, size, offT, color):
	global PreviousEndPlug
	global PreviousEndPlugInv
	global PreviousYInv
	global PreviousArrayInfo
	global ctrlBoxSize
	global ASCtrlNr
	global nonTransformFaceCtrlNr
	endPlug = PreviousEndPlug
	RigVMGraph = blueprint.get_model()
	numNodes = len(RigVMGraph.get_nodes())
	y = ASCtrlNr*400
	ASCtrlNr=ASCtrlNr+1
	ASDrivenNr = int()
	RootScale = unreal.Vector(x=1.0, y=1.0, z=1.0)
	ParentRigBone = unreal.RigBone()
	ParentRigBoneName = parent.replace("FK", "")
	hasCon = True
	x = joint.split("_")
	if len(x)>1:
		baseName = x[0]
		side = '_'+x[1]
	x = ParentRigBoneName.split("_")
	if len(x)>1:
		ParentRigBoneBaseName = x[0]
	RigElementKeys = asGetRigElementKeys ()
	for key in RigElementKeys:
		if (key.name == 'Root_M'):
			hierarchy.get_global_transform(key, initial = True)
		if (key.name == ParentRigBoneName):
			if (key.type == 1):#Bone
				ParentRigBone = hierarchy.find_bone (key)
	asAddController (name, parent, joint, type, gizmoName, ws, size, offT, color)

	if name=='Main':
		return

	if name=='RootX_M':
		#Item
		Item = asAddNode (sp+'Item','Execute',node_name=name+'_Item')
		RigVMController.set_node_position (Item, [-500, y])
		RigVMController.set_pin_default_value(name+'_Item.Item.Type','Control')
		RigVMController.set_pin_default_value(name+'_Item.Item.Name',name)

		#CON
		CON = asAddNode (sp+'ParentConstraint','Execute',node_name=name+'_CON')
		RigVMController.set_node_position (CON, [100, y-90])
		RigVMController.set_pin_default_value(name+'_CON.Child.Type','Bone')
		RigVMController.set_pin_default_value(name+'_CON.Child.Name',joint)
		RigVMController.add_link(name+'_Item.Item', name+'_CON.Parents.0.Item')
		RigVMController.add_link(endPlug , name+'_CON.ExecuteContext')
		endPlug = name+'_CON.ExecuteContext'
	elif (not "inbetweenJoints" in arrayInfo) and (not "inbetweenJoints" in PreviousArrayInfo):
		#GT
		GT = asAddNode (sp+'GetTransform','Execute',node_name=name+'_GT')
		RigVMController.set_node_position (GT, [-500, y])
		RigVMController.set_pin_default_value(name+'_GT.Item.Type','Control')
		RigVMController.set_pin_default_value(name+'_GT.Item.Name',name)

		#ST
		ST = asAddNode (sp+'SetTransform','Execute',node_name=name+'_ST')
		RigVMController.set_node_position (ST, [100, y])
		RigVMController.set_pin_default_value(name+'_ST.Item.Type','Bone')
		RigVMController.set_pin_default_value(name+'_ST.Item.Name',joint)
		RigVMController.add_link(name+'_GT.Transform' , name+'_ST.Transform')
		RigVMController.set_pin_default_value(name+'_ST.bPropagateToChildren','True')
		RigVMController.add_link(endPlug , name+'_ST.ExecuteContext')
		endPlug = name+'_ST.ExecuteContext'

	#twistJoints
	if ("twistJoints" in arrayInfo) and (not "twistJoints" in PreviousArrayInfo):
		GT2 = asAddNode (sp+'GetTransform','Execute',node_name=name+'_GT2')
		RigVMController.set_node_position (GT2, [500, y+50])
		RigVMController.set_pin_default_value(name+'_GT2.Item.Type','Control')
		RigVMController.set_pin_default_value(name+'_GT2.Item.Name',name)
		RigVMController.set_pin_default_value(name+'_GT2.Space','LocalSpace')

		TWSW = asAddNode (sp+'MathQuaternionSwingTwist','Execute',node_name=name+'_TWSW')
		RigVMController.set_node_position (TWSW, [850, y+90])
		RigVMController.add_link(name+'_GT2.Transform.Rotation' , name+'_TWSW.Input')

		INV = asAddNode (sp+'MathQuaternionInverse','Execute',node_name=name+'_INV')
		RigVMController.set_node_position (INV, [850, y+220])
		RigVMController.add_link(name+'_TWSW.Twist' , name+'_INV.Value')

		OFF= asAddNode (sp+'OffsetTransformForItem','Execute',node_name=name+'_OFF')
		RigVMController.set_node_position (OFF, [1050, y])
		RigVMController.set_pin_default_value(name+'_OFF.Item.Type','Bone')
		RigVMController.set_pin_default_value(name+'_OFF.Item.Name',joint)
		RigVMController.add_link(name+'_INV.Result' , name+'_OFF.OffsetTransform.Rotation')
		RigVMController.add_link(endPlug , name+'_OFF.ExecuteContext')
		endPlug = name+'_OFF.ExecuteContext'

		GT3 = asAddNode (sp+'GetTransform','Execute',node_name=name+'_GT3')
		RigVMController.set_node_position (GT3, [1400, y+50])
		RigVMController.set_pin_default_value(name+'_GT3.Item.Type','Control')
		RigVMController.set_pin_default_value(name+'_GT3.Item.Name',name)

		ST2 = asAddNode (sp+'SetTranslation','Execute',node_name=name+'_ST2')
		RigVMController.set_node_position (ST2, [1700, y])
		RigVMController.set_pin_default_value(name+'_ST2.Item.Type','Bone')
		RigVMController.set_pin_default_value(name+'_ST2.Item.Name',joint)
		RigVMController.add_link(name+'_GT3.Transform.Translation' , name+'_ST2.Translation')
		RigVMController.add_link(endPlug , name+'_ST2.ExecuteContext')
		endPlug = name+'_ST2.ExecuteContext'

	if "twistJoints" in PreviousArrayInfo:
		twistJoints = int (PreviousArrayInfo["twistJoints"])
		TwistArray = asAddNode (sp+'CollectionChainArray','Execute',node_name=name+'_TwistArray')
		RigVMController.set_node_position (TwistArray , [500, y+50])
		RigVMController.set_pin_default_value(name+'_TwistArray.FirstItem', '(Type=Bone,Name='+ParentRigBoneBaseName+'Part1'+side+')')
		RigVMController.set_pin_default_value(name+'_TwistArray.LastItem', '(Type=Bone,Name='+ParentRigBoneBaseName+'Part'+str(twistJoints)+side+')')

		TwistArrayIterator = RigVMController.add_array_node_from_object_path(unreal.RigVMOpCode.ARRAY_ITERATOR, 'FRigElementKey', '/Script/ControlRig.RigElementKey', unreal.Vector2D(0, 0), name+'_TwistArrayIterator')
		RigVMController.set_node_position (TwistArrayIterator , [850, y])
		RigVMController.add_link(name+'_TwistArray.Items' , name+'_TwistArrayIterator.Array')
		RigVMController.add_link(endPlug , name+'_TwistArrayIterator.ExecuteContext')
		endPlug = name+'_TwistArrayIterator.Completed'

		GTRE = asAddNode (sp+'GetRelativeTransformForItem','Execute',node_name=name+'_GTRE')
		RigVMController.set_node_position (GTRE , [1050, y+50])
		RigVMController.set_pin_default_value(name+'_GTRE.Child', '(Type=Bone,Name='+joint+')')
		RigVMController.set_pin_default_value(name+'_GTRE.Parent', '(Type=Bone,Name='+ParentRigBoneName+')')

		TWSW = asAddNode (sp+'MathQuaternionSwingTwist','Execute',node_name=name+'_TWSW')
		RigVMController.set_node_position (TWSW, [1350, y+50])
		RigVMController.add_link(name+'_GTRE.RelativeTransform.Rotation' , name+'_TWSW.Input')

		TOEU = asAddNode (sp+'MathQuaternionToEuler','Execute',node_name=name+'_TOEU')
		RigVMController.set_node_position (TOEU, [1350, y+170])
		RigVMController.add_link(name+'_TWSW.Twist' , name+'_TOEU.Value')

		FREU = asAddNode (sp+'MathQuaternionFromEuler','Execute',node_name=name+'_FREU')
		RigVMController.set_node_position (FREU, [1350, y+270])
		RigVMController.add_link(name+'_TOEU.Result' , name+'_FREU.Euler')

		QS = asAddNode (sp+'MathQuaternionScale','Execute',node_name=name+'_QS')
		RigVMController.set_node_position (QS, [1550, y+50])
		RigVMController.set_pin_default_value(name+'_QS.Scale', str(1.0/int (PreviousArrayInfo["twistJoints"])))
		RigVMController.add_link(name+'_FREU.Result' , name+'_QS.Value')

		SR = asAddNode (sp+'SetRotation','Execute',node_name=name+'_SR')
		RigVMController.set_node_position (SR, [1700, y])
		RigVMController.add_link(name+'_QS.Value' , name+'_SR.Rotation')
		RigVMController.add_link(name+'_TwistArrayIterator.Element', name+'_SR.Item')
		RigVMController.set_pin_default_value(name+'_SR.Space','LocalSpace')
		RigVMController.set_pin_default_value(name+'_SR.bPropagateToChildren','False')
		RigVMController.add_link(name+'_TwistArrayIterator.ExecuteContext' , name+'_SR.ExecuteContext')

	#inbetweenJoints
	if "inbetweenJoints" in arrayInfo:
		inbetweenJoints = int (arrayInfo["inbetweenJoints"])
		Chain = asAddNode (sp+'CollectionChainArray','Execute',node_name=name+'_Chain')
		RigVMController.set_node_position (Chain, [500, y])
		RigVMController.set_pin_default_value(name+'_Chain.FirstItem.Name',joint)
		RigVMController.set_pin_default_value(name+'_Chain.LastItem.Name',baseName+'Part'+str(inbetweenJoints)+side)

		#GTDistr
		GTDistr = asAddNode (sp+'GetTransform','Execute',node_name=name+'_GTDistr')
		RigVMController.set_node_position (GTDistr, [850, y])
		RigVMController.set_pin_default_value(name+'_GTDistr.Item.Type','Control')
		RigVMController.set_pin_default_value(name+'_GTDistr.Item.Name',name)
		RigVMController.set_pin_default_value(name+'_GTDistr.Space','LocalSpace')

		#Distr
		Distr = asAddNode (sp+'DistributeRotationForItemArray','Execute',node_name=name+'_Distr')
		RigVMController.set_node_position (Distr, [1200, y])
		weight = (1.0 / inbetweenJoints)
		RigVMController.set_pin_default_value(name+'_Distr.Weight',str(weight))
		RigVMController.add_link(name+'_Chain.Items' , name+'_Distr.Items')
		RigVMController.add_array_pin(name+'_Distr.Rotations')
		RigVMController.add_link(name+'_GTDistr.Transform.Rotation' , name+'_Distr.Rotations.0.Rotation')
		RigVMController.add_link(endPlug , name+'_Distr.ExecuteContext')
		endPlug = name+'_Distr.ExecuteContext'

	if "inbetweenJoints" in PreviousArrayInfo:
		jointKey = asGetKeyFromName (joint)
		jointTransform = hierarchy.get_global_transform(jointKey, initial = True)
		NullKey = hierarchy_controller.add_null ('Null'+joint,asGetKeyFromName(parent),jointTransform)

		#GTNull
		GTNull = asAddNode (sp+'GetTransform','Execute',node_name=name+'_GTNull')
		RigVMController.set_node_position (GTNull, [1600, y+50])
		RigVMController.set_pin_default_value(name+'_GTNull.Item.Type','Bone')
		RigVMController.set_pin_default_value(name+'_GTNull.Item.Name',joint)

		#STNull
		STNull = asAddNode (sp+'SetTransform','Execute',node_name=name+'_STNull')
		RigVMController.set_node_position (STNull, [2000, y])
		RigVMController.set_pin_default_value(name+'_STNull.Item.Type','Null')
		RigVMController.set_pin_default_value(name+'_STNull.Item.Name','Null'+joint)
		RigVMController.add_link(name+'_GTNull.Transform' , name+'_STNull.Transform')
		RigVMController.set_pin_default_value(name+'_STNull.bPropagateToChildren','False')
		RigVMController.add_link(endPlug , name+'_STNull.ExecuteContext')
		endPlug = name+'_STNull.ExecuteContext'

		hierarchy_controller.set_parent(asGetKeyFromName(name),asGetKeyFromName('Null'+joint))
		hierarchy.set_control_offset_transform(asGetKeyFromName(name), unreal.Transform(),initial=True)
		hierarchy.set_local_transform(asGetKeyFromName(name), unreal.Transform(),initial=True)
		hierarchy.set_local_transform(asGetKeyFromName(name), unreal.Transform(),initial=False)

		#GT2
		GT2 = asAddNode (sp+'GetTransform','Execute',node_name=name+'_GT2')
		RigVMController.set_node_position (GT2, [2400, y])
		RigVMController.set_pin_default_value(name+'_GT2.Item.Type','Control')
		RigVMController.set_pin_default_value(name+'_GT2.Item.Name',name)

		#ST2
		ST2 = asAddNode (sp+'SetTransform','Execute',node_name=name+'_ST2')
		RigVMController.set_node_position (ST2, [2700, y])
		RigVMController.set_pin_default_value(name+'_ST2.Item.Type','Bone')
		RigVMController.set_pin_default_value(name+'_ST2.Item.Name',joint)
		RigVMController.add_link(name+'_GT2.Transform' , name+'_ST2.Transform')
		RigVMController.set_pin_default_value(name+'_ST2.bPropagateToChildren','True')
		RigVMController.add_link(endPlug , name+'_ST2.ExecuteContext')
		endPlug = name+'_ST2.ExecuteContext'

	if "global" in arrayInfo and float(arrayInfo["global"])==10:
		Transform = hierarchy.get_global_transform(asGetKeyFromName(name), initial = True)
		NullKey = hierarchy_controller.add_null ('Global'+name,asGetKeyFromName(parent),Transform)
		hierarchy_controller.set_parent(asGetKeyFromName(name),asGetKeyFromName('Global'+name),maintain_global_transform=True)
		hierarchy.set_control_offset_transform(asGetKeyFromName(name), unreal.Transform(), True, True)
		hierarchy.set_local_transform(asGetKeyFromName(name), unreal.Transform(),initial=True)
		hierarchy.set_local_transform(asGetKeyFromName(name), unreal.Transform(),initial=False)

		#resolve where `PreviousEndPlug` is connected to, as that is the start for this Row, which is not always the _ST node
		Pin = RigVMGraph.find_pin(PreviousEndPlug)
		LinkePins = Pin.get_linked_target_pins()
		PreviousEndPlugConnectedToPin = LinkePins[0].get_pin_path()

		PNPGlobal = asAddNode (sp+'ProjectTransformToNewParent','Execute',node_name=name+'_PNPGlobal')
		RigVMController.set_node_position (PNPGlobal, [-1200, y])
		RigVMController.set_pin_default_value(name+'_PNPGlobal.Child.Type','Control')
		RigVMController.set_pin_default_value(name+'_PNPGlobal.Child.Name',name)
		RigVMController.set_pin_default_value(name+'_PNPGlobal.OldParent.Type','Control')
		RigVMController.set_pin_default_value(name+'_PNPGlobal.OldParent.Name','Main')
		RigVMController.set_pin_default_value(name+'_PNPGlobal.NewParent.Type','Control')
		RigVMController.set_pin_default_value(name+'_PNPGlobal.NewParent.Name','Main')

		#SRGlobal
		SRGlobal = asAddNode (sp+'SetRotation','Execute',node_name=name+'_SRGlobal')
		RigVMController.set_node_position (SRGlobal, [-850, y])
		RigVMController.set_pin_default_value(name+'_SRGlobal.Item.Type','Null')
		RigVMController.set_pin_default_value(name+'_SRGlobal.Item.Name','Global'+name)
		RigVMController.set_pin_default_value(name+'_SRGlobal.bPropagateToChildren','True')
		RigVMController.add_link(name+'_PNPGlobal.Transform.Rotation' , name+'_SRGlobal.Rotation')
		RigVMController.add_link(PreviousEndPlug , name+'_SRGlobal.ExecuteContext')
		endPlug = name+'_SRGlobal.ExecuteContext'

		#STGlobal
		STGlobal = asAddNode (sp+'SetTranslation','Execute',node_name=name+'_STGlobal')
		RigVMController.set_node_position (STGlobal, [-850, y+250])
		RigVMController.set_pin_default_value(name+'_STGlobal.Item.Type','Null')
		RigVMController.set_pin_default_value(name+'_STGlobal.Item.Name','Global'+name)
		RigVMController.set_pin_default_value(name+'_STGlobal.Space','LocalSpace')
		RigVMController.set_pin_default_value(name+'_STGlobal.bPropagateToChildren','True')
		Transform = hierarchy.get_local_transform(NullKey)
		RigVMController.set_pin_default_value(name+'_STGlobal.Translation.X',str(Transform.translation.x))
		RigVMController.set_pin_default_value(name+'_STGlobal.Translation.Y',str(Transform.translation.y))
		RigVMController.set_pin_default_value(name+'_STGlobal.Translation.Z',str(Transform.translation.z))
		RigVMController.add_link(name+'_SRGlobal.ExecuteContext' , name+'_STGlobal.ExecuteContext')
		endPlug = name+'_STGlobal.ExecuteContext'

		RigVMController.add_link(endPlug , PreviousEndPlugConnectedToPin)
		endPlug = PreviousEndPlugConnectedToPin

	if type=='IK':
		asAddController ('Pole'+name, parent, joint, type, 'Sphere_Solid', ws, size/5.0, offT, color)
		hierarchy.set_control_offset_transform(asGetKeyFromName('Pole'+name), unreal.Transform(location=[float(arrayInfo["ppX"])*RootScale.x,float(arrayInfo["ppZ"])*RootScale.y,float(arrayInfo["ppY"])*RootScale.z],scale=RootScale), True, True)

		#IK(Basic IK)
		IK = asAddNode (sp+'TwoBoneIKSimplePerItem','Execute',node_name=name+'_IK')
		RigVMController.set_node_position (IK, [600, y-130])
		RigVMController.set_pin_default_value(name+'_IK.ItemA.Name',arrayInfo["startJoint"])
		RigVMController.set_pin_default_value(name+'_IK.ItemB.Name',arrayInfo["middleJoint"])
		RigVMController.set_pin_default_value(name+'_IK.EffectorItem.Name',arrayInfo["endJoint"])
		RigVMController.set_pin_default_value(name+'_IK.PrimaryAxis.X',arrayInfo["paX"])
		RigVMController.set_pin_default_value(name+'_IK.PrimaryAxis.Y',arrayInfo["paY"])
		RigVMController.set_pin_default_value(name+'_IK.PrimaryAxis.Z',arrayInfo["paZ"])
		RigVMController.set_pin_default_value(name+'_IK.SecondaryAxis.X',arrayInfo["saX"])
		RigVMController.set_pin_default_value(name+'_IK.SecondaryAxis.Y',arrayInfo["paY"])
		RigVMController.set_pin_default_value(name+'_IK.SecondaryAxis.Z',arrayInfo["paZ"])
		RigVMController.set_pin_default_value(name+'_IK.PoleVectorKind','Location')
		RigVMController.set_pin_default_value(name+'_IK.PoleVectorSpace.Type','Control')
		RigVMController.set_pin_default_value(name+'_IK.PoleVectorSpace.Name','Pole'+name)
		RigVMController.set_pin_default_value(name+'_IK.bPropagateToChildren','True')
		RigVMController.add_link(name+'_GT.Transform' , name+'_IK.Effector')
		RigVMController.add_link(endPlug , name+'_IK.ExecuteContext')
		endPlug = name+'_IK.ExecuteContext'

		#GTPole
		GTPole = asAddNode (sp+'GetTransform','Execute',node_name=name+'_GTPole')
		RigVMController.set_node_position (GTPole, [1000, y])
		RigVMController.set_pin_default_value(name+'_GTPole.Item.Type','Control')
		RigVMController.set_pin_default_value(name+'_GTPole.Item.Name','Pole'+name)

		#GTMidJoint
		GTMidJoint = asAddNode (sp+'GetTransform','Execute',node_name=name+'_GTMidJoint')
		RigVMController.set_node_position (GTMidJoint, [1000, y+200])
		RigVMController.set_pin_default_value(name+'_GTMidJoint.Item.Type','Bone')
		RigVMController.set_pin_default_value(name+'_GTMidJoint.Item.Name',arrayInfo["middleJoint"])

		PoleLine = asAddNode (sp+'DebugLineItemSpace','Execute',node_name=name+'_PoleLine')
		RigVMController.set_node_position (PoleLine, [1350, y])
		RigVMController.add_link(name+'_GTPole.Transform.Translation' , name+'_PoleLine.A')
		RigVMController.add_link(name+'_GTMidJoint.Transform.Translation' , name+'_PoleLine.B')
		RigVMController.add_link(endPlug , name+'_PoleLine.ExecuteContext')
		endPlug = name+'_PoleLine.ExecuteContext'

		control_value = hierarchy.make_control_value_from_euler_transform(unreal.EulerTransform(scale=[1, 1, 1]))
		control_settings = unreal.RigControlSettings()
		control_settings.shape_visible = False
		hierarchy_controller.add_control (name+'LS','', control_settings,control_value)
		hierarchy_controller.set_parent(asGetKeyFromName(name+'LS'),asGetKeyFromName(name),maintain_global_transform=False)
		RigVMController.set_pin_default_value(name+'_GT.Item.Name',name+'LS')

		for key in RigElementKeys:
			if (key.name == arrayInfo["endJoint"]):
				endJointKey = key
		EndJointTransform = hierarchy.get_global_transform(endJointKey, initial = False)
		Rotation = EndJointTransform.rotation.rotator()
		Transform = unreal.Transform(location=[0,0,0],rotation=Rotation,scale=[1,1,1])
		hierarchy.set_control_offset_transform(asGetKeyFromName(name+'LS'), Transform, True, True)

		#Backwards solve nodes (IK)
		PNPinvIK = asAddNode (sp+'ProjectTransformToNewParent','Execute',node_name=name+'_PNPinvIK')
		RigVMController.set_node_position (PNPinvIK, [-2900, y])
		RigVMController.set_pin_default_value(name+'_PNPinvIK.Child.Type','Control')
		RigVMController.set_pin_default_value(name+'_PNPinvIK.Child.Name',name)
		RigVMController.set_pin_default_value(name+'_PNPinvIK.OldParent.Type','Control')
		RigVMController.set_pin_default_value(name+'_PNPinvIK.OldParent.Name',name+'LS')
		RigVMController.set_pin_default_value(name+'_PNPinvIK.NewParent.Type','Bone')
		RigVMController.set_pin_default_value(name+'_PNPinvIK.NewParent.Name',joint)

		#STinvIK
		STinvIK = asAddNode (sp+'SetTransform','Execute',node_name=name+'_STinvIK')
		RigVMController.set_node_position (STinvIK, [-2500, y])
		RigVMController.set_pin_default_value(name+'_STinvIK.Item.Type','Control')
		RigVMController.set_pin_default_value(name+'_STinvIK.Item.Name',name)
		RigVMController.set_pin_default_value(name+'_STinvIK.bPropagateToChildren','True')
		RigVMController.add_link(name+'_PNPinvIK.Transform' , name+'_STinvIK.Transform')
		RigVMController.add_link(PreviousEndPlugInv , name+'_STinvIK.ExecuteContext')

		#GTinvPole
		GTinvPole = asAddNode (sp+'GetTransform','Execute',node_name=name+'_GTinvPole')
		RigVMController.set_node_position (GTinvPole, [-1700, y])
		RigVMController.set_pin_default_value(name+'_GTinvPole.Item.Type','Bone')
		RigVMController.set_pin_default_value(name+'_GTinvPole.Item.Name',arrayInfo["middleJoint"])

		#STinvPole
		STinvPole = asAddNode (sp+'SetTransform','Execute',node_name=name+'_STinvPole')
		RigVMController.set_node_position (STinvPole, [-1300, y])
		RigVMController.set_pin_default_value(name+'_STinvPole.Item.Type','Control')
		RigVMController.set_pin_default_value(name+'_STinvPole.Item.Name','Pole'+name)
		RigVMController.set_pin_default_value(name+'_STinvPole.bPropagateToChildren','True')
		RigVMController.add_link(name+'_GTinvPole.Transform' , name+'_STinvPole.Transform')
		RigVMController.add_link(name+'_STinvIK.ExecuteContext' , name+'_STinvPole.ExecuteContext')
		endPlugInv = name+'_STinvPole.ExecuteContext'
		PreviousEndPlugInv = endPlugInv
		PreviousYInv = y

	if "twistJoints" in arrayInfo or "inbetweenJoints" in arrayInfo:
		PreviousArrayInfo = arrayInfo
	else:
		PreviousArrayInfo.clear()

	#DrivingSystem
	if type=='DrivingSystem' or type=='ctrlBox':
		if type=='DrivingSystem':
			RigVMController.set_pin_default_value(name+'_GT.Item.Type','Bone')
			RigVMController.set_pin_default_value(name+'_GT.Item.Name',joint)
			RigVMController.set_pin_default_value(name+'_ST.Item.Type','Control')
			RigVMController.set_pin_default_value(name+'_ST.Item.Name',name)
		if type=='ctrlBox' and name!='ctrlBox':
			parentTransform = hierarchy.get_global_control_offset_transform(asGetKeyFromName(parent))
			Transform = unreal.Transform(location=[offT[0]/ctrlBoxSize,offT[2]/ctrlBoxSize,offT[1]/ctrlBoxSize])
			Transform.rotation = [0,0,0]
			hierarchy.set_control_offset_transform(asGetKeyFromName(name), Transform, True, True)
		if name=='ctrlBox':
			#add a ws oriented ctrlBoxOffset
			NullKey = hierarchy_controller.add_null ('ctrlBoxOffset',asGetKeyFromName(parent),unreal.Transform(),transform_in_global=True)
			Transform = hierarchy.get_local_transform(NullKey)
			Transform.translation = [0,0,0]
			hierarchy.set_local_transform(NullKey, Transform,initial=True)
			hierarchy.set_local_transform(NullKey, Transform,initial=False)
			hierarchy_controller.set_parent(asGetKeyFromName(name),NullKey,maintain_global_transform=False)			
			ctrlBoxSize = float (arrayInfo["ctrlBoxSize"])
			Scale = [0.6,1.5,1.0]
			ctrlBoxScale = [ctrlBoxSize,ctrlBoxSize,ctrlBoxSize]
			parentTransform = hierarchy.get_global_control_offset_transform(asGetKeyFromName(parent))
			Transform2 = hierarchy.get_global_control_offset_transform(asGetKeyFromName(name)).make_relative(parentTransform)
			Transform2.translation = [offT[0],offT[2],offT[1]]
			Transform2.scale3d = [ctrlBoxSize,ctrlBoxSize,ctrlBoxSize]
			hierarchy.set_control_offset_transform(asGetKeyFromName(name), Transform2, True, True)
			Transform = unreal.Transform(location=[0,0,-1],rotation=[0,0,90],scale=Scale)
			hierarchy.set_control_shape_transform(asGetKeyFromName(name), Transform, True, True)
			return
		nonTransformFaceCtrl = False
		if type=='ctrlBox':
			RigVMController.remove_node(RigVMGraph.find_node_by_name(name+'_GT'))
			RigVMController.remove_node(RigVMGraph.find_node_by_name(name+'_ST'))
			Transform = unreal.Transform(scale=[0.05,0.05,0.05])
			hierarchy.set_control_shape_transform(asGetKeyFromName(name), Transform, True, True)

			maxXform = unreal.Vector2D(1,1)
			minXform = unreal.Vector2D(-1,-1)
			if name=='ctrlMouth_M':
					maxXform = [1,0]
			if re.search("^ctrlCheek_", name) or re.search("^ctrlNose_", name):
					minXform = [-1,0]
			RigElementKeys = asGetRigElementKeys ()
			for key in RigElementKeys:
				if (key.name == name):
					RigControlKey = key
			hierarchy.set_control_value(asGetKeyFromName(name),unreal.RigHierarchy.make_control_value_from_vector2d(maxXform), unreal.RigControlValueType.MAXIMUM)
			hierarchy.set_control_value(asGetKeyFromName(name),unreal.RigHierarchy.make_control_value_from_vector2d(minXform), unreal.RigControlValueType.MINIMUM)
			endPlug = PreviousEndPlug

		control_settings = unreal.RigControlSettings()
		control_value = hierarchy.make_control_value_from_euler_transform(unreal.EulerTransform(scale=[1, 1, 1]))
		AttrGrpkey = hierarchy_controller.add_control (name+"_Attributes",'',control_settings,control_value)
		if type=='ctrlBox':
			hierarchy_controller.set_parent(asGetKeyFromName(name+"_Attributes"),asGetKeyFromName(parent),maintain_global_transform=False)
		else:
			hierarchy_controller.set_parent(asGetKeyFromName(name+"_Attributes"),asGetKeyFromName(name),maintain_global_transform=False)
		hierarchy.set_control_shape_transform(asGetKeyFromName(name+"_Attributes"), unreal.Transform(location=[0,0,0],rotation=[0,0,0],scale=[0,0,0]), True)

		Transform = hierarchy.get_global_control_offset_transform(asGetKeyFromName(name), initial = True).copy()
		parentTransform = hierarchy.get_global_transform(asGetKeyFromName(parent), initial = True)
		Transform = Transform.make_relative(parentTransform)
		if type=='ctrlBox':
			Transform.translation.x = -5.0
			Transform.translation.z += 0.8
			if re.search("_L", name) or re.search("_M", name):
				Transform.translation.x = 4.0
			if nonTransformFaceCtrl:
				Transform.translation.z = -5.5-(nonTransformFaceCtrlNr*2) # stack rows of sliders downwards
		else:
			numAttrs = 0
			for Attr in arrayInfo.keys():
				if not re.search("-set", Attr):
					numAttrs=numAttrs+1
			Transform = unreal.Transform(location=[0,0,0],rotation=[0,0,0],scale=[1,1,1])
			Transform.translation.x = offT[0]
			Transform.translation.y = numAttrs*0.5*(size/4.0)*-0.5
			Transform.translation.z = size/8.0
			if re.search("_L", name):
					Transform.translation.z *= -1
			Transform.scale3d=[size/4.0,size/4.0,size/4.0]
		hierarchy.set_control_offset_transform(asGetKeyFromName(name+"_Attributes"), Transform, True, True)

		Attrs = arrayInfo.keys()
		attrNr = 0
		for Attr in Attrs:
			if re.search("-set", Attr):
				if re.search("-setLimits", Attr):
					DictDrivens = arrayInfo.get(Attr)
					min = float(list(DictDrivens.keys())[0])
					max = float(list(DictDrivens.values())[0])
					RigElementKeys = asGetRigElementKeys ()
					for key in RigElementKeys:
						if key.name == name+"_"+Attr.replace("-setLimits", ""):
								hierarchy.set_control_value(key, unreal.RigHierarchy.make_control_value_from_float(min), unreal.RigControlValueType.MINIMUM)
								hierarchy.set_control_value(key, unreal.RigHierarchy.make_control_value_from_float(max), unreal.RigControlValueType.MAXIMUM)
					continue

			transformAttrDriver = True
			if not re.search("translate", Attr) or re.search("rotate", Attr) or re.search("scale", Attr):
				control_settings = unreal.RigControlSettings()
				control_settings.control_type = unreal.RigControlType.FLOAT
				control_settings.shape_color = unreal.LinearColor(r=1.0, g=0.0, b=0.0, a=1.0)
				control_settings.limit_enabled = [unreal.RigControlLimitEnabled(True, True)]
				if nonTransformFaceCtrl or re.search("_M", name):
					control_settings.primary_axis = unreal.RigControlAxis.Z
				else:
					control_settings.primary_axis = unreal.RigControlAxis.X
				key = hierarchy_controller.add_control (name+"_"+Attr,asGetKeyFromName(name+"_Attributes"),control_settings,control_value)
				
				hierarchy.set_control_value(key,unreal.RigHierarchy.make_control_value_from_float(1), unreal.RigControlValueType.MAXIMUM)
				hierarchy.set_control_shape_transform(asGetKeyFromName(name+"_"+Attr), unreal.Transform(location=[0,0,0],rotation=[0,0,0],scale=[0.035,0.035,0.035]), True, True)

				Transform = unreal.Transform(location=[0,0,0],rotation=[0,0,0],scale=[1,1,1])
				Transform.translation = [0,attrNr*0.5,0]
				if type=='ctrlBox':
					Transform.translation = [0,0,attrNr*-0.5]
				if nonTransformFaceCtrl or re.search("_M", name):
					Transform.translation = [attrNr,0,0]
				attrNr = attrNr+1
				hierarchy.set_control_offset_transform(asGetKeyFromName(name+"_"+Attr), Transform, True, True)
				transformAttrDriver = False
			DictDrivens = arrayInfo.get(Attr)
			KeysDrivens = DictDrivens.keys()

			for Driven in KeysDrivens:
				Value = float(DictDrivens.get(Driven))
				x2 = ASDrivenNr*1200
				dNr = str(ASDrivenNr)
				ASDrivenNr = ASDrivenNr+1
				x = Driven.split(".")
				obj = x[0]
				attr = '_'+x[1]
				axis = attr[-1]
				valueMult = 1
				if re.search("rotate", attr):
					if axis == 'X' or axis=='Z':
						valueMult = -1
				if re.search("translate", attr):
					if axis=='Y':
						valueMult = -1
				multiplier = Value*valueMult

				asFaceBSDriven = False
				if re.search("asFaceBS[.]", Driven):#asFaceBS
					asFaceBSDriven = True

				if not (asFaceBSDriven):
					RigElementKeys = asGetRigElementKeys ()
					for key in RigElementKeys:
						if key.name == obj:
							objKey = key
					if not asObjExists ('Offset'+obj):
						objParentKey = hierarchy.get_parents(objKey)[0]
						OffsetKey = hierarchy_controller.add_null ('Offset'+obj,objKey,unreal.Transform(),transform_in_global=False)
						hierarchy_controller.set_parent(OffsetKey,objParentKey)
						hierarchy_controller.set_parent(objKey,OffsetKey)
						parentTo = 'Offset'+obj

					for x in range(1,9):
						sdk = 'SDK'+obj+"_"+str(x)
						if not asObjExists (sdk):
							break
					if x>1:
						parentTo = 'SDK'+obj+"_"+str(x-1)

					SDKKey = hierarchy_controller.add_null (sdk,asGetKeyFromName(parentTo),unreal.Transform(),transform_in_global=False)
					hierarchy_controller.set_parent(objKey,SDKKey)

				#GTDriver
				if transformAttrDriver:
					GTDriver = asAddNode (sp+'GetControlVector2D','Execute',node_name=name+"_"+obj+"_"+attr+dNr+'_GTDriver')
					RigVMController.set_pin_default_value(name+"_"+obj+"_"+attr+dNr+'_GTDriver.Control',name)
					gtPlug = name+"_"+obj+"_"+attr+dNr+'_GTDriver.Vector.'+Attr[-1]#Attr[-1] is DriverAxis
				else:
					GTDriver = asAddNode (sp+'GetControlFloat','Execute',node_name=name+"_"+obj+"_"+attr+dNr+'_GTDriver')
					RigVMController.set_pin_default_value(name+"_"+obj+"_"+attr+dNr+'_GTDriver.Control',name+"_"+Attr)
					gtPlug = name+"_"+obj+"_"+attr+dNr+'_GTDriver.FloatValue'
				RigVMController.set_node_position (GTDriver, [500+x2, y])

				#MFM
				MFM = asAddNode (sp+'MathFloatMul','Execute',node_name=name+"_"+obj+"_"+attr+dNr+'_MFM')
				RigVMController.set_node_position (MFM, [900+x2, y])
				RigVMController.add_link(gtPlug , name+"_"+obj+"_"+attr+dNr+'_MFM.A')
				RigVMController.set_pin_default_value(name+"_"+obj+"_"+attr+dNr+'_MFM.B',str(multiplier))

				if asFaceBSDriven:
					#Clamp
					Clamp = asAddNode (sp+'MathFloatClamp','Execute',node_name=name+"_"+obj+"_"+attr+dNr+'_Clamp')
					RigVMController.set_node_position (Clamp, [900+x2, y+100])
					RigVMController.set_pin_default_value(name+"_"+obj+"_"+attr+dNr+'_Clamp.Maximum','5.0')
					RigVMController.add_link(name+"_"+obj+"_"+attr+dNr+'_MFM.Result' , name+"_"+obj+"_"+attr+dNr+'_Clamp.Value')
					#STDriven
					STDriven = asAddNode (sp+'SetCurveValue','Execute',node_name=name+"_"+Attr+"_"+attr+'_STDriven')
					RigVMController.set_node_position (STDriven, [1100+x2, y])
					RigVMController.set_pin_default_value(name+"_"+Attr+"_"+attr+'_STDriven.Curve',Driven.split(".")[1])
					RigVMController.add_link(name+"_"+obj+"_"+attr+dNr+'_Clamp.Result' , name+"_"+Attr+"_"+attr+'_STDriven.Value')
					RigVMController.add_link(endPlug , name+"_"+Attr+"_"+attr+'_STDriven.ExecuteContext')
					endPlug =name+"_"+Attr+"_"+attr+'_STDriven.ExecuteContext'
				else:
					#STDriven
					STDriven = asAddNode (sp+'SetTransform','Execute',node_name=name+"_"+obj+"_"+attr+dNr+'_STDriven')
					RigVMController.set_node_position (STDriven, [1300+x2, y])
					RigVMController.set_pin_default_value(name+"_"+obj+"_"+attr+dNr+'_STDriven.Item.Type','Null')
					RigVMController.set_pin_default_value(name+"_"+obj+"_"+attr+dNr+'_STDriven.Item.Name',sdk)
					RigVMController.set_pin_default_value(name+"_"+obj+"_"+attr+dNr+'_STDriven.Space','LocalSpace')
					RigVMController.set_pin_default_value(name+"_"+obj+"_"+attr+dNr+'_STDriven.bPropagateToChildren','True')
					RigVMController.add_link(endPlug , name+"_"+obj+"_"+attr+dNr+'_STDriven.ExecuteContext')
					endPlug = name+"_"+obj+"_"+attr+dNr+'_STDriven.ExecuteContext'

					#TFSRT
					TFSRT = asAddNode (sp+'MathTransformFromSRT','Execute',node_name=name+"_"+obj+"_"+attr+dNr+'_TFSRT')
					RigVMController.set_node_position (TFSRT, [900+x2, y+150])
					if re.search("translate", attr):
						RigVMController.add_link(name+"_"+obj+"_"+attr+dNr+'_MFM.Result' , name+"_"+obj+"_"+attr+dNr+'_TFSRT.Location.'+axis)
					if re.search("rotate", attr):
						RigVMController.add_link(name+"_"+obj+"_"+attr+dNr+'_MFM.Result' , name+"_"+obj+"_"+attr+dNr+'_TFSRT.Rotation.'+axis)
					if re.search("scale", attr):
						#scale just add 1, not accurate but simplified workaround
						MFA = asAddNode (sp+'MathFloatAdd','Execute',node_name=name+"_"+obj+"_"+attr+dNr+'_MFA')
						RigVMController.set_node_position (MFA, [1100+x2, y])
						RigVMController.add_link(name+"_"+obj+"_"+attr+dNr+'_MFM.Result' , name+"_"+obj+"_"+attr+dNr+'_MFA.A')
						RigVMController.set_pin_default_value(name+"_"+obj+"_"+attr+dNr+'_MFA.B','1')
						RigVMController.add_link(name+"_"+obj+"_"+attr+dNr+'_MFA.Result' , name+"_"+obj+"_"+attr+dNr+'_TFSRT.Scale.'+axis)
					RigVMController.add_link(name+"_"+obj+"_"+attr+dNr+'_TFSRT.Transform', name+"_"+obj+"_"+attr+dNr+'_STDriven.Transform')

	#face
	if re.search("Teeth_M", name):
		RigControl.set_editor_property('gizmo_enabled',False)
		hierarchy_controller.set_control (RigControl)
	if name=="Jaw_M":
		RigControl.set_editor_property('gizmo_name','HalfCircle_Thick')
		RigControl.set_editor_property('gizmo_name','HalfCircle_Thick')
		Transform = RigControl.get_editor_property('gizmo_transform')
		Transform.rotation=[0,0,180]
		RigControl.set_editor_property('gizmo_transform', Transform)
		hierarchy_controller.set_control (RigControl)	

	PreviousEndPlug = endPlug

def asObjExists (obj):
	RigElementKeys = asGetRigElementKeys ()
	LocObject = None
	for key in RigElementKeys:
		if key.name == obj:
			return True
	return False

def asAddController (name, parent, joint, type, gizmoName, ws, size, offT, color):
	parentKey = asGetKeyFromName(parent)
	if gizmoName=='Gizmo':
		gizmoName='Default'

	control_settings = unreal.RigControlSettings()
	if type=='ctrlBox':
		control_settings.control_type = unreal.RigControlType.VECTOR2D
		control_settings.primary_axis = unreal.RigControlAxis.Y
		control_settings.limit_enabled = [unreal.RigControlLimitEnabled(True, True), unreal.RigControlLimitEnabled(True, True)]
	else:
		control_settings.control_type = unreal.RigControlType.EULER_TRANSFORM
	if name=='ctrlEmotions_M' or name=='ctrlPhonemes_M' or name=='ctrlARKit_M' or name=='ctrlBoxRobloxHead_M':
			control_settings.shape_visible = False
	control_settings.shape_name = gizmoName
	control_settings.shape_color = unreal.LinearColor(color[0], color[1], color[2], 1.0)
	control_value = hierarchy.make_control_value_from_euler_transform(unreal.EulerTransform(scale=[1, 1, 1]))
	key = hierarchy_controller.add_control (name,'', control_settings,control_value)
	jointKey = asGetKeyFromName (joint)
	jointTransform = hierarchy.get_global_transform(jointKey, initial = True)
	if ws == 1:
		jointTransform.rotation = [0,0,0]
	parentTransform = unreal.Transform()
	if parent!='':
		parentTransform = hierarchy.get_global_transform(asGetKeyFromName(parent), initial = True)
	OffsetTransform = jointTransform.make_relative(parentTransform)
	if parent!='':
		hierarchy_controller.set_parent(asGetKeyFromName(name),asGetKeyFromName(parent),maintain_global_transform=False)
	hierarchy.set_control_offset_transform(key, OffsetTransform, True, True)
	GizmoLocation = [offT[0],offT[2],offT[1]]
	GizmoRotation = [0,0,0]
	if ws == 0:
		GizmoRotation = [90,0,0]
		GizmoLocation = [offT[0],offT[1]*-1,offT[2]]
	if type=="DrivingSystem":
		GizmoRotation = [0,0,0]
	if type=="ctrlBox":
		GizmoRotation = [0,0,90]
	if re.search("^Eye_.*", name) or re.search("^Iris_.*", name) or re.search("^Pupil_.*", name):
		GizmoRotation = [0,90,0]
		hierarchy.set_control_visibility(key,False)
	x = re.search("^Pole.*", name)
	if x:
		GizmoLocation = [0,0,0]
	s = 0.1*size
	Scale = [s,s,s]
	if type=='FK' and ws == 0:
		Scale[2]*=2.5
	hierarchy.set_control_shape_transform(key, unreal.Transform(location=GizmoLocation,rotation=GizmoRotation,scale=Scale), True)

def asAddNode (script_struct_path, method_name, node_name):
	#RigVMController.
	#add_struct_node_from_struct_path UE4
	#add_unit_node_from_struct_path UE5
	try:
		node = RigVMController.add_struct_node_from_struct_path(script_struct_path,method_name,node_name=node_name) #UE4
	except:
		node = RigVMController.add_unit_node_from_struct_path(script_struct_path,method_name,node_name=node_name) #UE5
	return node

def asGetRigElementKeys ():
	try:
		RigElementKeys = hierarchy_controller.get_elements() #UE4
	except:
		RigElementKeys = hierarchy.get_all_keys() #UE5
	return RigElementKeys

def asGetKeyFromName (name):
	all_keys = hierarchy.get_all_keys(traverse = True)
	for key in all_keys:
		if key.name == name:
			return key
	return ''

def asBackwardsSolveNodes ():
	global PreviousYInv
	global PreviousEndPlugInv
	PNP = asAddNode (sp+'ProjectTransformToNewParent','Execute',node_name='Root_M_PNP')
	RigVMController.set_node_position (PNP, [-1500, PreviousYInv+400-90])
	RigVMController.set_pin_default_value('Root_M_PNP.Child.Type','Control')
	RigVMController.set_pin_default_value('Root_M_PNP.Child.Name','RootX_M')
	RigVMController.set_pin_default_value('Root_M_PNP.OldParent.Type','Bone')
	RigVMController.set_pin_default_value('Root_M_PNP.OldParent.Name','Root_M')
	RigVMController.set_pin_default_value('Root_M_PNP.NewParent.Type','Bone')
	RigVMController.set_pin_default_value('Root_M_PNP.NewParent.Name','Root_M')

	STInv = asAddNode (sp+'SetTransform','Execute',node_name='Root_M_STInv')
	RigVMController.set_node_position (STInv, [-1200, PreviousYInv+400-90])
	RigVMController.set_pin_default_value('Root_M_STInv.Item.Type','Control')
	RigVMController.set_pin_default_value('Root_M_STInv.Item.Name','RootX_M')
	RigVMController.add_link('Root_M_PNP.Transform' , 'Root_M_STInv.Transform')
	RigVMController.add_link(PreviousEndPlugInv , 'Root_M_STInv.ExecuteContext')

	CCinv = asAddNode (sp+'CollectionChildren','Execute',node_name='CCinv')
	RigVMController.set_node_position (CCinv, [-2600, PreviousYInv+1000])
	RigVMController.set_pin_default_value('CCinv.Parent.Type','Bone')
	RigVMController.set_pin_default_value('CCinv.Parent.Name','Root_M')
	RigVMController.set_pin_default_value('CCinv.bRecursive','True')
	RigVMController.set_pin_default_value('CCinv.TypeToSearch','Bone')

	CLinv = asAddNode (sp+'CollectionLoop','Execute',node_name='CLinv')
	RigVMController.set_node_position (CLinv, [-2150, PreviousYInv+1000])
	RigVMController.add_link('Root_M_STInv.ExecuteContext' , 'CLinv.ExecuteContext')
	RigVMController.add_link('CCinv.Collection' , 'CLinv.Collection')
	PreviousEndPlugInv = 'CLinv.Completed'

	NCinv = asAddNode (sp+'NameConcat','Execute',node_name='NCinv')
	RigVMController.set_node_position (NCinv, [-1900, PreviousYInv+900])
	RigVMController.set_pin_default_value('NCinv.A','FK')
	RigVMController.add_link('CLinv.Item.Name' , 'NCinv.B')

	GTinv = asAddNode (sp+'GetTransform','Execute',node_name='GTinv')
	RigVMController.set_node_position (GTinv, [-1900, PreviousYInv+1000])
	RigVMController.add_link('CLinv.Item.Name' , 'GTinv.Item.Name')

	IEinv = asAddNode (sp+'ItemExists','Execute',node_name='IEinv')
	RigVMController.set_node_position (IEinv, [-1700, PreviousYInv+700])
	RigVMController.set_pin_default_value('IEinv.Item.Type','Control')
	RigVMController.add_link('NCinv.Result' , 'IEinv.Item.Name')

	BRinv = RigVMController.add_branch_node(node_name='BRinv')
	RigVMController.set_node_position (BRinv, [-1650, PreviousYInv+850])
	RigVMController.add_link('IEinv.Exists' , 'BRinv.Condition')
	RigVMController.add_link('CLinv.ExecuteContext' , 'BRinv.ExecuteContext')

	STinv = asAddNode (sp+'SetTransform','Execute',node_name='STinv')
	RigVMController.set_node_position (STinv, [-1500, PreviousYInv+1000])
	RigVMController.set_pin_default_value('STinv.Item.Type','Control')
	RigVMController.add_link('NCinv.Result' , 'STinv.Item.Name')
	RigVMController.add_link('GTinv.Transform' , 'STinv.Transform')
	RigVMController.add_link('BRinv.True' , 'STinv.ExecuteContext')

def main ():
	global PreviousEndPlugInv
	RigElementKeys = asGetRigElementKeys ()
	RigVMGraph = blueprint.get_model()

	#Clear out existing rig-setup
	nodes = RigVMGraph.get_nodes()
	for node in nodes:
		RigVMController.remove_node(node)
	#Clear out existing controllers
	for key in RigElementKeys:
		if key.name == 'MotionSystem':
			hierarchy_controller.remove_element(key)
		elif not (key.type==1 or key.type==8): #BONE
			try:
				hierarchy_controller.remove_element(key)
			except:
				pass
	#UE5 does not include deleting children, so we will try to clean-up
	controls = hierarchy.get_controls()
	for key in controls:
		hierarchy_controller.remove_element(key)
	nulls = hierarchy.get_nulls()
	for key in nulls:
		hierarchy_controller.remove_element(key)
	bones = hierarchy.get_bones()
	for key in bones:
		x = re.search("UnTwist", str(key.name))
		if x:
			hierarchy_controller.remove_element(key)

	BeginExecutionNode = asAddNode (sp+'BeginExecution','Execute',node_name='RigUnit_BeginExecution')
	RigVMController.set_node_position (BeginExecutionNode, [-300, -100])
	InverseExecutionNode = asAddNode (sp+'InverseExecution','Execute',node_name='RigUnit_InverseExecution')
	RigVMController.set_node_position (InverseExecutionNode, [-1900, -100])

	MotionSystemKey = hierarchy_controller.add_null ('MotionSystem','',unreal.Transform())
	MainSystemKey = hierarchy_controller.add_null ('MainSystem',MotionSystemKey,unreal.Transform())
	DrivingSystemKey = hierarchy_controller.add_null ('DrivingSystem',MotionSystemKey ,unreal.Transform())

	#//-- ASControllers Starts Here --//
	asAddCtrl ('Main','MainSystem','DeformationSystem','Main',{},'Circle_Thick',1,45.3477697,[0, 0, 0],[0, 0.2549999952, 0.6000000238, 1.0])
	asAddCtrl ('RootX_M','Main','Root_M','FK',{},'Arrow4_Thin',1,36.10997164,[0, 0, 0],[0, 1, 0, 1.0])
	asAddCtrl ('FKRoot_M','RootX_M','Root_M','FK',{'numMainExtras':'0','centerBtwFeet':'0'},'Circle_Thick',0,25.05464363,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKSpine1_M','FKRoot_M','Spine1_M','FK',{},'Circle_Thick',0,25.05464363,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKSpine2_M','FKSpine1_M','Spine2_M','FK',{},'Circle_Thick',0,25.05464363,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKChest_M','FKSpine2_M','Chest_M','FK',{},'Circle_Thick',0,25.05464363,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKBreast_L','FKChest_M','Breast_L','FK',{},'Circle_Thick',0,1.055594325,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKScapula_L','FKChest_M','Scapula_L','FK',{},'Circle_Thick',0,13.29913616,[0, 0, -3.679451227],[1, 1, 0.3880000114, 1.0])
	asAddCtrl ('FKShoulder_L','FKScapula_L','Shoulder_L','FK',{'twistJoints':'2','bendyJoints':'0'},'Circle_Thick',0,14.73802471,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKElbow_L','FKShoulder_L','Elbow_L','FK',{'twistJoints':'2','bendyJoints':'0'},'Circle_Thick',0,10.20324802,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKWrist_L','FKElbow_L','Wrist_L','FK',{},'Circle_Thick',0,8.904034615,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKCup_L','FKWrist_L','Cup_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKRingFinger1_L','FKCup_L','RingFinger1_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKRingFinger2_L','FKRingFinger1_L','RingFinger2_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKRingFinger3_L','FKRingFinger2_L','RingFinger3_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKPinkyFinger1_L','FKCup_L','PinkyFinger1_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKPinkyFinger2_L','FKPinkyFinger1_L','PinkyFinger2_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKPinkyFinger3_L','FKPinkyFinger2_L','PinkyFinger3_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKIndexFinger1_L','FKWrist_L','IndexFinger1_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKIndexFinger2_L','FKIndexFinger1_L','IndexFinger2_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKIndexFinger3_L','FKIndexFinger2_L','IndexFinger3_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKThumbFinger1_L','FKWrist_L','ThumbFinger1_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKThumbFinger2_L','FKThumbFinger1_L','ThumbFinger2_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKThumbFinger3_L','FKThumbFinger2_L','ThumbFinger3_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKMiddleFinger1_L','FKWrist_L','MiddleFinger1_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKMiddleFinger2_L','FKMiddleFinger1_L','MiddleFinger2_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKMiddleFinger3_L','FKMiddleFinger2_L','MiddleFinger3_L','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKBreast_R','FKChest_M','Breast_R','FK',{},'Circle_Thick',0,1.055594325,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKScapula_R','FKChest_M','Scapula_R','FK',{},'Circle_Thick',0,13.29913616,[0, 0, 3.679451227],[1, 1, 0.3880000114, 1.0])
	asAddCtrl ('FKShoulder_R','FKScapula_R','Shoulder_R','FK',{'twistJoints':'2','bendyJoints':'0'},'Circle_Thick',0,14.73802471,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKElbow_R','FKShoulder_R','Elbow_R','FK',{'twistJoints':'2','bendyJoints':'0'},'Circle_Thick',0,10.20324802,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKWrist_R','FKElbow_R','Wrist_R','FK',{},'Circle_Thick',0,8.904034615,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKCup_R','FKWrist_R','Cup_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKRingFinger1_R','FKCup_R','RingFinger1_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKRingFinger2_R','FKRingFinger1_R','RingFinger2_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKRingFinger3_R','FKRingFinger2_R','RingFinger3_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKPinkyFinger1_R','FKCup_R','PinkyFinger1_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKPinkyFinger2_R','FKPinkyFinger1_R','PinkyFinger2_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKPinkyFinger3_R','FKPinkyFinger2_R','PinkyFinger3_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKIndexFinger1_R','FKWrist_R','IndexFinger1_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKIndexFinger2_R','FKIndexFinger1_R','IndexFinger2_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKIndexFinger3_R','FKIndexFinger2_R','IndexFinger3_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKThumbFinger1_R','FKWrist_R','ThumbFinger1_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKThumbFinger2_R','FKThumbFinger1_R','ThumbFinger2_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKThumbFinger3_R','FKThumbFinger2_R','ThumbFinger3_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKMiddleFinger1_R','FKWrist_R','MiddleFinger1_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKMiddleFinger2_R','FKMiddleFinger1_R','MiddleFinger2_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKMiddleFinger3_R','FKMiddleFinger2_R','MiddleFinger3_R','FK',{},'Circle_Thick',0,2.720866203,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKNeck_M','FKChest_M','Neck_M','FK',{'inbetweenJoints':'2','unTwister':'1'},'Circle_Thick',0,7.255643368,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKHead_M','FKNeck_M','Head_M','FK',{'global':'0'},'Circle_Thick',0,14.51128674,[10.24168041, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('IKLeg_L','Main','Ankle_L','IK',{'startJoint':'Hip_L','middleJoint':'Knee_L','endJoint':'Ankle_L','paX':'-1','paY':'-0','paZ':'-0','saX':'0','saY':'1','saZ':'0','ppX':'5.807764231','ppY':'36.35763271','ppZ':'53.45712303'},'Box_Thick',1,14.12760845,[0.6875393239, -2.387104361, 2.375574212],[1, 0, 0, 1.0])
	asAddCtrl ('FKToes_L','IKLeg_L','Toes_L','FK',{},'Circle_Thick',0,6.802165508,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKPinkyToe1_L','FKToes_L','PinkyToe1_L','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKRingToe1_L','FKToes_L','RingToe1_L','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKMidToe1_L','FKToes_L','MidToe1_L','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKIndexToe1_L','FKToes_L','IndexToe1_L','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKBigToe1_L','FKToes_L','BigToe1_L','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('IKLeg_R','Main','Ankle_R','IK',{'startJoint':'Hip_R','middleJoint':'Knee_R','endJoint':'Ankle_R','paX':'1','paY':'0','paZ':'0','saX':'-0','saY':'-1','saZ':'-0','ppX':'-5.807764231','ppY':'36.35763268','ppZ':'53.45712301'},'Box_Thick',1,14.12760845,[-0.6875393233, -2.387104361, 2.375574209],[1, 0, 0, 1.0])
	asAddCtrl ('FKToes_R','IKLeg_R','Toes_R','FK',{},'Circle_Thick',0,6.802165508,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKPinkyToe1_R','FKToes_R','PinkyToe1_R','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKRingToe1_R','FKToes_R','RingToe1_R','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKMidToe1_R','FKToes_R','MidToe1_R','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKIndexToe1_R','FKToes_R','IndexToe1_R','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('FKBigToe1_R','FKToes_R','BigToe1_R','FK',{},'Circle_Thick',0,0.8502706885,[0, 0, 0],[0.3919999897, 0.8629999757, 1, 1.0])
	asAddCtrl ('Fingers_L','Main','Wrist_L','DrivingSystem',{'spread':{'FKIndexFinger1_L.rotateZ':'40','FKPinkyFinger1_L.rotateZ':'-60','FKRingFinger1_L.rotateZ':'-30'},'spread-setLimits':{'-0.5':'1'},'cup':{'FKCup_L.rotateX':'65'},'indexCurl':{'FKIndexFinger1_L.rotateY':'90','FKIndexFinger2_L.rotateY':'90','FKIndexFinger3_L.rotateY':'90'},'indexCurl-setLimits':{'-0.2':'1'},'middleCurl':{'FKMiddleFinger1_L.rotateY':'90','FKMiddleFinger2_L.rotateY':'90','FKMiddleFinger3_L.rotateY':'90'},'middleCurl-setLimits':{'-0.2':'1'},'ringCurl':{'FKRingFinger1_L.rotateY':'90','FKRingFinger2_L.rotateY':'90','FKRingFinger3_L.rotateY':'90'},'ringCurl-setLimits':{'-0.2':'1'},'pinkyCurl':{'FKPinkyFinger1_L.rotateY':'90','FKPinkyFinger2_L.rotateY':'90','FKPinkyFinger3_L.rotateY':'90'},'pinkyCurl-setLimits':{'-0.2':'1'},'thumbCurl':{'FKThumbFinger2_L.rotateY':'90','FKThumbFinger3_L.rotateY':'90'},'thumbCurl-setLimits':{'-0.2':'1'}},'Circle_Thick',0,12.42077414,[-6.21038707, -0, -0],[1, 1, 0, 1.0])
	asAddCtrl ('Fingers_R','Main','Wrist_R','DrivingSystem',{'spread':{'FKIndexFinger1_R.rotateZ':'40','FKPinkyFinger1_R.rotateZ':'-60','FKRingFinger1_R.rotateZ':'-30'},'spread-setLimits':{'-0.5':'1'},'cup':{'FKCup_R.rotateX':'65'},'indexCurl':{'FKIndexFinger1_R.rotateY':'90','FKIndexFinger2_R.rotateY':'90','FKIndexFinger3_R.rotateY':'90'},'indexCurl-setLimits':{'-0.2':'1'},'middleCurl':{'FKMiddleFinger1_R.rotateY':'90','FKMiddleFinger2_R.rotateY':'90','FKMiddleFinger3_R.rotateY':'90'},'middleCurl-setLimits':{'-0.2':'1'},'ringCurl':{'FKRingFinger1_R.rotateY':'90','FKRingFinger2_R.rotateY':'90','FKRingFinger3_R.rotateY':'90'},'ringCurl-setLimits':{'-0.2':'1'},'pinkyCurl':{'FKPinkyFinger1_R.rotateY':'90','FKPinkyFinger2_R.rotateY':'90','FKPinkyFinger3_R.rotateY':'90'},'pinkyCurl-setLimits':{'-0.2':'1'},'thumbCurl':{'FKThumbFinger2_R.rotateY':'90','FKThumbFinger3_R.rotateY':'90'},'thumbCurl-setLimits':{'-0.2':'1'}},'Circle_Thick',0,12.42077414,[6.21038707, 0, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlBox','FKHead_M','Head_M','ctrlBox',{'ctrlBoxSize':'0.7710663478'},'Square_Thick',1,11.10335541,[10.67347952, 3.274693809, 6.162718498],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlBrow_R','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.brow_innerRaiser_R':'1','asFaceBS.brow_innerlower_R':'-1'},'translateY':{'asFaceBS.brow_raiser_R':'1','asFaceBS.brow_lower_R':'-1'},'squeeze':{'asFaceBS.brow_squeeze_R':'1'},'outerUpDown':{'asFaceBS.brow_outerUpDown_R':'1'}},'Gizmo',1,0.4361809946,[-1.156599522, 3.469798565, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlBrow_L','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.brow_innerRaiser_L':'1','asFaceBS.brow_innerlower_L':'-1'},'translateY':{'asFaceBS.brow_raiser_L':'1','asFaceBS.brow_lower_L':'-1'},'squeeze':{'asFaceBS.brow_squeeze_L':'1'},'outerUpDown':{'asFaceBS.brow_outerUpDown_L':'1'}},'Gizmo',1,0.4361809946,[1.156599522, 3.469798565, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlEye_R','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'Eye_R.rotateY':'30','asFaceBS.eye_left_R':'1','asFaceBS.eye_right_R':'-1'},'translateY':{'Eye_R.rotateX':'-30','asFaceBS.eye_up_R':'1','asFaceBS.eye_down_R':'-1'},'wide':{'asFaceBS.ctrlEyewide_R':'1'},'blink':{'asFaceBS.blink_R':'1'},'squint':{'asFaceBS.squint_R':'1'}},'Gizmo',1,0.4361809946,[-1.156599522, 1.758031249, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlEye_L','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'Eye_L.rotateY':'30','asFaceBS.eye_left_L':'1','asFaceBS.eye_right_L':'-1'},'translateY':{'Eye_L.rotateX':'-30','asFaceBS.eye_up_L':'1','asFaceBS.eye_down_L':'-1'},'wide':{'asFaceBS.ctrlEyewide_L':'1'},'blink':{'asFaceBS.blink_L':'1'},'squint':{'asFaceBS.squint_L':'1'}},'Gizmo',1,0.4361809946,[1.156599522, 1.758031249, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlCheek_R','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.cheek_out_R':'1','asFaceBS.cheek_in_R':'-1'},'translateY':{'asFaceBS.cheek_raiser_R':'1'}},'Gizmo',1,0.4361809946,[-1.156599522, 0, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlCheek_L','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.cheek_out_L':'1','asFaceBS.cheek_in_L':'-1'},'translateY':{'asFaceBS.cheek_raiser_L':'1'}},'Gizmo',1,0.4361809946,[1.156599522, 0, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlNose_R','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.nose_wide_R':'1','asFaceBS.nose_narrow_R':'-1'},'translateY':{'asFaceBS.nose_raiser_R':'1'}},'Gizmo',1,0.4361809946,[-1.156599522, -1.017807603, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlNose_L','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.nose_wide_L':'1','asFaceBS.nose_narrow_L':'-1'},'translateY':{'asFaceBS.nose_raiser_L':'1'}},'Gizmo',1,0.4361809946,[1.156599522, -1.017807603, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlLips_M','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.lip_left_M':'1','asFaceBS.lip_right_M':'-1'},'translateY':{'asFaceBS.lip_up_M':'1','asFaceBS.lip_down_M':'-1'},'upperPress':{'asFaceBS.lip_upperPressPos_M':'1','asFaceBS.lip_upperPressNeg_M':'-1'},'upperPress-setLimits':{'-1':'1'},'lowerPress':{'asFaceBS.lip_lowerPressPos_M':'1','asFaceBS.lip_lowerPressNeg_M':'-1'},'lowerPress-setLimits':{'-1':'1'},'upperSqueeze':{'asFaceBS.lip_upperSqueezePos_M':'1','asFaceBS.lip_upperSqueezeNeg_M':'-1'},'upperSqueeze-setLimits':{'-1':'1'},'lowerSqueeze':{'asFaceBS.lip_lowerSqueezePos_M':'1','asFaceBS.lip_lowerSqueezeNeg_M':'-1'},'lowerSqueeze-setLimits':{'-1':'1'},'upperRoll':{'asFaceBS.lip_upperRollPos_M':'1','asFaceBS.lip_upperRollNeg_M':'-1'},'upperRoll-setLimits':{'-1':'1'},'lowerRoll':{'asFaceBS.lip_lowerRollPos_M':'1','asFaceBS.lip_lowerRollNeg_M':'-1'},'lowerRoll-setLimits':{'-1':'1'},'upperPucker':{'asFaceBS.lip_upperPuckerPos_M':'1','asFaceBS.lip_upperPuckerNeg_M':'-1'},'upperPucker-setLimits':{'-1':'1'},'lowerPucker':{'asFaceBS.lip_lowerPuckerPos_M':'1','asFaceBS.lip_lowerPuckerNeg_M':'-1'},'lowerPucker-setLimits':{'-1':'1'}},'Gizmo',1,0.4361809946,[0, -1.94308722, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlMouth_M','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.mouth_wide_M':'1','asFaceBS.mouth_narrow_M':'-1'},'translateY':{'asFaceBS.mouth_close_M':'1','asFaceBS.mouth_open_M':'-1'},'jawForward':{'asFaceBS.mouth_jawForwardIn_M':'1','asFaceBS.mouth_jawForwardOut_M':'-1'},'jawForward-setLimits':{'-1':'1'},'jawSide':{'asFaceBS.mouth_jawSideIn_M':'1','asFaceBS.mouth_jawSideOut_M':'-1'},'jawSide-setLimits':{'-1':'1'}},'Gizmo',1,0.4361809946,[0, -2.960894823, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlMouthCorner_R','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.mouth_wide_R':'1','asFaceBS.mouth_narrow_R':'-1'},'translateY':{'asFaceBS.mouth_raiser_R':'1','asFaceBS.mouth_lower_R':'-1'}},'Gizmo',1,0.4361809946,[-1.156599522, -4.626398087, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlMouthCorner_L','ctrlBox','ctrlBox','ctrlBox',{'translateX':{'asFaceBS.mouth_wide_L':'1','asFaceBS.mouth_narrow_L':'-1'},'translateY':{'asFaceBS.mouth_raiser_L':'1','asFaceBS.mouth_lower_L':'-1'}},'Gizmo',1,0.4361809946,[1.156599522, -4.626398087, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlPhonemes_M','ctrlBox','ctrlBox','ctrlBox',{'aaa':{'asFaceBS.aaa_M':'1'},'eh':{'asFaceBS.eh_M':'1'},'ahh':{'asFaceBS.ahh_M':'1'},'ohh':{'asFaceBS.ohh_M':'1'},'uuu':{'asFaceBS.uuu_M':'1'},'iee':{'asFaceBS.iee_M':'1'},'rrr':{'asFaceBS.rrr_M':'1'},'www':{'asFaceBS.www_M':'1'},'sss':{'asFaceBS.sss_M':'1'},'fff':{'asFaceBS.fff_M':'1'},'tth':{'asFaceBS.tth_M':'1'},'mbp':{'asFaceBS.mbp_M':'1'},'ssh':{'asFaceBS.ssh_M':'1'},'schwa':{'asFaceBS.schwa_M':'1'},'gk':{'asFaceBS.gk_M':'1'},'lntd':{'asFaceBS.lntd_M':'1'}},'Gizmo',1,0.3347000501,[-0.7402236938, -6.014317513, 0],[1, 1, 0, 1.0])
	asAddCtrl ('ctrlEmotions_M','ctrlBox','ctrlBox','ctrlBox',{'happy':{'asFaceBS.happy_M':'1'},'angry':{'asFaceBS.angry_M':'1'},'sad':{'asFaceBS.sad_M':'1'},'surprise':{'asFaceBS.surprise_M':'1'},'fear':{'asFaceBS.fear_M':'1'},'disgust':{'asFaceBS.disgust_M':'1'},'contempt':{'asFaceBS.contempt_M':'1'}},'Gizmo',1,0.4350137246,[0, -6.014317513, 0],[1, 1, 0, 1.0])
	#//-- ASControllers Ends Here --//

	asBackwardsSolveNodes()
	print ("ControlRig created")

if __name__ == "__main__":
    main()