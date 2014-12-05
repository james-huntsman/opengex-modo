// =============================================================
// 
//  Open Game Engine Exchange
//  http://opengex.org/
//
//  Export plugin for MODO
//  by James Huntsman
//	opengex [at] jameshuntsman [dot] com
//  
//  Or use the discussion threadbelow in the C4 forums.
//  http://www.terathon.com/forums/viewtopic.php?f=15&t=13669
//
//	Version 1.0.0
//
//  Copyright 2014, James Huntsman
//
//  Based on the Max and Maya OpenGex Export Plugins
//  by Eric Lengyel
// 
//  Copyright 2014, Terathon Software LLC
//
//  Created using some functions already created by Eric Lengyel
//  from the site http://opengex.org. I have tried to maintain at
//  least the same 'Write' functions, similar structs for holding
//  data, and the same UnifyingVertices function. So at least you
//  should be able to visit the other projects and understand the
//  basics that have been used.
//
//  This software is licensed under the Creative Commons
//  Attribution-ShareAlike 3.0 Unported License:
//
//  http://creativecommons.org/licenses/by-sa/3.0/deed.en_US
// 
// =============================================================


#include "OpenGex-Modo.h"

#include <lxu_prefvalue.hpp>
#include "lxu_math.hpp"
#include "lxu_prefvalue.hpp"
#include "lx_locator.hpp"
#include "lx_select.hpp"

namespace
{
	const double kRadToDeg = 57.295779513082320876798154814105F;
	const float kDegToRad = 0.01745329252F;		// pi / 180

	const float kExportEpsilon = 1.0e-6F;		// Values smaller than this are considered to be zero.

	const char *ATTRVALUE_DOTSEPARATORSYMBOL = ".";

	const char *ATTRVALUE_X = "X";
	const char *ATTRVALUE_Y = "Y";
	const char *ATTRVALUE_Z = "Z";

	static const std::string translationChannelXname = std::string(LXsICHAN_TRANSLATION_POS) + std::string(ATTRVALUE_DOTSEPARATORSYMBOL) + std::string(ATTRVALUE_X);
	static const std::string translationChannelYname = std::string(LXsICHAN_TRANSLATION_POS) + std::string(ATTRVALUE_DOTSEPARATORSYMBOL) + std::string(ATTRVALUE_Y);
	static const std::string translationChannelZname = std::string(LXsICHAN_TRANSLATION_POS) + std::string(ATTRVALUE_DOTSEPARATORSYMBOL) + std::string(ATTRVALUE_Z);
	static const std::string rotationChannelXname = std::string(LXsICHAN_ROTATION_ROT) + std::string(ATTRVALUE_DOTSEPARATORSYMBOL) + std::string(ATTRVALUE_X);
	static const std::string rotationChannelYname = std::string(LXsICHAN_ROTATION_ROT) + std::string(ATTRVALUE_DOTSEPARATORSYMBOL) + std::string(ATTRVALUE_Y);
	static const std::string rotationChannelZname = std::string(LXsICHAN_ROTATION_ROT) + std::string(ATTRVALUE_DOTSEPARATORSYMBOL) + std::string(ATTRVALUE_Z);
	static const std::string scaleChannelXname = std::string(LXsICHAN_SCALE_SCL) +  std::string(ATTRVALUE_DOTSEPARATORSYMBOL) + std::string(ATTRVALUE_X);
	static const std::string scaleChannelYname = std::string(LXsICHAN_SCALE_SCL) + std::string(ATTRVALUE_DOTSEPARATORSYMBOL) + std::string(ATTRVALUE_Y);
	static const std::string scaleChannelZname = std::string(LXsICHAN_SCALE_SCL) + std::string(ATTRVALUE_DOTSEPARATORSYMBOL) + std::string(ATTRVALUE_Z);

	static const char* LXsUSER_VALUE_OPENGEX_USE_ABSOLUTE_PATH = "sceneio.opengex.save.use.absolute.path";
	static const char* LXsUSER_VALUE_OPENGEX_TEXTURE_REMOVE_FILE_EXTENSION = "sceneio.opengex.texture.remove.file.extension";
	static const char* LXsUSER_VALUE_OPENGEX_SAVE_HIDDEN_ITEMS = "sceneio.opengex.save.hidden.items";
	static const char* LXsUSER_VALUE_OPENGEX_SAVE_QUADS = "sceneio.opengex.save.quads";
	static const char* LXsUSER_VALUE_OPENGEX_SAVE_TRIANGULATE_MESH = "sceneio.opengex.save.triangulate.mesh";
	static const char* LXsUSER_VALUE_OPENGEX_Z_NEAR = "sceneio.opengex.z.near";
	static const char* LXsUSER_VALUE_OPENGEX_Z_FAR = "sceneio.opengex.z.far";


	static void BuildAxisOrder(int rotationOrder, int axis[3])
	{
		switch (rotationOrder)
		{
			case ROTATION_ORDER_XZY:

				axis[0] = 0;
				axis[1] = 2;
				axis[2] = 1;
				break;

			case ROTATION_ORDER_YXZ:

				axis[0] = 1;
				axis[1] = 0;
				axis[2] = 2;
				break;

			case ROTATION_ORDER_YZX:

				axis[0] = 1;
				axis[1] = 2;
				axis[2] = 0;
				break;

			case ROTATION_ORDER_ZXY:

				axis[0] = 2;
				axis[1] = 0;
				axis[2] = 1;
				break;

			case ROTATION_ORDER_ZYX:

				axis[0] = 2;
				axis[1] = 1;
				axis[2] = 0;
				break;

			case ROTATION_ORDER_XYZ:

			default:

				axis[0] = 0;
				axis[1] = 1;
				axis[2] = 2;
				break;
		}
	}

	static void CalculateRotation(Quaternion &q, LXtMatrix4& a)
	{
		double trace = a[0][0] + a[1][1] + a[2][2];
		if (trace > 0.0F) 
		{
			double s = 0.5F / sqrt(trace + 1.0F);

			q[0] = (a[2][1] - a[1][2]) * s;
			q[1] = (a[0][2] - a[2][0]) * s;
			q[2] = (a[1][0] - a[0][1]) * s;
			q[3] = 0.25F / s;
		}
		else 
		{
			if (a[0][0] > a[1][1] && a[0][0] > a[2][2]) 
			{
				double s = 2.0F * sqrt(1.0F + a[0][0] - a[1][1] - a[2][2]);

				q[0] = 0.25F * s;
				q[1] = (a[0][1] + a[1][0]) / s;
				q[2] = (a[0][2] + a[2][0]) / s;
				q[3] = (a[2][1] - a[1][2]) / s;
			}
			else if (a[1][1] > a[2][2]) 
			{
				double s = 2.0F * sqrt(1.0F + a[1][1] - a[0][0] - a[2][2]);

				q[0] = (a[0][1] + a[1][0]) / s;
				q[1] = 0.25F * s;
				q[2] = (a[1][2] + a[2][1]) / s;
				q[3] = (a[0][2] - a[2][0]) / s;
			}
			else 
			{
				double s = 2.0F * sqrt(1.0F + a[2][2] - a[0][0] - a[1][1]);

				q[0] = (a[0][2] + a[2][0]) / s;
				q[1] = (a[1][2] + a[2][1]) / s;
				q[2] = 0.25F * s;
				q[3] = (a[1][0] - a[0][1]) / s;
			}
		}
	}

	static void NormalizeVector2D(double& x, double& y)
	{
		double magR = 1.0 / sqrt(x * x + y * y);
		x *= magR;
		y *= magR;
	}

	void Matrix4CreateRotation(LXtMatrix4& m, const double angle, const int axis)
	{
		if (angle == 0.0F)
		{
			return;
		}

		double s = sin(angle);
		double c = cos(angle);

		switch (axis)
		{
			case 0:		// x rotation

				m[1][2] = s;
				m[2][1] = -s;
				m[1][1] = m[2][2] = c;
				break;

			case 1:		// y rotation

				m[0][2] = -s;
				m[2][0] = s;
				m[0][0] = m[2][2] = c;
				break;

			case 2:		// z rotation

				m[0][1] = s;
				m[1][0] = -s;
				m[0][0] = m[1][1] = c;
				break;
		}
	}
}


using namespace OpenGex;

ExportVertex::ExportVertex()
{
	for (int i = 0; i < kMaxVertexColorCount; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			color[i][j] = 1.0F; // .set(MColor::kRGB, 1.0F, 1.0F, 1.0F, 1.0F);
		}

	}

	for (int i = 0; i < kMaxTexcoordCount; i++)
	{
		texcoord[i][0] = 0.0F;
		texcoord[i][1] = 0.0F;
	}
}

bool ExportVertex::operator ==(const ExportVertex& v) const
{
	if (hash != v.hash)
	{
		return (false);
	}

	for (int i = 0; i < 3; i++)
	{
		if (position[i] != v.position[i])
		{
			return (false);
		}
	}

	for (int i = 0; i < 3; i++)
	{
		if (normal[i] != v.normal[i])
		{
			return (false);
		}
	}

	for (int i = 0; i < kMaxVertexColorCount; i++)
	{
		if (color[i][0] != v.color[i][0] || color[i][1] != v.color[i][1] || color[i][2] != v.color[i][2])
		{
			return (false);
		}
	}

	for (int i = 0; i < kMaxTexcoordCount; i++)
	{
		if ((texcoord[i][0] != v.texcoord[i][0]) || (texcoord[i][1] != v.texcoord[i][1]))
		{
			return (false);
		}
	}

	return (true);
}

void ExportVertex::Hash(void)
{
	const unsigned int *data = reinterpret_cast<const unsigned int *>(&position);

	unsigned int h = data[0];
	h = ((h << 5) | (h >> 26)) + data[1];
	h = ((h << 5) | (h >> 26)) + data[2];

	data = reinterpret_cast<const unsigned int *>(&normal);

	h = ((h << 5) | (h >> 26)) + data[0];
	h = ((h << 5) | (h >> 26)) + data[1];
	h = ((h << 5) | (h >> 26)) + data[2];

	for (int i = 0; i < kMaxVertexColorCount; i++)
	{
		data = reinterpret_cast<const unsigned int *>(&color[i]);

		h = ((h << 5) | (h >> 26)) + data[0];
		h = ((h << 5) | (h >> 26)) + data[1];
		h = ((h << 5) | (h >> 26)) + data[2];
		h = ((h << 5) | (h >> 26)) + data[3];
	}

	for (int i = 0; i < kMaxTexcoordCount; i++)
	{
		data = reinterpret_cast<const unsigned int *>(&texcoord[i]);

		h = ((h << 5) | (h >> 26)) + data[0];
		h = ((h << 5) | (h >> 26)) + data[1];
	}

	hash = h;
}


OpenGexPreferences::OpenGexPreferences()
{

}

OpenGexPreferences::~OpenGexPreferences()
{

}

bool OpenGexPreferences::GetUseAbsolutePath() const
{
	CLxReadUserValue userValue;

	bool useAbsolutePath = false;
	
	if (userValue.Query(LXsUSER_VALUE_OPENGEX_USE_ABSOLUTE_PATH))
	{
		useAbsolutePath = userValue.GetInt() == 1;
	}

	return (useAbsolutePath);
}

bool OpenGexPreferences::GetTextureRemoveFileExtension() const
{
	CLxReadUserValue userValue;

	bool removeExtension = true;

	if (userValue.Query(LXsUSER_VALUE_OPENGEX_TEXTURE_REMOVE_FILE_EXTENSION))
	{
		removeExtension = userValue.GetInt() == 1;
	}

	return (removeExtension);
}

int OpenGexPreferences::GetUnitSystem() const
{
	CLxReadPreferenceValue prefValue;
	int unitSystem = LXiPREFERENCE_VALUE_UNIT_SYSTEM_METRIC;

	if (prefValue.Query(LXsPREFERENCE_VALUE_ACCURACY_UNIT_SYSTEM))
	{
		unitSystem = prefValue.GetInt();
	}

	return (unitSystem);
}

int OpenGexPreferences::GetDefaultUnit() const
{
	CLxReadPreferenceValue	prefValue;
	int unit = LXsPREFERENCE_VALUE_UNIT_SI_METERS;

	if (prefValue.Query(LXsPREFERENCE_VALUE_ACCURACY_DEFAULT_UNIT))
	{
		unit = prefValue.GetInt();
	}

	return (unit);
}

double OpenGexPreferences::GetMetersPerGameUnit() const
{
	CLxReadPreferenceValue prefValue;
	double unit = 1.0F;

	if (prefValue.Query(LXsPREFERENCE_VALUE_METERS_PER_GAME_UNIT))
	{
		unit = prefValue.GetFloat();
	}

	return (unit);
}

int OpenGexPreferences::GetUpAxis() const
{
	CLxReadPreferenceValue	prefValue;
	int	upAxis = LXsPREFERENCE_VALUE_UP_AXIS_Y;

	if (prefValue.Query(LXsPREFERENCE_VALUE_ACCURACY_UP_AXIS))
	{
		upAxis = prefValue.GetInt();
	}

	return (upAxis);
}

bool OpenGexPreferences::GetSaveHiddenItems() const
{
	CLxReadUserValue userValue;

	bool saveHiddenItems = false;

	if (userValue.Query(LXsUSER_VALUE_OPENGEX_SAVE_HIDDEN_ITEMS))
	{
		saveHiddenItems = userValue.GetInt() == 1;
	}

	return (saveHiddenItems);
}

bool OpenGexPreferences::GetSaveQuads() const
{
	CLxReadUserValue userValue;

	bool saveQuads = true;

	if (userValue.Query(LXsUSER_VALUE_OPENGEX_SAVE_QUADS))
	{
		saveQuads = userValue.GetInt() == 1;
	}

	return (saveQuads);
}

bool OpenGexPreferences::GetTriangulateMesh() const
{
	CLxReadUserValue userValue;

	bool triangulateMesh = true;

	if (userValue.Query(LXsUSER_VALUE_OPENGEX_SAVE_TRIANGULATE_MESH))
	{
		triangulateMesh = userValue.GetInt() == 1;
	}

	return (triangulateMesh);
}

double OpenGexPreferences::GetZNear() const
{
	CLxReadUserValue userValue;
	double zNear = 0.01f;

	if (userValue.Query(LXsUSER_VALUE_OPENGEX_Z_NEAR))
	{
		zNear = userValue.GetFloat();
	}

	return (zNear);
}

double OpenGexPreferences::GetZFar() const
{
	CLxReadUserValue userValue;

	double zFar = 10000.0F;

	if (userValue.Query(LXsUSER_VALUE_OPENGEX_Z_FAR))
	{
		zFar = userValue.GetFloat();
	}

	return (zFar);
}

DeformerWeightToVertexVisitor::DeformerWeightToVertexVisitor(CLxUser_Item& meshItem, CLxUser_GroupDeformer *groupDeformerID, 
		unsigned groupDeformerMeshIndex, std::set<std::string> *jointNodeIDs) :
	isLocatorPass(false),
	groupDeformer(groupDeformerID),
	meshIndex(groupDeformerMeshIndex)
{
	CLxUser_Mesh mesh;
	scene.from(meshItem);

	// Link to the set of node IDs and clear it out for the next visitation.

	nodeIDs = jointNodeIDs;
	nodeIDs->clear();

	if (scene.GetChannels(setupChannels, LXs_ACTIONLAYER_EDIT))  	// get channels for base mesh
	{
		// Get the mesh from the mesh item.

		setupChannels.Object(meshItem, LXsICHAN_MESH_MESH, mesh);

		if (mesh.test())
		{
			meshPoint.fromMesh(mesh);
		}
	}

	scene.GetSetupChannels(setupChannels);
	scene.GetGraph(LXsGRAPH_XFRMLOCAL, sceneGraph);
	itemGraph.set(sceneGraph);
	
	// Skin bind pose

	CalculateBindPoseMatrix(meshItem, skinBindMatrix.m);
}

DeformerWeightToVertexVisitor::~DeformerWeightToVertexVisitor()
{
}

void DeformerWeightToVertexVisitor::CalculateBindPoseMatrix(CLxUser_Item& locatorItem, LXtMatrix4& bindMatrix)
{
	LXtMatrix4 itemMatrix;
	CLxUser_Matrix umatrix;
	CLxUser_Matrix pmatrix;

	setupChannels.Object(locatorItem, LXsICHAN_XFRMCORE_WORLDMATRIX, umatrix);
	setupChannels.Object(locatorItem, LXsICHAN_XFRMCORE_PARENTMATRIX, pmatrix);
	pmatrix.Invert();

	pmatrix.Get4(itemMatrix);
	umatrix.Multiply4(itemMatrix);
	umatrix.Get4(itemMatrix);

	lx::Matrix4Copy(bindMatrix, itemMatrix);
}

void DeformerWeightToVertexVisitor::FindLocators()
{
	isLocatorPass = true;
	Enumerate();
	isLocatorPass = false;
}

LxResult DeformerWeightToVertexVisitor::Enumerate()
{
	// First iterate over all of the points.
	
	LxResult result = meshPoint.Enum(this);

	// If this is the second pass (the first pass gathered the Locators),
	// create the matching array of bind matrices.

	if (LXx_OK(result) && !isLocatorPass)
	{
		for (std::set<std::string>::const_iterator iter = nodeIDs->begin (); iter != nodeIDs->end (); ++iter)
		{
			CLxUser_Item	nodeItem;
			std::string		nodeID = *iter;

			if (scene.GetItem(nodeID.c_str (), nodeItem))
			{
				Matrix4Wrap invBindMatrix;

				CalculateBindPoseMatrix(nodeItem, invBindMatrix.m);
				bindMatrices.push_back(invBindMatrix);
				result = LXe_OK;
			}
		}
	}

	return (result);
}

LxResult DeformerWeightToVertexVisitor::Evaluate()
{
	LxResult result = LXe_FAILED;

	if (meshPoint.test())
	{
		unsigned index;
		meshPoint.Index(&index);

		// Get the list of deformers and weights for this point.

		groupDeformer->PointEffectList(meshIndex, meshPoint.ID(), deformerWeightList);

		if (isLocatorPass)
		{
			result = EvaluateLocatorPass();
		}
		else
		{
			result = EvaluateJointWeightPass();
		}
	}

	return (result);
}

LxResult DeformerWeightToVertexVisitor::EvaluateLocatorPass()
{
	LxResult result = LXe_FAILED;

	for (DeformerWeightArray::const_iterator iter = deformerWeightList.begin (); iter != deformerWeightList.end (); ++iter)
	{
		// Get the indexed vertex influence within the group.

		CLxLoc_Item	indexedDeformer;
		unsigned deformerIndex = iter->deformer;

		if (groupDeformer->GetDeformer(deformerIndex, indexedDeformer))
		{
			// Get the locator referenced by the vertex influence.

			CLxUser_DeformerService deformerSvc;
			CLxUser_Item locator;
			bool isLocator;

			if (deformerSvc.GetDeformerDeformationItem(indexedDeformer, locator, isLocator))
			{
				if (isLocator)
				{
					// Insert the locator ID into the set of unique IDs.

					nodeIDs->insert(locator.GetIdentity());
				}
			}
		}
	}

	result = LXe_OK;

	return (result);
}

LxResult DeformerWeightToVertexVisitor::EvaluateJointWeightPass()
{
	LxResult result = LXe_OK;

	boneCounts.push_back(static_cast<unsigned>(deformerWeightList.size()));

	for (DeformerWeightArray::const_iterator iter = deformerWeightList.begin(); iter != deformerWeightList.end (); ++iter)
	{
		// Get the indexed vertex influence within the group.

		CLxLoc_Item	indexedDeformer;
		unsigned deformerIndex = iter->deformer;

		if (groupDeformer->GetDeformer(deformerIndex, indexedDeformer))
		{
			// Get the locator referenced by the vertex influence.

			CLxUser_DeformerService deformerSvc;
			CLxUser_Item locator;
			bool isLocator;

			if (deformerSvc.GetDeformerDeformationItem(indexedDeformer, locator, isLocator))
			{
				if (isLocator)
				{
					std::string locatorID = locator.GetIdentity();
					NodeIDIndexMap::const_iterator locatorIter = nodeIndexMap.find(locatorID);

					if (locatorIter != nodeIndexMap.end())
					{
						jointWeightIndices.push_back(locatorIter->second);
					}

					jointWeightIndices.push_back(static_cast<unsigned>(weights.size()));
					weights.push_back(iter->weight);
				}
			}
		}
	}

	return (result);
}


OpenGexExport::OpenGexExport()
{

}

OpenGexExport::~OpenGexExport()
{
}

void OpenGexExport::ss_Verify()
{

}

double OpenGexExport::SetUnitScale(int unitSystem, int unit)
{
	double unitScale = 1.0F;

	if (unitSystem == LXiPREFERENCE_VALUE_UNIT_SYSTEM_SI)
	{
		switch (unit)
		{
			case LXsPREFERENCE_VALUE_UNIT_SI_MICRONS:

				unitScale = METERS_PER_MICRON;
				break;

			case LXsPREFERENCE_VALUE_UNIT_SI_MILLIMETERS:
	
				unitScale = METERS_PER_MILLIMETER;
				break;

			case LXsPREFERENCE_VALUE_UNIT_SI_METERS:

				break; // default value

			case LXsPREFERENCE_VALUE_UNIT_SI_KILOMETERS:

				unitScale = METERS_PER_KILOMETER;
				break;

			case LXsPREFERENCE_VALUE_UNIT_SI_MEGAMETERS:
				
				unitScale = METERS_PER_MEGAMETER;
				break;
		}
	}
	else if (unitSystem == LXiPREFERENCE_VALUE_UNIT_SYSTEM_METRIC)
	{
		switch (unit)
		{
			case LXsPREFERENCE_VALUE_UNIT_METRIC_MICRONS:

				unitScale = METERS_PER_MICRON;
				break;

			case LXsPREFERENCE_VALUE_UNIT_METRIC_MILLIMETERS:

				unitScale = METERS_PER_MILLIMETER;
				break;

			case LXsPREFERENCE_VALUE_UNIT_METRIC_CENTIMETERS:

				unitScale = METERS_PER_CENTIMETER;
				break;

			case LXsPREFERENCE_VALUE_UNIT_METRIC_METERS:

				break; // default value

			case LXsPREFERENCE_VALUE_UNIT_METRIC_KILOMETERS:

				unitScale = METERS_PER_KILOMETER;
				break;

			case LXsPREFERENCE_VALUE_UNIT_METRIC_MEGAMETERS:

				unitScale = METERS_PER_MEGAMETER;
				break;
		}
	}
	else if (unitSystem == LXiPREFERENCE_VALUE_UNIT_SYSTEM_ENGLISH)
	{
		switch (unit)
		{
			case LXsPREFERENCE_VALUE_UNIT_ENGLISH_MILS:

				unitScale = METERS_PER_MIL;
				break;

			case LXsPREFERENCE_VALUE_UNIT_ENGLISH_INCHES:

				unitScale = METERS_PER_INCH;
				break;

			case LXsPREFERENCE_VALUE_UNIT_ENGLISH_FEET:

				unitScale = METERS_PER_FOOT;
				break;

			case LXsPREFERENCE_VALUE_UNIT_ENGLISH_MILES:

				unitScale = METERS_PER_MILE;
				break;
		}
	}
	else if (unitSystem == LXiPREFERENCE_VALUE_UNIT_SYSTEM_GAME_UNITS)
	{
		unitScale = prefs.GetMetersPerGameUnit();
	}
	else if (unitSystem == LXiPREFERENCE_VALUE_UNIT_SYSTEM_UNITLESS)
	{
		// default value
	}

	return (unitScale);
}

LxResult OpenGexExport::ss_Save()
{
	// There are two passes of ss_Save(), the first we'll use to 
	// populate the nodeArray and then  on the second pass we'll 
	// use the nodeArray to export the nodes.
	// ReallySaving() returns true for the second pass

	LxResult result(LXe_OK);

	if (!ReallySaving())
	{
		// Start of the first pass, set the arrays up.

		nodeArray = new std::vector<NodeReference>;
		geometryArray = new std::vector<ObjectReference>;
		lightArray = new std::vector<ObjectReference>;
		cameraArray = new std::vector<ObjectReference>;
		materialArray = new std::vector<MaterialReference>;
		textureArray = new std::vector<TextureReference>;
	}
	else
	{
		// We will output the unit scale as 1 meter == 1 unit

		int	upAxis = prefs.GetUpAxis();
		int unit = prefs.GetDefaultUnit();
		int unitSystem = prefs.GetUnitSystem();
		double unitScale = SetUnitScale(unitSystem, unit);

		Write("Metric (key = \"distance\") {float {");
		WriteFloat(unitScale);
		Write("}}\n");
		
		Write("Metric (key = \"angle\") {float {1}}\n");
		Write("Metric (key = \"time\") {float {1}}\n");

		if (upAxis == LXsPREFERENCE_VALUE_UP_AXIS_Z)
		{
			Write("Metric (key = \"up\") {string {\"z\"}}\n");
		}
		else
		{
			// If not Z up then default to Y

			Write("Metric (key = \"up\") {string {\"y\"}}\n");
		}

		// This is called on the second pass, we just want to parse the captured
		// objects and convert any locators that are deformers to Bones

		ProcessObjects();
	}

	CLxUser_Scene scene(SceneObject());
	scene.GetGraph(LXsGRAPH_XFRMCORE, itemGraph);

	saveVertexNormals = true;
	saveUVTextureCoordinates = true;
	saveColors = true;

	indentLevel = 0;
	pointIndex = 0;

	// This is the first pass of the scene graph, identify all of the nodes 
	// that we want to use and populate the respective node Arrays

	if (!ReallySaving())
	{
		StartScan();
		while (NextItem())
		{
			std::string itemName = ItemName();
			std::string itemIdentity = ItemIdentity();

			int type = kNodeTypeNone;

			if (ItemIsA(LXsITYPE_MESH))
			{
				type = kNodeTypeGeometry;

				if (PointCount() == 0)
				{
					// Mesh object is in the scene but has no data
					// Do not process by setting type to kNodeTypeNone

					type = kNodeTypeNone;
				}
			}
			else if (ItemIsA(LXsITYPE_CAMERA))
			{
				type = kNodeTypeCamera;
			}
			else if (ItemIsA(LXsITYPE_LIGHT))
			{
				type = kNodeTypeLight;
			}
			else if (ItemIsA(LXsITYPE_LOCATOR))
			{
				// We only want the Locators that are potentially bones and groups. There's a 
				// LXsITYPE_GROUPLOCATOR but we can leave that in it'll be converted to a node.
				// But the Locators we don't want are Texture Locators

				if (!ItemIsA(LXsITYPE_TEXTURELOC))
				{
					type = kNodeTypeNode;
				}
			}

			if (type > kNodeTypeNone)
			{
				bool saveItem(prefs.GetSaveHiddenItems());

				if (!saveItem)
				{
					saveItem = ItemVisible();
				}
				
				// Only add to the nodeArray if the item is visible or save hidden items is
				// checked for saving.

				if (saveItem)
				{
					int size = (int)nodeArray->size();
					nodeArray->push_back(NodeReference(itemIdentity.c_str(), itemName.c_str(), type, (std::string("node") += std::to_string(size + 1)).c_str()));
				}
			}
		}
	}
	else
	{
		// The second pass exports the respective Nodes, Objects and Materials.

		StartScan();
		while(NextItem())
		{
			CLxLoc_Item root;
			GetItem(root);

			CLxLoc_Item parent;
			root.Parent(parent);

			if (!parent)
			{
				const char * itemIdentity;
				root.Ident(&itemIdentity);

				ExportNode(itemIdentity);
			}
		}

		ExportObjects();
		ExportMaterials();

		// End of the second pass, clear all of the variables.

		ClearData();

		delete textureArray;
		delete materialArray;
		delete cameraArray;
		delete lightArray;
		delete geometryArray;
		delete nodeArray;
	}

	return (result);
}

void OpenGexExport::ProcessObjects(void)
{
	int count = (int)nodeArray->size();

	if (count != 0)
	{
		NodeReference *nodeRef = &nodeArray->front();
		StringArray groupDeformers;
		CLxUser_Scene scene(SceneObject());
		CLxUser_DeformerService deformerSvc;

		for (int i = 0; i < count; i++)
		{
			CLxUser_Item item;
			scene.GetItemByIdent(nodeRef->node.c_str(), item);

			if (item)
			{
				CLxUser_GroupDeformer groupDeformer;
				unsigned groupDeformerMeshIndex;

				// If the item has deformers, change NodeReference::nodeType of Locators to bones.

				if (FindGroupDeformer(item, groupDeformer, groupDeformerMeshIndex))
				{
					if (groupDeformer.test())
					{
						int deformerCount = groupDeformer.DeformerCount();

						for (int index = 0; index < deformerCount; index++)
						{
							CLxUser_Item bone;
							if (groupDeformer.GetDeformer(index, bone))
							{
								const char * identity;
								bone.Ident(&identity);

								bool	isLocator;
								CLxLoc_Item	locator;

								if (deformerSvc.GetDeformerDeformationItem(bone, locator, isLocator))
								{
									const char *locatorName;
									locator.Name(&locatorName);

									const char *locatorIdentity;
									locator.Ident(&locatorIdentity);

									int size = (int)nodeArray->size();

									NodeReference *boneRef = FindNode(locatorIdentity);
									if (boneRef)
									{
										boneRef->nodeType = kNodeTypeBone;
									}
								}
							}
						}
					}
				}
			}

			nodeRef++;
		}
	}
}

void OpenGexExport::ss_Point()
{
	// We add a pointMap, the pointMap returns the LXtPointID from a vertex index

	pointMap[PntID()] = pointIndex++;

	double	position[3];
	PntPosition(position);

	points.push_back(static_cast<float>(position[0]));
	points.push_back(static_cast<float>(position[1]));
	points.push_back(static_cast<float>(position[2]));
}

void OpenGexExport::ss_Polygon()
{
	switch (polyPass)
	{
		case POLYPASS_POLYGONS:

			AddPolygon();

			break;

		case POLYPASS_BUILDMATERIALSET:

			// Add unique materials (get the material name from the
			// polygon's material tag) to the materials array

			std::string polyMaterialTag(PolyTag(LXi_PTAG_MATR));
			bool unique = true;

			if (materials.size() > 0)
			{
				for (unsigned int i = 0; i < materials.size(); i++)
				{
					std::string material = materials[i];

					if (polyMaterialTag.compare(material) == 0)
					{
						unique = false;
						break;
					}
				}
			}

			if (unique)
			{
				materials.push_back(polyMaterialTag);
			}

			break;

	}
}

void OpenGexExport::AddPolygon()
{
	// We just use the AddPolygon function to store the following 
	// - number of vertex indices per polygon
	//   - for each index we store the vertex index
	//   - for each index we store the pointID - LXtPointID
	// - material for each polygon

	std::string polyMaterialTag(PolyTag(LXi_PTAG_MATR));
	unsigned polyVertexCount = PolyNumVerts();
	polyCounts.push_back(polyVertexCount);
	polyMaterial.push_back(polyMaterialTag);

	if (polyVertexCount)
	{
		for (unsigned polyVertexIndex = 0; polyVertexIndex < polyVertexCount; ++polyVertexIndex)
		{
			LXtPointID pointID = PolyVertex(polyVertexIndex);
			unsigned vertexIndex = pointMap[pointID];
			polyIndices.push_back(vertexIndex);
			polyPoints.push_back(pointID);
		}
	}
}

bool OpenGexExport::FindGroupDeformer(CLxUser_Item& meshItem, CLxUser_GroupDeformer& groupDeformer, unsigned& groupDeformerMeshIndex)
{
	CLxUser_DeformerService deformerSvc;
	CLxUser_Item deformer;

	const char *ident1;
	meshItem.Ident(&ident1);

	CLxUser_Scene scene(SceneObject());
	unsigned itemCount = scene.NItems(LXiTYPE_ANY);

	for (unsigned itemIndex = 0; itemIndex < itemCount; ++itemIndex)
	{
		CLxUser_Item testItem;
		CLxUser_Item otherMesh;
		const char *ident2;

		if (scene.GetItem(LXiTYPE_ANY, itemIndex, testItem))
		{
			if (!deformerSvc.IsDeformer(testItem))
			{
				continue;
			}

			unsigned meshCount = 0;

			if (deformerSvc.MeshCount(testItem, &meshCount) != LXe_TRUE)
			{
				continue;
			}

			for (unsigned meshIndex = 0; meshIndex < meshCount; ++meshIndex)
			{
				if (deformerSvc.GetMesh(testItem, meshIndex, otherMesh))
				{
					otherMesh.Ident(&ident2);

					if ((ident1 == ident2) && deformer.set(testItem))
					{
						groupDeformerMeshIndex = meshIndex;
						break;
					}
				}
			}

			if (deformer.test())
			{
				break;
			}
		}
	}

	if (deformer.test())
	{
		// Get the group deformer object.

		CLxUser_ChannelRead	 chans;
		scene.GetChannels(chans, 0.0F);

		if (deformerSvc.GetGroupDeformer(deformer, chans, groupDeformer))
		{
			unsigned	count = groupDeformer.DeformerCount();

			for (unsigned index = 0; index < count; ++index)
			{
				CLxLoc_Item	indexedDeformer;

				if (groupDeformer.GetDeformer(index, indexedDeformer))
				{
					const char *deformerName;
					const char *deformerID;
					indexedDeformer.Name(&deformerName);
					indexedDeformer.Ident(&deformerID);

					bool	isLocator;
					CLxLoc_Item	locator;

					if (deformerSvc.GetDeformerDeformationItem(indexedDeformer, locator, isLocator))
					{
						// Found at least one deformer.

						return (true);
					}
				}
			}
		}
	}

	return (false);
}

void OpenGexExport::ClearData()
{
	pointMap.clear();
	points.clear();

	for (TexcoordMapIDArray::iterator tcmIter = texcoordMaps.begin(); tcmIter != texcoordMaps.end(); ++tcmIter)
	{
		delete *tcmIter;
	}

	for (ColorRGBMapIDArray::iterator cmIter = colorRGBMaps.begin(); cmIter != colorRGBMaps.end(); ++cmIter)
	{
		delete *cmIter;
	}

	for (ColorRGBAMapIDArray::iterator cmIter = colorRGBAMaps.begin(); cmIter != colorRGBAMaps.end(); ++cmIter)
	{
		delete *cmIter;
	}

	texcoordMaps.clear();
	colorRGBMaps.clear();
	colorRGBAMaps.clear();
	normalMapNames.clear();
	uvMapNames.clear();
	colorRGBMapNames.clear();
	colorRGBAMapNames.clear();

	polyCounts.clear();
	polyMaterial.clear();
	polyIndices.clear();
	polyPoints.clear();
}

void OpenGexExport::VisitPolygons(PolyPass inPolyPass)
{
	polyPass = inPolyPass;

	polyCounts.clear();
	polyMaterial.clear();
	polyIndices.clear();
	polyPoints.clear();

	WritePolys(0, polyPass == POLYPASS_POLYGONS);
}

void OpenGexExport::ExportNode(const char *ident)
{
	const NodeReference *nodeRef = FindNode(ident);

	if (nodeRef)
	{
		static const char *const structIdentifier[kNodeTypeCount] =
		{
			"Node $", "BoneNode $", "GeometryNode $", "LightNode $", "CameraNode $"
		};

		int type = nodeRef->nodeType;

		IndentWrite(structIdentifier[type], 0, true);
		Write(nodeRef->structName.c_str());

		IndentWrite("{\n", 0, true);
		indentLevel++;

		bool structFlag = false;

		CLxUser_Scene scene(SceneObject());
		CLxUser_Item item;
		scene.GetItemByIdent(ident, item);

		const char *name = nodeRef->nodeName.c_str();

		if (name[0] != 0)
		{
			// Export the node's name if it has one.

			IndentWrite("Name {string {\"");
			Write(name);
			Write("\"}}\n");

			structFlag = true;
		}

		if (type == kNodeTypeGeometry)
		{
			ObjectReference *objectRef = FindObject(geometryArray, ident);

			if (!objectRef)
			{
				int size = (int)geometryArray->size();
				geometryArray->push_back(ObjectReference(ident, (std::string("geometry") += std::to_string(size + 1)).c_str(), ident));
				objectRef = &geometryArray->at(size);
			}
			else
			{
				objectRef->nodeTable.push_back(ident);
			}

			IndentWrite("ObjectRef {ref {$");
			Write(objectRef->structName.c_str());
			Write("}}\n");

			SetItem(item);
			materials.clear();
			materialIndexArray.clear();

			VisitPolygons(POLYPASS_BUILDMATERIALSET);

			unsigned int materialCount = (unsigned)materials.size();

			for (unsigned int i = 0; i < materialCount; i++)
			{
				const char *material = materials[i].c_str();
				ExportMaterialRef(material, i);
			}

			structFlag = true;
		}
		else if (type == kNodeTypeCamera)
		{
			ObjectReference *objectRef = FindObject(cameraArray, ident);

			if (!objectRef)
			{
				int size = (int)cameraArray->size();
				cameraArray->push_back(ObjectReference(ident, (std::string("camera") += std::to_string(size + 1)).c_str(), ident));
				objectRef = &cameraArray->at(size);
			}
			else
			{
				objectRef->nodeTable.push_back(ident);
			}

			IndentWrite("ObjectRef {ref {$");
			Write(objectRef->structName.c_str());
			Write("}}\n");

			structFlag = true;
		}
		else if (type == kNodeTypeLight)
		{
			ObjectReference *objectRef = FindObject(lightArray, ident);

			if (!objectRef)
			{
				int size = (int)lightArray->size();
				lightArray->push_back(ObjectReference(ident, (std::string("light") += std::to_string(size + 1)).c_str(), ident));
				objectRef = &lightArray->at(size);
			}
			else
			{
				objectRef->nodeTable.push_back(ident);
			}

			IndentWrite("ObjectRef {ref {$");
			Write(objectRef->structName.c_str());
			Write("}}\n");

			structFlag = true;
		}

		if (structFlag)
		{
			Write("\n");
		}

		// Export the transform. If the node is animated, then animation tracks are exported here.

		ExportTransform(ident);

		// Recursively export the sub nodes.

		unsigned int subnodeCount;
		item.SubCount(&subnodeCount);
		for (unsigned int i = 0; i < subnodeCount; i++)
		{
			CLxLoc_Item subItem;
			item.GetSubItem(i, subItem);

			const char * identity;
			subItem.Ident(&identity);
			ExportNode(identity);
		}

		indentLevel--;
		IndentWrite("}\n");
	}
}

void OpenGexExport::ExportMaterialRef(const char *ident, int index)
{
	MaterialReference *materialRef = FindMaterial(ident);

	if (!materialRef)
	{
		int size = (int)materialArray->size();
		materialArray->push_back(MaterialReference(ident, (std::string("material") += std::to_string(size + 1)).c_str()));
		materialRef = &materialArray->at(size);
	}

	if (index < 0)
	{
		IndentWrite("MaterialRef {ref {$");
	}
	else
	{
		IndentWrite("MaterialRef (index = ");
		WriteInt(index);
		Write(") {ref {$");
	}

	Write(materialRef->structName.c_str());
	Write("}}\n");
}

void OpenGexExport::ExportKeyTimes(const Envelope& animationEnvelope)
{
	IndentWrite("Key {float {");

	int keyCount = (int)animationEnvelope.times.size();
	for (int i = 0;;)
	{
		float time = animationEnvelope.times[i];
		WriteFloat(time);

		if (++i >= keyCount)
		{
			break;
		}

		Write(", ");
	}

	Write("}}\n");
}

void OpenGexExport::ExportKeyTimeControlPoints(const Envelope& animationEnvelope)
{
	IndentWrite("Key (kind = \"-control\") {float {");

	int keyCount = (int)animationEnvelope.times.size();
	for (int i = 0;;)
	{
		float x = animationEnvelope.inTanControlX[i];
		WriteFloat(x);

		if (++i >= keyCount)
		{
			break;
		}

		Write(", ");
	}

	Write("}}\n");
	IndentWrite("Key (kind = \"+control\") {float {");

	for (int i = 0;;)
	{
		float x = animationEnvelope.outTanControlX[i];
		WriteFloat(x);

		if (++i >= keyCount)
		{
			break;
		}

		Write(", ");
	}

	Write("}}\n");
}

void OpenGexExport::ExportKeyValues(const Envelope& animationEnvelope)
{
	IndentWrite("Key {float {");

	int keyCount = (int)animationEnvelope.outputs.size();
	for (int i = 0;;)
	{
		float value = animationEnvelope.outputs[i];
		WriteFloat(value);

		if (++i >= keyCount)
		{
			break;
		}

		Write(", ");
	}

	Write("}}\n");
}

void OpenGexExport::ExportKeyValueControlPoints(const Envelope& animationEnvelope)
{
	IndentWrite("Key (kind = \"-control\") {float {");

	int keyCount = (int)animationEnvelope.outputs.size();
	for (int i = 0;;)
	{
		float y = animationEnvelope.inTanControlY[i];
		WriteFloat(animationEnvelope.outputs[i] - y * 0.333333F);

		if (++i >= keyCount)
		{
			break;
		}

		Write(", ");
	}

	Write("}}\n");
	IndentWrite("Key (kind = \"+control\") {float {");

	for (int i = 0;;)
	{
		float y = animationEnvelope.inTanControlY[i];
		WriteFloat(animationEnvelope.outputs[i] + y * 0.333333F);

		if (++i >= keyCount)
		{
			break;
		}

		Write(", ");
	}

	Write("}}\n");
}

void OpenGexExport::ExportAnimationTrack(const Envelope& animationEnvelope, const char *target, bool newline)
{
	// This function exports a single animation track. The curve type for the
	// Time structure depends on whether the animation curve is weighted.

	IndentWrite("Track (target = %", 0, newline);
	Write(target);
	Write(")\n");
	IndentWrite("{\n");
	indentLevel++;

	if (animationEnvelope.interpolationType == LXiENVv_INTERP_LINEAR)
	{
		IndentWrite("Time\n");
		IndentWrite("{\n");
		indentLevel++;

		ExportKeyTimes(animationEnvelope);

		IndentWrite("}\n\n", -1);
		IndentWrite("Value\n", -1);
		IndentWrite("{\n", -1);

		ExportKeyValues(animationEnvelope);

		indentLevel--;
		IndentWrite("}\n");
	}
	else if (animationEnvelope.interpolationType == LXiENVv_INTERP_CURVE)
	{
		IndentWrite("Time (curve = \"bezier\")\n");
		IndentWrite("{\n");
		indentLevel++;

		ExportKeyTimes(animationEnvelope);
		ExportKeyTimeControlPoints(animationEnvelope);

		IndentWrite("}\n\n", -1);
		IndentWrite("Value (curve = \"bezier\")\n", -1);
		IndentWrite("{\n", -1);

		ExportKeyValues(animationEnvelope);
		ExportKeyValueControlPoints(animationEnvelope);

		indentLevel--;
		IndentWrite("}\n");
	}
	else if (animationEnvelope.interpolationType == LXiENVv_INTERP_STEPPED)
	{

	}

	indentLevel--;
	IndentWrite("}\n");
}

bool OpenGexExport::GetKeyFrames(CLxUser_Envelope &envelope, Envelope &animationEnvelope)
{
	bool			 atLeastOneKey(false);
	bool			 knownInterpolationType;
	unsigned		 interpolationType = envelope.Interpolation();

	switch (interpolationType)
	{
		case LXiENVv_INTERP_CURVE:
		case LXiENVv_INTERP_LINEAR:
		case LXiENVv_INTERP_STEPPED:

			knownInterpolationType = true;
			break;

		default:

			knownInterpolationType = false;
			break;
	}

	if (knownInterpolationType)
	{
		CLxUser_Keyframe key(envelope);
		LxResult keyResult = key.First();

		BoolArray brokenValues, brokenWeights, brokenSlopes;
		bool atLeastOneBrokenValue(false), atLeastOneBrokenWeight(false), atLeastOneBrokenSlope(false);

		double initialTime = key.Time();
		double key1 = 0.0F;
		double key2 = 0.0F;

		animationEnvelope.interpolationType = interpolationType;

		while (keyResult != LXe_NOTFOUND)
		{
			double time = key.Time();
			animationEnvelope.times.push_back(static_cast<float>(time));

			unsigned int brokenFlags;
			bool isBrokenValue(false);
			bool isBrokenWeight(false);
			bool isBrokenSlope(false);

			if (interpolationType == LXiENVv_INTERP_CURVE)
			{
				key.GetBroken(&brokenFlags, 0);
				isBrokenValue = (brokenFlags & LXfKEYBREAK_VALUE) ? true : false;
				brokenValues.push_back(isBrokenValue);

				isBrokenWeight = (brokenFlags & LXfKEYBREAK_WEIGHT) ? true : false;
				brokenWeights.push_back(isBrokenWeight);

				isBrokenSlope = (brokenFlags & LXfKEYBREAK_SLOPE) ? true : false;
				brokenSlopes.push_back(isBrokenSlope);
			}

			double value;
			double inValue, outValue;

			if (isBrokenValue)
			{
				atLeastOneBrokenValue = true;

				keyResult = key.GetValueF(&inValue, LXiENVSIDE_IN);
				keyResult = key.GetValueF(&outValue, LXiENVSIDE_OUT);

				if (initialTime == time)
				{
					key1 = inValue;
				}
				key2 = inValue;

				animationEnvelope.outputs.push_back(static_cast<float>(inValue));
			}
			else
			{
				keyResult = key.GetValueF(&value, LXiENVSIDE_BOTH);

				if (initialTime == time)
				{
					key1 = value;
				}
				key2 = value;

				animationEnvelope.outputs.push_back(static_cast<float>(value));
			}

			if (fabs(key2 - key1) > kExportEpsilon)
			{
				atLeastOneKey = true;
			}

			LXtSlopeType	 slopeModeIn = LXiSLOPE_LINEAR_IN;
			LXtSlopeType	 slopeModeOut = LXiSLOPE_LINEAR_IN;

			if (envelope.Interpolation() == LXiENVv_INTERP_CURVE)
			{
				key.GetSlopeType(&slopeModeIn, 0, LXiENVSIDE_IN);
				key.GetSlopeType(&slopeModeOut, 0, LXiENVSIDE_OUT);
			}

			if ((envelope.Interpolation() == LXiENVv_INTERP_LINEAR) || ((slopeModeIn == LXiSLOPE_LINEAR_IN) && (slopeModeOut == LXiSLOPE_LINEAR_OUT)))
			{
				animationEnvelope.interpolations.push_back(INTERPOLATION_LINEAR);
				brokenFlags = 0;
			}
			else if ((envelope.Interpolation() == LXiENVv_INTERP_STEPPED) || ((slopeModeIn == LXiSLOPE_STEPPED) && (slopeModeOut == LXiSLOPE_STEPPED)))
			{
				animationEnvelope.interpolations.push_back(INTERPOLATION_STEP);
				brokenFlags = 0;
			}
			else if (envelope.Interpolation() == LXiENVv_INTERP_CURVE)
			{
				animationEnvelope.interpolations.push_back(INTERPOLATION_BEZIER);
			}
			else
			{
				keyResult = LXe_NOTFOUND;
				break;
			}

			if (envelope.Interpolation() == LXiENVv_INTERP_CURVE)
			{
				double weightIn, weightOut;

				if (isBrokenWeight)
				{
					atLeastOneBrokenWeight = true;

					key.GetWeight(&weightIn, LXiENVSIDE_IN);
					key.GetWeight(&weightOut, LXiENVSIDE_OUT);
				}
				else
				{
					key.GetWeight(&weightIn, LXiENVSIDE_IN);
					weightOut = weightIn;
				}

				double slopeIn, slopeOut;

				if (isBrokenSlope)
				{
					atLeastOneBrokenSlope = true;

					key.GetSlope(&slopeIn, LXiENVSIDE_IN);
					key.GetSlope(&slopeOut, LXiENVSIDE_OUT);
				}
				else
				{
					key.GetSlope(&slopeOut, LXiENVSIDE_BOTH);
					slopeIn = slopeOut;
				}

				double inTCX = -1;
				double inTCY = -slopeIn;
				NormalizeVector2D(inTCX, inTCY);

				double outTCX = 1;
				double outTCY = slopeOut;
				NormalizeVector2D(outTCX, outTCY);

				inTCX *= weightIn;
				inTCX += time;

				outTCX *= weightOut;
				outTCX += time;

				inTCY = weightIn;
				outTCY = weightOut;

				animationEnvelope.inTanControlX.push_back(static_cast<float>(inTCX));
				animationEnvelope.inTanControlY.push_back(static_cast<float>(inTCY));

				animationEnvelope.outTanControlX.push_back(static_cast<float>(outTCX));
				animationEnvelope.outTanControlY.push_back(static_cast<float>(outTCY));
			}

			keyResult = key.Next();
		}
	}

	return (atLeastOneKey);
}

LxResult OpenGexExport::GetChannelKeyframeTimes(const std::string &channelName, Envelope &animationEnvelope, FloatSet &times)
{
	LxResult result(LXe_FALSE);
	CLxUser_Envelope envelope;

	if (ChanEnvelope(channelName.c_str(), envelope))
	{
		if (GetKeyFrames(envelope, animationEnvelope))
		{
			for (FloatArray::const_iterator iter = animationEnvelope.times.begin(); iter != animationEnvelope.times.end(); ++iter)
			{
				times.insert(*iter);
			}

			result = LXe_OK;
		}
	}

	return (result);
}

void OpenGexExport::ExportTransform(const char *ident)
{
	// This function determines whether any animation needs to be exported
	// and writes the node transform in the appropriate form. This is followed
	// by the object transform and the animation tracks.

	Envelope *posAnimationEnvelope[3] = { nullptr, nullptr, nullptr };
	Envelope *rotAnimationEnvelope[3] = { nullptr, nullptr, nullptr };
	Envelope *sclAnimationEnvelope[3] = { nullptr, nullptr, nullptr };

	bool positionAnimated = false;
	bool rotationAnimated = false;
	bool scaleAnimated = false;
	bool posAnimated[3] = { false, false, false };
	bool rotAnimated[3] = { false, false, false };
	bool sclAnimated[3] = { false, false, false };

	bool sampledAnimation = false;

	CLxUser_Scene scene(SceneObject());
	CLxLoc_Item item;
	scene.GetItemByIdent(ident, item);

	SetItem(item);

	FloatSet times;

	const std::string translationChannelNames[3] = { translationChannelXname, translationChannelYname, translationChannelZname };
	const std::string rotationChannelNames[3] = { rotationChannelXname, rotationChannelYname, rotationChannelZname };
	const std::string scaleChannelNames[3] = { scaleChannelXname, scaleChannelYname, scaleChannelZname };

	if (HasXformItem(LXiXFRM_POSITION) && XformItem(LXiXFRM_POSITION))
	{
		for (int i = 0; i < 3; i++)
		{
			Envelope animationEnvelope;
			if (GetChannelKeyframeTimes(translationChannelNames[i], animationEnvelope, times) == LXe_OK && times.size() > 0)
			{
				positionAnimated = true;
				posAnimated[i] = true;
				posAnimationEnvelope[i] = new Envelope(animationEnvelope);
			}
		}
	}

	SetItem(item);

	if (HasXformItem(LXiXFRM_ROTATION) && XformItem(LXiXFRM_ROTATION))
	{
		for (int i = 0; i < 3; i++)
		{
			Envelope animationEnvelope;
			if (GetChannelKeyframeTimes(rotationChannelNames[i], animationEnvelope, times) == LXe_OK && times.size() > 0)
			{
				rotationAnimated = true;
				rotAnimated[i] = true;
				rotAnimationEnvelope[i] = new Envelope(animationEnvelope);
			}
		}
	}

	SetItem(item);

	if (HasXformItem(LXiXFRM_SCALE) && XformItem(LXiXFRM_SCALE))
	{
		for (int i = 0; i < 3; i++)
		{
			Envelope animationEnvelope;
			if (GetChannelKeyframeTimes(scaleChannelNames[i], animationEnvelope, times) == LXe_OK && times.size() > 0)
			{
				scaleAnimated = true;
				sclAnimated[i] = true;
				sclAnimationEnvelope[i] = new Envelope(animationEnvelope);
			}
		}
	}

	if ((!positionAnimated) && (!rotationAnimated) && (!scaleAnimated))
	{
		// If there's no key frame animation at all, then write the node transform as a single 4x4 matrix.
		// We might still be exporting sampled animation below.

		LXtMatrix4 outMatrix;
		GetItemTransform(item, outMatrix);

		IndentWrite("Transform");

		if (sampledAnimation)
		{
			Write(" %transform");
		}

		IndentWrite("{\n", 0, true);
		indentLevel++;

		IndentWrite("float[16]\n");
		IndentWrite("{\n");
		WriteMatrix(outMatrix);
		IndentWrite("}\n");

		indentLevel--;
		IndentWrite("}\n");
	}
	else
	{
		// If there is some key frame animation, then write the transform as factored position,
		// rotation, and scale followed by the non-animated object-offset transformation.

		static const char *const subtranslationName[3] =
		{
			"xpos", "ypos", "zpos"
		};

		static const char *const subrotationName[3] =
		{
			"xrot", "yrot", "zrot"
		};

		static const char *const subscaleName[3] =
		{
			"xscl", "yscl", "zscl"
		};

		static const char *const axisName[3] =
		{
			"x", "y", "z"
		};

		bool structFlag = false;

		int rotationAxis[3];
		LXtVector translation;
		LXtVector rotate;
		LXtVector scale;

		double shear[3];

		unsigned transformCount = itemGraph.Reverse(item);
		for (unsigned transformIndex = 0; transformIndex < transformCount; ++transformIndex)
		{
			CLxUser_Item transform;

			if (itemGraph.Reverse(item, transformIndex, transform))
			{
				SetItem(transform);

				// Item-level type (scale, translate, rotate, transform).

				const char *xformType = ItemType();

				// Sub-type (core, pivot, pivot comp, etc.)

				std::string transformType = GetTextHintEncodedChannelValue(*this, LXsICHAN_TRANSFORM_TYPE);

				if (std::string(xformType) == std::string(LXsITYPE_SCALE))
				{
					ChanXform(LXsICHAN_SCALE_SCL, scale);
				}
				else if (std::string(xformType) == std::string(LXsITYPE_TRANSLATION))
				{
					ChanXform(LXsICHAN_TRANSLATION_POS, translation);
				}
				else if (std::string(xformType) == std::string(LXsITYPE_ROTATION))
				{
					ChanXform(LXsICHAN_ROTATION_ROT, rotate);
					BuildAxisOrder(ChanInt(LXsICHAN_ROTATION_ORDER), rotationAxis);
				}
				else if (std::string(xformType) == std::string(LXsITYPE_SHEAR))
				{
					//shear[0] = ChanFloat(LXsICHAN_SHEAR_SHEARXY);
					//shear[1] = ChanFloat(LXsICHAN_SHEAR_SHEARXZ);
					//shear[2] = ChanFloat(LXsICHAN_SHEAR_SHEARYZ);

					//shear[0] = ChanFloat(LXsICHAN_QUATERNION_SHEARXY);
					//shear[1] = ChanFloat(LXsICHAN_QUATERNION_SHEARXY);
				}
				else if (std::string(xformType) == std::string(LXsITYPE_QUATERNION))
				{
				}
			}
		}

		if (positionAnimated)
		{
			// When the position is animated, write the x, y, and z components separately
			// so they can be targeted by different tracks having different sets of keys.

			for (int i = 0; i < 3; i++)
			{
				double pos = translation[i];

				if ((posAnimated[i]) || (fabs(pos) > kExportEpsilon))
				{
					IndentWrite("Translation %", 0, structFlag);
					Write(subtranslationName[i]);
					Write(" (kind = \"");
					Write(axisName[i]);
					Write("\")\n");
					IndentWrite("{\n");
					IndentWrite("float {", 1);
					WriteHexFloat(pos);
					Write("}\t\t// ");
					WriteFloat(pos);
					IndentWrite("}\n", 0, true);

					structFlag = true;
				}
			}
		}
		else if ((fabs(translation[0]) > kExportEpsilon) || (fabs(translation[1]) > kExportEpsilon) || (fabs(translation[2]) > kExportEpsilon))
		{
			IndentWrite("Translation\n");
			IndentWrite("{\n");
			IndentWrite("float[3] {", 1);
			WriteHexPoint3(translation);
			Write("}\t\t// ");
			WritePoint3(translation);
			IndentWrite("}\n", 0, true);

			structFlag = true;
		}

		if (rotationAnimated)
		{
			// When the rotation is animated, write three separate Euler angle rotations
			// so they can be targeted by different tracks having different sets of keys.
			
			for (int i = 2; i >= 0; i--)
			{
				int axis = rotationAxis[i];
				double angle = rotate[axis];

				if ((rotAnimated[axis]) || (fabs(angle) > kExportEpsilon))
				{
					IndentWrite("Rotation %", 0, structFlag);
					Write(subrotationName[axis]);
					Write(" (kind = \"");
					Write(axisName[axis]);
					Write("\")\n");
					IndentWrite("{\n");

					IndentWrite("float {", 1);
					WriteHexFloat(angle);
					Write("}\t\t// ");
					WriteFloat(angle);
					IndentWrite("}\n", 0, true);

					structFlag = true;
				}
			}
		}
		else
		{
			// When the rotation is not animated, write it as a single quaternion.

			LXtMatrix4 outMatrix;
			GetItemTransform(item, outMatrix);

			Quaternion quaternion;
			CalculateRotation(quaternion, outMatrix);

			IndentWrite("Rotation (kind = \"quaternion\")\n", 0, structFlag);
			IndentWrite("{\n");
			IndentWrite("float[4] {", 1);
			WriteHexQuaternion(quaternion);
			Write("}\t\t// ");
			WriteQuaternion(quaternion);
			IndentWrite("}\n", 0, true);

			structFlag = true;
		}

		bool shearPresent = false; // ((fabs(shear[0]) > kExportEpsilon) || (fabs(shear[1]) > kExportEpsilon) || (fabs(shear[2]) > kExportEpsilon));

		if (shearPresent)
		{
			LXtMatrix4 shearMatrix;

			lx::Matrix4Ident(shearMatrix);
			shearMatrix[1][0] = shear[0];
			shearMatrix[2][0] = shear[1];
			shearMatrix[2][1] = shear[2];

			IndentWrite("Transform\n", 0, structFlag);
			IndentWrite("{\n");
			indentLevel++;

			IndentWrite("float[16]\n");
			IndentWrite("{\n");
			WriteMatrix(shearMatrix);
			IndentWrite("}\n");

			indentLevel--;
			IndentWrite("}\n");

			structFlag = true;
		}

		if (scaleAnimated)
		{
			// When the scale is animated, write the x, y, and z components separately
			// so they can be targeted by different tracks having different sets of keys.

			for (int i = 0; i < 3; i++)
			{
				double scl = scale[i];

				if ((sclAnimated[i]) || (fabs(scl) - 1.0 > kExportEpsilon))
				{
					IndentWrite("Scale %", 0, structFlag);
					Write(subscaleName[i]);
					Write(" (kind = \"");
					Write(axisName[i]);
					Write("\")\n");
					IndentWrite("{\n");
					IndentWrite("float {", 1);
					WriteHexFloat(scl);
					Write("}\t\t// ");
					WriteFloat(scl);
					IndentWrite("}\n", 0, true);

					structFlag = true;
				}
			}
		}
		else if ((fabs(scale[0] - 1.0F) > kExportEpsilon) || (fabs(scale[1] - 1.0F) > kExportEpsilon) || (fabs(scale[2] - 1.0F) > kExportEpsilon))
		{
			IndentWrite("Scale\n", 0, structFlag);
			IndentWrite("{\n");
			IndentWrite("float[3] {", 1);
			WriteHexPoint3(scale);
			Write("}\t\t// ");
			WritePoint3(scale);
			IndentWrite("}\n", 0, true);

			structFlag = true;
		}

		// Export the animation tracks.

		std::set<float>::iterator it = times.begin();
		
		IndentWrite("Animation (begin = ", 0, true);
		WriteFloat(*it);
		Write(", end = ");
		it = times.end();
		--it;
		WriteFloat(*it);
		Write(")\n");
		IndentWrite("{\n");
		indentLevel++;

		structFlag = false;

		if (positionAnimated)
		{
			for (int i = 0; i < 3; i++)
			{
				if (posAnimated[i])
				{
					ExportAnimationTrack(*posAnimationEnvelope[i], subtranslationName[i], structFlag);
					structFlag = true;
				}
			}
		}

		if (rotationAnimated)
		{
			for (int i = 0; i < 3; i++)
			{
				if (rotAnimated[i])
				{
					ExportAnimationTrack(*rotAnimationEnvelope[i], subrotationName[i], structFlag);
					structFlag = true;
				}
			}
		}

		if (scaleAnimated)
		{
			for (int i = 0; i < 3; i++)
			{
				if (sclAnimated[i])
				{
					ExportAnimationTrack(*sclAnimationEnvelope[i], subscaleName[i], structFlag);
					structFlag = true;
				}
			}
		}

		indentLevel--;
		IndentWrite("}\n");
	}

	for (int i = 2; i >= 0; i--)
	{
		delete rotAnimationEnvelope[i];
		delete posAnimationEnvelope[i];
		delete sclAnimationEnvelope[i];
	}
}

void OpenGexExport::GetItemTransform(CLxLoc_Item& item, LXtMatrix4& matrix)
{
	LXtMatrix4 m;
	lx::Matrix4Ident(m);

	// For the matrix case, we need to reverse iterate over the transform graph.

	unsigned transformCount = itemGraph.Reverse(item);

	for (unsigned transformIndex = 0; transformIndex < transformCount; ++transformIndex)
	{
		CLxUser_Item transform;

		if (itemGraph.Reverse(item, transformIndex, transform))
		{
			SetItem(transform);

			const char *xformType = ItemType();

			if (std::string(xformType) == std::string(LXsITYPE_SCALE))
			{
				LXtVector	scale;
				ChanXform(LXsICHAN_SCALE_SCL, scale);
				LXtMatrix4 sm;

				lx::Matrix4Ident(sm);

				sm[0][0] = scale[0];
				sm[1][1] = scale[1];
				sm[2][2] = scale[2];

				lx::Matrix4Multiply(m, sm);
			}
			else if (std::string(xformType) == std::string(LXsITYPE_ROTATION))
			{
				LXtVector rotate;
				ChanXform(LXsICHAN_ROTATION_ROT, rotate);

				// Take rotation order into account.

				int axis[3];
				BuildAxisOrder(ChanInt(LXsICHAN_ROTATION_ORDER), axis);

				LXtMatrix4 rm;
				lx::Matrix4Ident(rm);
				Matrix4CreateRotation(rm, rotate[axis[0]], axis[0]);
				lx::Matrix4Multiply(m, rm);

				lx::Matrix4Ident(rm);
				Matrix4CreateRotation(rm, rotate[axis[1]], axis[1]);
				lx::Matrix4Multiply(m, rm);

				lx::Matrix4Ident(rm);
				Matrix4CreateRotation(rm, rotate[axis[2]], axis[2]);
				lx::Matrix4Multiply(m, rm);
			}
			else if (std::string(xformType) == std::string(LXsITYPE_TRANSLATION))
			{
				LXtVector	translate;
				ChanXform(LXsICHAN_TRANSLATION_POS, translate);

				LXtMatrix4 sm;
				lx::Matrix4Ident(sm);

				sm[3][0] = translate[0];
				sm[3][1] = translate[1];
				sm[3][2] = translate[2];

				lx::Matrix4Multiply(m, sm);
			}
			else if (std::string(xformType) == std::string(LXsITYPE_TRANSFORM))
			{
				LXtVector translate = { 0, 0, 0 };

				// Walk the local graph and find the translate item for which this transform is the inverse.

				CLxUser_Scene scene(SceneObject());
				CLxUser_SceneGraph  sceneGraph;
				scene.GetGraph(LXsGRAPH_XFRMLOCAL, sceneGraph);
				CLxUser_ItemGraph localGraph;
				localGraph.set(sceneGraph);
				CLxUser_Item localTransform;

				if (localGraph.Reverse(transform, 0, localTransform))
				{
					// Fetch the translation vector and negate it.

					SetItem(localTransform);

					if (ItemType() == std::string(LXsITYPE_TRANSLATION))
					{
						ChanXform(LXsICHAN_TRANSLATION_POS, translate);
						translate[0] = -translate[0];
						translate[1] = -translate[1];
						translate[2] = -translate[2];

						LXtMatrix4 sm;

						lx::Matrix4Ident(sm);

						sm[3][0] = translate[0];
						sm[3][1] = translate[1];
						sm[3][2] = translate[2];

						lx::Matrix4Multiply(m, sm);
					}
				}
			}
		}
	}

	lx::Matrix4Copy(matrix, m);
}

void OpenGexExport::ExportObjects()
{
	int count = (int)geometryArray->size();

	if (count != 0)
	{
		const ObjectReference *objectRef = &geometryArray->front();

		for (int i = 0; i < count; i++)
		{
			ExportGeometry(objectRef);
			objectRef++;
		}
	}

	count = (int)lightArray->size();

	if (count != 0)
	{
		const ObjectReference *objectRef = &lightArray->front();

		for (int i = 0; i < count; i++)
		{
			ExportLight(objectRef);
			objectRef++;
		}
	}

	count = (int)cameraArray->size();

	if (count != 0)
	{
		const ObjectReference *objectRef = &cameraArray->front();

		for (int i = 0; i < count; i++)
		{
			ExportCamera(objectRef);
			objectRef++;
		}
	}
}

std::string OpenGexExport::UVMapName(CLxUser_Item&	imageMap)
{
	std::string texcoordSet = std::string("ATTRVALUE_TEXCOORDSET0");
	SetItem(imageMap);

	if (TxtrLocator())
	{
		const char *uvMap = ChanString(LXsICHAN_TEXTURELOC_UVMAP);

		if (uvMap)
		{
			texcoordSet = std::string(uvMap);
		}
	}

	return (texcoordSet);
}

bool OpenGexExport::ExportTexture(CLxUser_Item& material, CLxUser_Item& locator, const char *materialName, const char *attrib)
{
	const char *fx;

	if (material.test() && SetItem(material) && TxtrImage() && (fx = ChanString(LXsICHAN_VIDEOSTILL_FILENAME)))
	{
		IndentWrite("Texture (attrib = \"", 0, true);
		Write(attrib);
		Write("\"");

		std::string mapName = UVMapName(material);
		const TextureReference *textureRef = FindTexture(mapName.c_str());

		if (textureRef)
		{
			Write(", texcoord = ");
			WriteInt(textureRef->texcoord);
		}

		Write(")\n");
		IndentWrite("{\n");
		indentLevel++;

		const std::string name = NativePathToURI(fx);
		IndentWrite("string {\"");
		WriteFileName(name.c_str());
		Write("\"}\n");

		if (locator)
		{
			LXtMatrix4 outMatrix;
			GetItemTransform(locator, outMatrix);

			double uscale = outMatrix[0][0];
			double vscale = outMatrix[1][1];
			double uoffset = outMatrix[3][0];
			double voffset = outMatrix[3][1];

			if ((uscale != 1.0F) || (vscale != 1.0F) || (uoffset != 0.0F) || (voffset != 0.0F))
			{
				IndentWrite("Transform\n", 0, true);
				IndentWrite("{\n");
				indentLevel++;

				IndentWrite("float[16]\n");
				IndentWrite("{\n");
				WriteMatrix(outMatrix);
				IndentWrite("}\n");

				indentLevel--;
				IndentWrite("}\n");
			}
		}

		indentLevel--;
		IndentWrite("}\n");

	}

	return (true);
}

TextureReference *OpenGexExport::FindTexture(const char *texture)
{
	int count = (int)textureArray->size();

	if (count != 0)
	{
		TextureReference *textureRef = &textureArray->front();

		for (int i = 0; i < count; i++)
		{
			if (textureRef->texture == texture)
			{
				return (textureRef);
			}

			textureRef++;
		}
	}

	return (nullptr);
}

void OpenGexExport::ExportMaterials()
{
	int count = (int)materialArray->size();
	bool doubleSided = false;

	if (count != 0)
	{
		const MaterialReference *materialRef = &materialArray->front();

		for (int i = 0; i < count; i++)
		{
			const char *fx;

			struct ItemMaterial
			{
				std::string materialName;
				std::string materialType;
				std::string locatorIdentity;
			};

			std::map<CLxUser_Item, ItemMaterial> itemMaps;
			std::map<CLxUser_Item, ItemMaterial> textureLocators;
			std::string material(materialRef->material);

			StartScan();
			CLxUser_Item matr;

			while (NextItem())
			{
				CLxUser_Item item;
				GetItem(item);

				CLxUser_Item parent;
				item.GetParent(parent);

				if (parent)
				{
					std::string parentName;
					parent.GetUniqueName(parentName);

					if (!std::strncmp(material.c_str(), parentName.c_str(), material.length()))
					{
						if (ItemIsA(LXsITYPE_IMAGEMAP))
						{
							fx = LayerEffect();

							ItemMaterial mat;
							mat.materialName = material;

							if (!strcmp(fx, LXs_FX_DIFFCOLOR))
							{
								mat.materialType = "diffuse";
							}
							else if (!strcmp(fx, LXs_FX_SPECCOLOR))
							{
								mat.materialType = "specular";
							}
							else if (!strcmp(fx, LXs_FX_TRANCOLOR))
							{
								mat.materialType = "transparency";
							}
							else if (!strcmp(fx, LXs_FX_LUMICOLOR))
							{
								mat.materialType = "emission";
							}
							else if (!strcmp(fx, LXs_FX_BUMP))
							{
								mat.materialType = "normal";
							}

							if (item && mat.materialType[0] != 0)
							{
								itemMaps.insert(std::pair<CLxUser_Item, ItemMaterial>(item, mat));
							}
						}
						else if (ItemIsA(LXsITYPE_ADVANCEDMATERIAL))
						{
							GetItem(matr);
						}
						else if (ItemIsA(LXsITYPE_CONSTANT))
						{
							bool t = true;
							t = false;
						}
					}
				}
				else
				{
					if (ItemIsA(LXsITYPE_TEXTURELOC) && i == 0)
					{
						GetItem(item);

						std::string identity = item.GetIdentity();
						std::string uvmap = ChanString(LXsICHAN_TEXTURELOC_UVMAP);

						ItemMaterial mat;
						mat.materialName = uvmap;
						mat.materialType = "locator";
						mat.locatorIdentity = identity;
						textureLocators.insert(std::pair<CLxUser_Item, ItemMaterial>(item, mat));
					}
				}
			}
			
			Write("\nMaterial $");
			Write(materialRef->structName.c_str());

			if (matr.test() && SetItem(matr))
			{
				doubleSided = ChanBool(LXsICHAN_ADVANCEDMATERIAL_DBLSIDED);
			}

			if (doubleSided)
			{
				Write(" (two_sided = true)");
			}

			Write("\n{\n");
			indentLevel++;

			IndentWrite("Name {string {\"");
			Write(material.c_str());
			Write("\"}}\n");

			for (std::map<CLxUser_Item, ItemMaterial>::iterator it = itemMaps.begin(); it != itemMaps.end(); ++it)
			{
				CLxUser_Item materialItem = it->first;
				ItemMaterial materialData = it->second;

				CLxUser_Item locator;
				std::string mapName = UVMapName(materialItem);
				const TextureReference *textureRef = FindTexture(mapName.c_str());

				if (textureRef)
				{
					for (std::map<CLxUser_Item, ItemMaterial>::iterator iter = textureLocators.begin(); iter != textureLocators.end(); ++iter)
					{
						CLxUser_Item locatorItem = iter->first;
						ItemMaterial locatorData = iter->second;

						if (!locatorData.materialName.compare(textureRef->texture))
						{
							locator = locatorItem;
							break;
						}
					}

					ExportTexture(materialItem, locator, materialData.materialName.c_str(), materialData.materialType.c_str());
				}
			}

			if (matr.test() && SetItem(matr))
			{
				double a = ChanFloat(LXsICHAN_ADVANCEDMATERIAL_DIFFAMT);

				LXtVector channelColor;
				ChanColor(LXsICHAN_ADVANCEDMATERIAL_DIFFCOL, channelColor);

				LXtFVector color;
				color[0] = (float)(channelColor[0] * a);
				color[1] = (float)(channelColor[1] * a);
				color[2] = (float)(channelColor[2] * a);

				IndentWrite("Color (attrib = \"diffuse\") {float[3] {", 0, true);
				WriteColor(color);
				Write("}}\n");

				a = ChanFloat(LXsICHAN_ADVANCEDMATERIAL_SPECAMT);
				ChanColor(LXsICHAN_ADVANCEDMATERIAL_SPECCOL, channelColor);

				color[0] = (float)(channelColor[0] * a);
				color[1] = (float)(channelColor[1] * a);
				color[2] = (float)(channelColor[2] * a);

				IndentWrite("Color (attrib = \"specular\") {float[3] {");
				WriteColor(color);
				Write("}}\n");

				IndentWrite("Param (attrib = \"specular_power\") {float {");
				WriteFloat(SpecExponent());
				Write("}}\n");

				a = ChanFloat(LXsICHAN_ADVANCEDMATERIAL_RADIANCE);
				ChanColor(LXsICHAN_ADVANCEDMATERIAL_LUMICOL, channelColor);

				color[0] = (float)(channelColor[0] * a);
				color[1] = (float)(channelColor[1] * a);
				color[2] = (float)(channelColor[2] * a);

				IndentWrite("Color (attrib = \"emission\") {float[3] {");
				WriteColor(color);
				Write("}}\n");

				a = ChanFloat(LXsICHAN_ADVANCEDMATERIAL_TRANAMT);
				ChanColor(LXsICHAN_ADVANCEDMATERIAL_TRANCOL, channelColor);

				color[0] = (float)(channelColor[0] * a);
				color[1] = (float)(channelColor[1] * a);
				color[2] = (float)(channelColor[2] * a);

				IndentWrite("Color (attrib = \"transparency\") {float[3] {");
				WriteColor(color);
				Write("}}\n");
			}

			indentLevel--;
			Write("}\n");

			materialRef++;
		}
	}
}

void OpenGexExport::ExportSkin(CLxUser_GroupDeformer& groupDeformer, unsigned groupDeformerMeshIndex, CLxUser_Item& item, int vertexCount, const OpenGex::ExportVertex *exportVertex)
{
	DeformerWeightToVertexVisitor visitor(item, &groupDeformer, groupDeformerMeshIndex, &jointNodeIDs);

	// Gather the Locators controlled by the group deformer
	
	visitor.FindLocators();
		
	// Generate a map for quick access to the indices of nodes by their IDs.

	unsigned jointIndex = 0;
	for (std::set<std::string>::const_iterator iter = jointNodeIDs.begin(); iter != jointNodeIDs.end(); ++iter)
	{
		visitor.nodeIndexMap[*iter] = jointIndex++;
	}
		
	// Visit the deformer/weight pairs for each point, and use the
	// locator array to generate the array of bind matrices.
	
	visitor.Enumerate();

	// This function exports all skinning data, which includes the skeleton
	// and per-vertex bone influence data.

	IndentWrite("Skin\n", 0, true);
	IndentWrite("{\n");
	indentLevel++;

	// Write the skin bind pose transform.

	IndentWrite("Transform\n");
	IndentWrite("{\n");
	indentLevel++;

	IndentWrite("float[16]\n");
	IndentWrite("{\n");

	LXtMatrix4 matrix;
	lx::Matrix4Copy(matrix, visitor.skinBindMatrix.m);
	WriteMatrix(matrix);
	IndentWrite("}\n");

	indentLevel--;
	IndentWrite("}\n\n");

	// Export the skeleton, which includes an array of bone node references
	// and an array of per-bone bind pose transforms.

	IndentWrite("Skeleton\n");
	IndentWrite("{\n");
	indentLevel++;

	// Write the bone node reference array.

	IndentWrite("BoneRefArray\n");
	IndentWrite("{\n");
	indentLevel++;

	unsigned int boneCount = (unsigned int)visitor.nodeIndexMap.size();

	IndentWrite("ref\t\t\t// ");
	WriteInt(boneCount);
	IndentWrite("{\n", 0, true);
	IndentWrite("", 1);

	for (std::map<std::string, unsigned int>::iterator it = visitor.nodeIndexMap.begin(); it != visitor.nodeIndexMap.end(); ++it)
	{
		std::string bone = it->first;
		const NodeReference *boneRef = FindNode(bone.c_str());

		if (boneRef)
		{
			Write("$");
			Write(boneRef->structName.c_str());
		}
		else
		{
			Write("null");
		}

		if (it->second < boneCount - 1)
		{
			Write(", ");
		}
		else
		{
			Write("\n");
		}
	}

	IndentWrite("}\n");

	indentLevel--;
	IndentWrite("}\n\n");

	// Write the bind pose transform array.

	IndentWrite("Transform\n");
	IndentWrite("{\n");
	indentLevel++;

	IndentWrite("float[16]\t// ");
	WriteInt(boneCount);
	IndentWrite("{\n", 0, true);

	for (std::map<std::string, unsigned int>::iterator it = visitor.nodeIndexMap.begin(); it != visitor.nodeIndexMap.end(); ++it)
	{
		WriteHexMatrixFlat(visitor.bindMatrices[it->second].m);

		if (it->second < boneCount - 1)
		{
			Write(",\n");
		}
		else
		{
			Write("\n");
		}
	}

	IndentWrite("}\n");

	indentLevel--;
	IndentWrite("}\n");

	indentLevel--;
	IndentWrite("}\n\n");

	// Export the per-vertex bone influence data.

	int weightCount = 0;
	int *countArray = new int[vertexCount];

	for (int i = 0; i < vertexCount; i++)
	{
		int count = visitor.boneCounts[exportVertex[i].index];
		countArray[i] = count;
		weightCount += count;
	}

	// Write the bone count array. There is one entry per vertex.

	IndentWrite("BoneCountArray\n");
	IndentWrite("{\n");
	indentLevel++;

	IndentWrite("unsigned_int16\t\t// ");
	WriteInt(vertexCount);
	IndentWrite("{\n", 0, true);
	WriteIntArray(vertexCount, countArray);
	IndentWrite("}\n");

	indentLevel--;
	IndentWrite("}\n\n");

	// Write the bone index array. The number of entries is the sum of the
	// bone counts for all vertices.

	IndentWrite("BoneIndexArray\n");
	IndentWrite("{\n");
	indentLevel++;

	// write out an array of start locations for each index as there are
	// multiple bone weights per index and we won't be starting at exportVertex[x].index 
	// equals zero for accessing the index and weights

	int *jointWeightIndicesStart = new int[vertexCount];
	int k = 0;
	for (int i = 0; i < vertexCount; i++)
	{
		jointWeightIndicesStart[i] = k;
		k += visitor.boneCounts[i] * 2;
	}

	// Now access each vertex but in the exportVertex[x].index order so
	// we use the jointWeightIndicesStart array to see where each vertex
	// data starts

	int *indexArray = new int[weightCount];
	int *index = indexArray;

	float *weightArray = new float[weightCount];
	float *weight = weightArray;

	for (int i = 0; i < vertexCount; i++)
	{
		int vertexIndex = (int)exportVertex[i].index;
		int startIndex = jointWeightIndicesStart[vertexIndex];
		int count = countArray[i];

		for (int j = 0; j < count; j++)
		{
			index[0] = (int)visitor.jointWeightIndices[j * 2 + startIndex];
			weight[0] = visitor.weights[(int)visitor.jointWeightIndices[j * 2 + startIndex + 1]];

			index++;
			weight++;
		}
	}

	IndentWrite("unsigned_int16\t\t// ");
	WriteInt(weightCount);
	IndentWrite("{\n", 0, true);
	WriteIntArray(weightCount, indexArray);
	IndentWrite("}\n");

	delete[] indexArray;

	indentLevel--;
	IndentWrite("}\n\n");

	// Write the bone weight array. The number of entries is the sum of the
	// bone counts for all vertices.

	IndentWrite("BoneWeightArray\n");
	IndentWrite("{\n");
	indentLevel++;

	IndentWrite("float\t\t// ");
	WriteInt(weightCount);
	IndentWrite("{\n", 0, true);
	WriteDoubleArray(weightCount, weightArray);
	IndentWrite("}\n");

	delete[] weightArray;
	delete[] jointWeightIndicesStart;

	indentLevel--;
	IndentWrite("}\n");

	delete[] countArray;

	indentLevel--;
	IndentWrite("}\n");
}

void OpenGexExport::ExportGeometry(const OpenGex::ObjectReference *objectRef)
{
	// This function exports a single geometry object.

	int	colorCount, texcoordCount;

	CLxUser_Scene scene(SceneObject());
	CLxUser_Item item;
	scene.GetItemByIdent(objectRef->object.c_str(), item);
	SetItem(item);

	CLxUser_Mesh		userMesh;
	CLxUser_MeshMap		meshMap;
	bool haveMeshMap = ChanObject(LXsICHAN_MESH_MESH, userMesh);

	if (haveMeshMap)
	{
		userMesh.GetMaps(meshMap);
	}

	if (meshMap)
	{
		meshMap.FilterByType(LXi_VMAP_MORPH);
		MeshMapVisitor morpVisitor(&meshMap);
		meshMap.Enum(&morpVisitor);
		morpVisitor.SortMapNames();

		morphMapNames = morpVisitor.GetMapNames();
	}

	pointIndex = 0;
	WritePoints();

	if (saveVertexNormals)
	{
		if (haveMeshMap)
		{
			meshMap.FilterByType(LXi_VMAP_NORMAL);
			MeshMapVisitor normalVisitor(&meshMap);
			meshMap.Enum(&normalVisitor);
			normalVisitor.SortMapNames();

			normalMapNames = normalVisitor.GetMapNames();
		}
	}

	if (saveUVTextureCoordinates)
	{
		if (haveMeshMap)
		{
			texcoordMaps.clear();

			meshMap.FilterByType(LXi_VMAP_TEXTUREUV);
			MeshMapVisitor uvVisitor(&meshMap);
			meshMap.Enum(&uvVisitor);
			uvVisitor.SortMapNames();

			uvMapNames = uvVisitor.GetMapNames();
		}
	}

	if (saveColors)
	{
		if (haveMeshMap)
		{
			// RGB
			meshMap.FilterByType(LXi_VMAP_RGB);
			MeshMapVisitor colorRGBVisitor(&meshMap);
			meshMap.Enum(&colorRGBVisitor);

			colorRGBVisitor.SortMapNames();
			colorRGBMapNames = colorRGBVisitor.GetMapNames();

			// RGBA
			meshMap.FilterByType(LXi_VMAP_RGBA);
			MeshMapVisitor colorRGBAVisitor(&meshMap);
			meshMap.Enum(&colorRGBAVisitor);

			colorRGBAVisitor.SortMapNames();
			colorRGBAMapNames = colorRGBAVisitor.GetMapNames();
		}

		saveColors = colorRGBMaps.size() > 0 || colorRGBAMaps.size() > 0;
	}

	if (PolygonCount())
	{
		materials.clear();
		materialIndexArray.clear();

		VisitPolygons(POLYPASS_BUILDMATERIALSET);
		VisitPolygons(POLYPASS_POLYGONS);
	}

	unsigned int polyType = kPolyTypeNone;

	if (AllTriangles())
	{
		polyType = kPolyTypeTriangle;
	}
	else if (prefs.GetSaveQuads() && AllQuads())
	{
		polyType = kPolyTypeQuad;
	}
	else if (prefs.GetTriangulateMesh())
	{
		polyType = kPolyTypeMixed;
	}

	if (polyType > kPolyTypeNone)
	{
		Write("\nGeometryObject $");
		Write(objectRef->structName.c_str());
		WriteNodeTable(objectRef);

		Write("\n{\n");
		indentLevel++;

		if (polyType == kPolyTypeTriangle || polyType == kPolyTypeMixed)
		{
			IndentWrite("Mesh (primitive = \"triangles\")\n");
		}
		else if (polyType == kPolyTypeQuad)
		{
			IndentWrite("Mesh (primitive = \"quads\")\n");
		}

		IndentWrite("{\n");
		indentLevel++;

		int multiplier = polyType == kPolyTypeQuad ? kPolyTypeQuad : kPolyTypeTriangle;
		int polygonCount = 0;
		ExportVertex *exportVertex = DeindexMesh(&polygonCount, polyType, this);
		ExportVertex *unifiedVertex = new ExportVertex[polygonCount * multiplier];
		int *indexTable = new int[polygonCount * multiplier];
		int unifiedCount = UnifyVertices(polygonCount * multiplier, exportVertex, unifiedVertex, indexTable);

		pointMap.clear();
		points.clear();

		if (saveUVTextureCoordinates)
		{
			texcoordCount = (int)uvMapNames.size();
			texcoordCount = min(texcoordCount, kMaxTexcoordCount);
		}

		if (saveColors)
		{
			int RGBColorCount = (int)colorRGBMapNames.size();
			int RGBAColorCount = (int)colorRGBAMapNames.size();

			colorCount = min(RGBColorCount + RGBAColorCount, kMaxVertexColorCount);
		}

		//Positions

		IndentWrite("VertexArray (attrib = \"position\")\n");
		IndentWrite("{\n");
		indentLevel++;

		IndentWrite("float[3]\t\t// ");
		WriteInt(unifiedCount);
		IndentWrite("{\n", 0, true);
		WriteVertexArray(unifiedCount, &unifiedVertex->position, sizeof(ExportVertex));
		IndentWrite("}\n");

		indentLevel--;
		IndentWrite("}\n\n");

		if (saveVertexNormals)
		{
			// Write the normal array.

			IndentWrite("VertexArray (attrib = \"normal\")\n");
			IndentWrite("{\n");
			indentLevel++;

			IndentWrite("float[3]\t\t// ");
			WriteInt(unifiedCount);
			IndentWrite("{\n", 0, true);
			WriteVertexArray(unifiedCount, &unifiedVertex->normal, sizeof(ExportVertex));
			IndentWrite("}\n");

			indentLevel--;
			IndentWrite("}\n");
		}

		if (saveColors)
		{
			// Write out the vertex colors

			for (int k = 0; k < colorCount; k++)
			{
				IndentWrite("VertexArray (attrib = \"color", 0, true);

				if (k != 0)
				{
					Write("[");
					WriteInt(k);
					Write("]");
				}

				Write("\")\n");

				IndentWrite("{\n");
				indentLevel++;

				IndentWrite("float[3]\t\t// ");
				WriteInt(unifiedCount);
				IndentWrite("{\n", 0, true);
				WriteVertexArray(unifiedCount, &unifiedVertex->color[k], sizeof(ExportVertex));
				IndentWrite("}\n");

				indentLevel--;
				IndentWrite("}\n");
			}
		}

		if (saveUVTextureCoordinates)
		{
			// Write the texcoord arrays.

			for (int k = 0; k < texcoordCount; k++)
			{
				IndentWrite("VertexArray (attrib = \"texcoord", 0, true);

				if (k != 0)
				{
					Write("[");
					WriteInt(k);
					Write("]");
				}

				Write("\")\n");

				IndentWrite("{\n");
				indentLevel++;

				IndentWrite("float[2]\t\t// ");
				WriteInt(unifiedCount);
				IndentWrite("{\n", 0, true);
				WriteVertexArray(unifiedCount, &unifiedVertex->texcoord[k], sizeof(ExportVertex));
				IndentWrite("}\n");

				indentLevel--;
				IndentWrite("}\n");
			}
		}

		// Write the index arrays.

		unsigned int maxMatID = (int)materials.size();

		if (maxMatID == 0)
		{
			// There is only one material, so write a single index array.

			IndentWrite("IndexArray\n", 0, true);
			IndentWrite("{\n");
			indentLevel++;

			IndentWrite("unsigned_int32[3]\t\t// ");
			WriteInt(polygonCount);
			IndentWrite("{\n", 0, true);

			if (polyType == kPolyTypeTriangle || polyType == kPolyTypeMixed)
			{
				WriteTriangleArray(polygonCount, indexTable);
			}
			else if (polyType == kPolyTypeQuad)
			{
				WriteQuadArray(polygonCount, indexTable);
			}

			IndentWrite("}\n");

			indentLevel--;
			IndentWrite("}\n");
		}
		else
		{
			// If there are multiple material IDs, then write a separate
			// index array for each one.

			int *materialPolygonCount = new int[maxMatID + 1];
			memset(materialPolygonCount, 0, (maxMatID + 1) * sizeof(int));

			for (int i = 0; i < polygonCount; i++)
			{
				std::string materialTag = materialIndexArray[i];
				auto materialIndex = find(materials.begin(), materials.end(), materialTag) - materials.begin();

				materialPolygonCount[materialIndex]++;
			}

			int *materialIndexTable = new int[polygonCount * multiplier];

			for (unsigned int m = 0; m <= maxMatID; m++)
			{
				if (materialPolygonCount[m] != 0)
				{
					int materialIndexCount = 0;
					for (int i = 0; i < polygonCount; i++)
					{
						if (materialIndexArray[i] == materials[m])
						{
							int k = i * multiplier;

							for (int j = 0; j < multiplier; j++)
							{
								materialIndexTable[materialIndexCount + j] = indexTable[k + j];
							}

							materialIndexCount += multiplier;
						}
					}

					IndentWrite("IndexArray (material = ", 0, true);
					WriteInt((int)m);
					Write(")\n");
					IndentWrite("{\n");
					indentLevel++;

					IndentWrite("unsigned_int32[3]\t\t// ");
					WriteInt(materialPolygonCount[m]);
					IndentWrite("{\n", 0, true);

					if (polyType == kPolyTypeTriangle || polyType == kPolyTypeMixed)
					{
						WriteTriangleArray(materialPolygonCount[m], materialIndexTable);
					}
					else if (polyType == kPolyTypeQuad)
					{
						WriteQuadArray(materialPolygonCount[m], materialIndexTable);
					}

					IndentWrite("}\n");

					indentLevel--;
					IndentWrite("}\n");
				}
			}

			delete[] materialIndexTable;
			delete[] materialPolygonCount;
		}

		// If the mesh is skinned, export the skinning data here.

		CLxUser_GroupDeformer groupDeformer;
		unsigned groupDeformerMeshIndex;

		if (FindGroupDeformer(item, groupDeformer, groupDeformerMeshIndex))
		{
			ExportSkin(groupDeformer, groupDeformerMeshIndex, item, unifiedCount, unifiedVertex);
		}

		bool allTexcoords = SetSelMap(LXi_VMAP_TEXTUREUV);

		if (!allTexcoords)
		{
			allTexcoords = SetMap(LXi_VMAP_TEXTUREUV);
		}

		delete[] indexTable;
		delete[] unifiedVertex;
		delete[] exportVertex;

		indentLevel--;
		IndentWrite("}\n");

		indentLevel--;
		Write("}\n");
	}
}

void OpenGexExport::ExportLight(const OpenGex::ObjectReference *objectRef)
{
	// This function exports a single light object.

	StartScan(LXsITYPE_POLYRENDER);

	LXtFVector color;
	double intensity = 1.0F;

	while (NextItem())
	{
		intensity = ChanFloat(LXsICHAN_RENDER_AMBRAD);

		LXtVector ambientColor;
		ChanColor(LXsICHAN_RENDER_AMBCOLOR, ambientColor);

		color[0] = (float)(ambientColor[0]);
		color[1] = (float)(ambientColor[1]);
		color[2] = (float)(ambientColor[2]);

		break;
	}

	Write("\nLightObject $");
	Write(objectRef->structName.c_str());

	CLxUser_Scene scene(SceneObject());
	CLxLoc_Item item;
	scene.GetItemByIdent(objectRef->object.c_str(), item);
	SetItem(item);

	bool pointFlag = false;
	bool spotFlag = false;
	Write(" (type = ");

	double decay = 0.0F;
	double coneAngle = 0.0F;
	double dropOff = 0.0F;

	if (ItemIsA(LXsITYPE_CYLINDERLIGHT))
	{
		// Not sure what to default to
	}
	else if (ItemIsA(LXsITYPE_AREALIGHT))
	{
		Write("\"infinite\"");
	}
	else if (ItemIsA(LXsITYPE_DOMELIGHT))
	{
		// Not sure what to default to
	}
	else if (ItemIsA(LXsITYPE_POINTLIGHT))
	{
		decay = ChanFloat(LXsICHAN_POINTLIGHT_VDISSOLVE);

		Write("\"point\"");
		pointFlag = true;
	}
	else if (ItemIsA(LXsITYPE_SPOTLIGHT))
	{
		decay = ChanFloat(LXsICHAN_SPOTLIGHT_VDISSOLVE);
		coneAngle = ChanFloat(LXsICHAN_SPOTLIGHT_CONE);
		dropOff = ChanFloat(LXsICHAN_SPOTLIGHT_EDGE);

		Write("\"spot\"");
		pointFlag = true;
		spotFlag = true;
	}
	else if (ItemIsA(LXsITYPE_SUNLIGHT))
	{
		Write("\"infinite\"");
	}
	else
	{
		Write("\"infinite\"");
	}

	//if (!light.useDepthMapShadows())
	//{
	//	Write(", shadow = false");
	//}

	Write(")");
	WriteNodeTable(objectRef);

	Write("\n{\n");
	indentLevel++;

	// Export the light's color, and include a separate intensity if necessary.

	IndentWrite("Color (attrib = \"light\") {float[3] {");
	WriteColor(color);
	Write("}}\n");

	if (intensity != 1.0F)
	{
		IndentWrite("Param (attrib = \"intensity\") {float {");
		WriteFloat(intensity);
		Write("}}\n");
	}

	if (pointFlag)
	{
		// Export an attenuation function.

		if (decay != 0)
		{
			IndentWrite("Atten (curve = \"", 0, true);
			Write((decay == 1.0F) ? "inverse\")\n" : "inverse_square\")\n");
			IndentWrite("{\n");

			IndentWrite("Param (attrib = \"scale\") {float {", 1);
			WriteFloat(1.0F);
			Write("}}\n");

			IndentWrite("}\n");
		}

		if (spotFlag)
		{
			// Export additional angular attenuation for spot lights.

			IndentWrite("Atten (kind = \"cos_angle\", curve = \"linear\")\n", 0, true);
			IndentWrite("{\n");

			IndentWrite("Param (attrib = \"begin\") {float {", 1);
			WriteFloat(0.0F);
			Write("}}\n");

			IndentWrite("Param (attrib = \"end\") {float {", 1);
			WriteFloat(cos(coneAngle * 0.5F));
			Write("}}\n");

			IndentWrite("Param (attrib = \"power\") {float {", 1);
			WriteFloat(dropOff);
			Write("}}\n");

			IndentWrite("}\n");
		}
	}

	indentLevel--;
	Write("}\n");
}

void OpenGexExport::ExportCamera(const OpenGex::ObjectReference *objectRef)
{
	// This function exports a single camera object.

	Write("\nCameraObject $");
	Write(objectRef->structName.c_str());
	WriteNodeTable(objectRef);

	Write("\n{\n");
	indentLevel++;

	CLxUser_Scene scene(SceneObject());
	CLxLoc_Item item;
	scene.GetItemByIdent(objectRef->object.c_str(), item);
	SetItem(item);

	double focalLength = ChanFloat(LXsICHAN_CAMERA_FOCALLEN);

	double xSize = ChanFloat(LXsICHAN_CAMERA_APERTUREX);
	double ySize = ChanFloat(LXsICHAN_CAMERA_APERTUREY);

	double zNear = prefs.GetZNear();
	double zFar = prefs.GetZFar();

	double xfov = 2.0F * atan(xSize / (2.0F * focalLength)) * kRadToDeg;
	double yfov = 2.0F * atan(ySize / (2.0F * focalLength)) * kRadToDeg;

	IndentWrite("Param (attrib = \"fov\") {float {");
	WriteFloat(xfov);
	Write("}}\n");

	IndentWrite("Param (attrib = \"near\") {float {");
	WriteFloat(zNear);
	Write("}}\n");

	IndentWrite("Param (attrib = \"far\") {float {");
	WriteFloat(zFar);
	Write("}}\n");

	indentLevel--;
	Write("}\n");
}

bool OpenGexExport::AllTriangles() const
{
	bool allTriangles(true);

	for (std::vector<unsigned>::const_iterator iter = polyCounts.begin(); iter != polyCounts.end(); ++iter)
	{
		if (*iter != 3)
		{
			allTriangles = false;
			break;
		}
	}

	return (allTriangles);
}

bool OpenGexExport::AllQuads() const
{
	bool allQuads(true);

	for (std::vector<unsigned>::const_iterator iter = polyCounts.begin(); iter != polyCounts.end(); ++iter)
	{
		if (*iter != 4)
		{
			allQuads = false;
			break;
		}
	}

	return (allQuads);
}

NodeReference *OpenGexExport::FindNode(const char *node) const
{
	int count = (int)nodeArray->size();

	if (count != 0)
	{
		NodeReference *nodeRef = &nodeArray->front();

		for (int i = 0; i < count; i++)
		{
			if (nodeRef->node.compare(node) == 0)
			{
				return (nodeRef);
			}

			nodeRef++;
		}
	}

	return (nullptr);
}

ObjectReference *OpenGexExport::FindObject(std::vector<ObjectReference> *array, const char *object)
{
	int count = (int)array->size();

	if (count != 0)
	{
		ObjectReference *objectRef = &array->front();

		for (int i = 0; i < count; i++)
		{
			if (objectRef->object == object)
			{
				return (objectRef);
			}

			objectRef++;
		}
	}

	return (nullptr);
}

MaterialReference *OpenGexExport::FindMaterial(const char *ident)
{
	int count = (int)materialArray->size();

	if (count != 0)
	{
		MaterialReference *materialRef = &materialArray->front();

		for (int i = 0; i < count; i++)
		{
			if (materialRef->material == ident)
			{
				return (materialRef);
			}

			materialRef++;
		}
	}

	return (nullptr);
}

void OpenGexExport::Write(const void *buffer, unsigned int size)
{
	std::string data(reinterpret_cast<const char *>(buffer), size);
	lf_Output(data);
}

void OpenGexExport::Write(const char *string)
{
	const char *s = string;

	while (*s != 0)
	{
		s++;
	}

	if (s != string)
	{
		Write(string, (unsigned int)(s - string));
	}
}

void OpenGexExport::Write(const wchar_t *string)
{
	// Convert a UTF-16 string to UTF-8, and write it to the file.

	const wchar_t *s = string;

	while (*s != 0)
	{
		s++;
	}

	if (s != string)
	{
		unsigned int length = (unsigned int)(s - string);
		char *buffer = new char[length * 4];

		int size = 0;

		for (unsigned int i = 0; i < length; i++)
		{
			unsigned int code = string[i] & 0xFFFF;

			if (code <= 0x007F)
			{
				if (code == 34)
				{
					buffer[size] = 92;
					buffer[size + 1] = 34;
					size += 2;
				}
				else if (code == 92)
				{
					buffer[size] = 92;
					buffer[size + 1] = 92;
					size += 2;
				}
				else
				{
					buffer[size] = (char)code;
					size += 1;
				}
			}
			else if (code <= 0x07FF)
			{
				buffer[size] = (char)(((code >> 6) & 0x1F) | 0xC0);
				buffer[size + 1] = (char)((code & 0x3F) | 0x80);
				size += 2;
			}
			else
			{
				unsigned int p1 = code - 0xD800;

				if (p1 < 0x0400U)
				{
					unsigned int p2 = (string[i + 1] & 0xFFFF) - 0xDC00;

					if (p2 < 0x0400U)
					{
						code = ((p1 << 10) | p2) + 0x010000;
						i++;
					}
				}

				if (code <= 0x00FFFF)
				{
					buffer[size] = (char)(((code >> 12) & 0x0F) | 0xE0);
					buffer[size + 1] = (char)(((code >> 6) & 0x3F) | 0x80);
					buffer[size + 2] = (char)((code & 0x3F) | 0x80);
					size += 3;
				}
				else
				{
					buffer[size] = (char)(((code >> 18) & 0x07) | 0xF0);
					buffer[size + 1] = (char)(((code >> 12) & 0x3F) | 0x80);
					buffer[size + 2] = (char)(((code >> 6) & 0x3F) | 0x80);
					buffer[size + 3] = (char)((code & 0x3F) | 0x80);
					size += 4;
				}
			}
		}

		Write(buffer, size);
		delete[] buffer;
	}
}

void OpenGexExport::IndentWrite(const char *string, int extra, bool newline)
{
	static const char tabs[16] = { 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9 };

	if (newline)
	{
		Write("\n");
	}

	int indent = indentLevel + extra;

	int count = indent >> 4;

	for (int i = 0; i < count; i++)
	{
		Write(tabs, 16);
	}

	indent &= 15;

	if (indent != 0)
	{
		Write(tabs, indent);
	}

	Write(string);
}

void OpenGexExport::WriteInt(int i)
{
	std::ostringstream	string;

	string << i;
	Write(string.str().c_str());
}

void OpenGexExport::WriteUnsignedInt(unsigned int i)
{
	std::ostringstream	string;

	string << i;
	Write(string.str().c_str());
}

void OpenGexExport::WriteFloat(double d)
{
	std::ostringstream	string;

	float f = (float)d;
	string << f;
	Write(string.str().c_str());
}

void OpenGexExport::WriteFloat(float f)
{
	std::ostringstream	string;

	string << f;
	Write(string.str().c_str());
}

void OpenGexExport::WriteHexFloat(double d)
{
	static const char hexdigit[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

	static char string[11] = "0x00000000";

	float f = (float)d;
	unsigned int i = *(unsigned int *)&f;
	string[2] = hexdigit[(i >> 28) & 0x0F];
	string[3] = hexdigit[(i >> 24) & 0x0F];
	string[4] = hexdigit[(i >> 20) & 0x0F];
	string[5] = hexdigit[(i >> 16) & 0x0F];
	string[6] = hexdigit[(i >> 12) & 0x0F];
	string[7] = hexdigit[(i >> 8) & 0x0F];
	string[8] = hexdigit[(i >> 4) & 0x0F];
	string[9] = hexdigit[i & 0x0F];

	Write(string);
}

void OpenGexExport::WriteHexFloat(float f)
{
	static const char hexdigit[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

	static char string[11] = "0x00000000";

	unsigned int i = *(unsigned int *)&f;
	string[2] = hexdigit[(i >> 28) & 0x0F];
	string[3] = hexdigit[(i >> 24) & 0x0F];
	string[4] = hexdigit[(i >> 20) & 0x0F];
	string[5] = hexdigit[(i >> 16) & 0x0F];
	string[6] = hexdigit[(i >> 12) & 0x0F];
	string[7] = hexdigit[(i >> 8) & 0x0F];
	string[8] = hexdigit[(i >> 4) & 0x0F];
	string[9] = hexdigit[i & 0x0F];

	Write(string);
}

void OpenGexExport::WriteMatrix(const LXtMatrix4& matrix)
{
	IndentWrite("{", 1);
	WriteHexFloat(matrix[0][0]);
	Write(", ");
	WriteHexFloat(matrix[0][1]);
	Write(", ");
	WriteHexFloat(matrix[0][2]);
	Write(", 0x00000000,\t\t// {");
	WriteFloat(matrix[0][0]);
	Write(", ");
	WriteFloat(matrix[0][1]);
	Write(", ");
	WriteFloat(matrix[0][2]);
	Write(", 0\n");

	IndentWrite(" ", 1);
	WriteHexFloat(matrix[1][0]);
	Write(", ");
	WriteHexFloat(matrix[1][1]);
	Write(", ");
	WriteHexFloat(matrix[1][2]);
	Write(", 0x00000000,\t\t//  ");
	WriteFloat(matrix[1][0]);
	Write(", ");
	WriteFloat(matrix[1][1]);
	Write(", ");
	WriteFloat(matrix[1][2]);
	Write(", 0\n");

	IndentWrite(" ", 1);
	WriteHexFloat(matrix[2][0]);
	Write(", ");
	WriteHexFloat(matrix[2][1]);
	Write(", ");
	WriteHexFloat(matrix[2][2]);
	Write(", 0x00000000,\t\t//  ");
	WriteFloat(matrix[2][0]);
	Write(", ");
	WriteFloat(matrix[2][1]);
	Write(", ");
	WriteFloat(matrix[2][2]);
	Write(", 0\n");

	IndentWrite(" ", 1);
	WriteHexFloat(matrix[3][0]);
	Write(", ");
	WriteHexFloat(matrix[3][1]);
	Write(", ");
	WriteHexFloat(matrix[3][2]);
	Write(", 0x3F800000}\t\t//  ");
	WriteFloat(matrix[3][0]);
	Write(", ");
	WriteFloat(matrix[3][1]);
	Write(", ");
	WriteFloat(matrix[3][2]);
	Write(", 1}\n");
}

void OpenGexExport::WriteMatrixFlat(const LXtMatrix4& matrix)
{
	IndentWrite("{", 1);
	WriteFloat(matrix[0][0]);
	Write(", ");
	WriteFloat(matrix[0][1]);
	Write(", ");
	WriteFloat(matrix[0][2]);
	Write(", 0, ");

	WriteFloat(matrix[1][0]);
	Write(", ");
	WriteFloat(matrix[1][1]);
	Write(", ");
	WriteFloat(matrix[1][2]);
	Write(", 0, ");

	WriteFloat(matrix[2][0]);
	Write(", ");
	WriteFloat(matrix[2][1]);
	Write(", ");
	WriteFloat(matrix[2][2]);
	Write(", 0, ");

	WriteFloat(matrix[3][0]);
	Write(", ");
	WriteFloat(matrix[3][1]);
	Write(", ");
	WriteFloat(matrix[3][2]);
	Write(", 1}");
}

void OpenGexExport::WriteHexMatrixFlat(const LXtMatrix4& matrix)
{
	IndentWrite("{", 1);
	WriteHexFloat(matrix[0][0]);
	Write(", ");
	WriteHexFloat(matrix[0][1]);
	Write(", ");
	WriteHexFloat(matrix[0][2]);
	Write(", 0x00000000, ");

	WriteHexFloat(matrix[1][0]);
	Write(", ");
	WriteHexFloat(matrix[1][1]);
	Write(", ");
	WriteHexFloat(matrix[1][2]);
	Write(", 0x00000000, ");

	WriteHexFloat(matrix[2][0]);
	Write(", ");
	WriteHexFloat(matrix[2][1]);
	Write(", ");
	WriteHexFloat(matrix[2][2]);
	Write(", 0x00000000, ");

	WriteHexFloat(matrix[3][0]);
	Write(", ");
	WriteHexFloat(matrix[3][1]);
	Write(", ");
	WriteHexFloat(matrix[3][2]);
	Write(", 0x3F800000}");
}

void OpenGexExport::WritePoint3(const LXtFVector& point)
{
	Write("{");
	WriteFloat(point[0]);
	Write(", ");
	WriteFloat(point[1]);
	Write(", ");
	WriteFloat(point[2]);
	Write("}");
}

void OpenGexExport::WritePoint3(const LXtVector& point)
{
	Write("{");
	WriteFloat(point[0]);
	Write(", ");
	WriteFloat(point[1]);
	Write(", ");
	WriteFloat(point[2]);
	Write("}");
}

void OpenGexExport::WriteHexPoint3(const LXtFVector& point)
{
	Write("{");
	WriteHexFloat(point[0]);
	Write(", ");
	WriteHexFloat(point[1]);
	Write(", ");
	WriteHexFloat(point[2]);
	Write("}");
}

void OpenGexExport::WriteHexPoint3(const LXtVector& point)
{
	Write("{");
	WriteHexFloat(point[0]);
	Write(", ");
	WriteHexFloat(point[1]);
	Write(", ");
	WriteHexFloat(point[2]);
	Write("}");
}

void OpenGexExport::WriteQuaternion(const Quaternion& quat)
{
	Write("{");
	WriteFloat(quat[0]);
	Write(", ");
	WriteFloat(quat[1]);
	Write(", ");
	WriteFloat(quat[2]);
	Write(", ");
	WriteFloat(quat[3]);
	Write("}");
}

void OpenGexExport::WriteHexQuaternion(const Quaternion& quat)
{
	Write("{");
	WriteHexFloat(quat[0]);
	Write(", ");
	WriteHexFloat(quat[1]);
	Write(", ");
	WriteHexFloat(quat[2]);
	Write(", ");
	WriteHexFloat(quat[3]);
	Write("}");
}

void OpenGexExport::WriteColor(const LXtFVector& color)
{
	Write("{");
	WriteFloat(color[0]);
	Write(", ");
	WriteFloat(color[1]);
	Write(", ");
	WriteFloat(color[2]);
	Write("}");
}

void OpenGexExport::WriteFileName(const char *string)
{
	const char *s = string;

	while (*s != 0)
	{
		s++;
	}

	if (s != string)
	{
		unsigned int length = (unsigned int)(s - string);
		char *buffer = new char[length + 3];

		unsigned int i = 0;

		if ((length >= 2) && (string[1] == ':'))
		{
			buffer[0] = '/';
			buffer[1] = '/';
			buffer[2] = string[0];
			string += 2;
			i = 3;
		}

		for (;; string++)
		{
			char c = string[0];

			if (c == 0)
			{
				break;
			}

			if (c == '\\')
			{
				c = '/';
			}

			buffer[i] = c;
			i++;
		}

		buffer[i] = 0;
		Write(buffer);
		delete[] buffer;
	}
}

void OpenGexExport::WriteIntArray(int count, const int *value)
{
	int lineCount = count >> 6;

	for (int i = 0; i < lineCount; i++)
	{
		IndentWrite("", 1);

		for (int j = 0; j < 63; j++)
		{
			WriteInt(value[j]);
			Write(", ");
		}

		WriteInt(value[63]);
		value += 64;

		if (i * 64 < count - 64)
		{
			Write(",\n");
		}
		else
		{
			Write("\n");
		}
	}

	count &= 63;

	if (count != 0)
	{
		IndentWrite("", 1);

		for (int j = count - 2; j >= 0; j--)
		{
			WriteInt(value[0]);
			Write(", ");
			value++;
		}

		WriteInt(value[0]);
		Write("\n");
	}
}

void OpenGexExport::WriteDoubleArray(int count, const float *value)
{
	int lineCount = count >> 4;

	for (int i = 0; i < lineCount; i++)
	{
		IndentWrite("", 1);

		for (int j = 0; j < 15; j++)
		{
			WriteHexFloat(value[j]);
			Write(", ");
		}

		WriteHexFloat(value[15]);
		value += 16;

		if (i * 16 < count - 16)
		{
			Write(",\n");
		}
		else
		{
			Write("\n");
		}
	}

	count &= 15;

	if (count != 0)
	{
		IndentWrite("", 1);

		for (int j = count - 2; j >= 0; j--)
		{
			WriteHexFloat(value[0]);
			Write(", ");
			value++;
		}

		WriteHexFloat(value[0]);
		Write("\n");
	}
}

void OpenGexExport::WriteVertex(const LXtFVector2& vertex)
{
	Write("{");
	WriteHexFloat(vertex[0]);
	Write(", ");
	WriteHexFloat(vertex[1]);
	Write("}");
}

void OpenGexExport::WriteVertex(const LXtFVector& vertex)
{
	Write("{");
	WriteHexFloat(vertex[0]);
	Write(", ");
	WriteHexFloat(vertex[1]);
	Write(", ");
	WriteHexFloat(vertex[2]);
	Write("}");
}

template <class type> void OpenGexExport::WriteVertexArray(int count, const type *vertex, int stride)
{
	int lineCount = count >> 3;

	for (int i = 0; i < lineCount; i++)
	{
		IndentWrite("", 1);

		for (int j = 0; j < 7; j++)
		{
			WriteVertex(vertex[0]);
			Write(", ");
			vertex = reinterpret_cast<const type *>(reinterpret_cast<const char *>(vertex) + stride);
		}

		WriteVertex(vertex[0]);
		vertex = reinterpret_cast<const type *>(reinterpret_cast<const char *>(vertex) + stride);

		if (i * 8 < count - 8)
		{
			Write(",\n");
		}
		else
		{
			Write("\n");
		}
	}

	count &= 7;

	if (count != 0)
	{
		IndentWrite("", 1);

		for (int j = count - 2; j >= 0; j--)
		{
			WriteVertex(vertex[0]);
			Write(", ");
			vertex = reinterpret_cast<const type *>(reinterpret_cast<const char *>(vertex) + stride);
		}

		WriteVertex(vertex[0]);
		Write("\n");
	}
}

void OpenGexExport::WriteTriangle(int triangleIndex, const int *indexTable)
{
	int i = triangleIndex * 3;

	Write("{");
	WriteUnsignedInt(indexTable[i]);
	Write(", ");
	WriteUnsignedInt(indexTable[i + 1]);
	Write(", ");
	WriteUnsignedInt(indexTable[i + 2]);
	Write("}");
}

void OpenGexExport::WriteTriangleArray(int count, const int *indexTable)
{
	int triangleIndex = 0;

	int lineCount = count >> 4;

	for (int i = 0; i < lineCount; i++)
	{
		IndentWrite("", 1);

		for (int j = 0; j < 15; j++)
		{
			WriteTriangle(triangleIndex, indexTable);
			Write(", ");
			triangleIndex++;
		}

		WriteTriangle(triangleIndex, indexTable);
		triangleIndex++;

		if (i * 16 < count - 16)
		{
			Write(",\n");
		}
		else
		{
			Write("\n");
		}
	}

	count &= 15;

	if (count != 0)
	{
		IndentWrite("", 1);

		for (int j = count - 2; j >= 0; j--)
		{
			WriteTriangle(triangleIndex, indexTable);
			Write(", ");
			triangleIndex++;
		}

		WriteTriangle(triangleIndex, indexTable);
		Write("\n");
	}
}

void OpenGexExport::WriteQuad(int quadIndex, const int *indexTable)
{
	int i = quadIndex * 4;

	Write("{");
	WriteUnsignedInt(indexTable[i]);
	Write(", ");
	WriteUnsignedInt(indexTable[i + 1]);
	Write(", ");
	WriteUnsignedInt(indexTable[i + 2]);
	Write(", ");
	WriteUnsignedInt(indexTable[i + 3]);
	Write("}");
}

void OpenGexExport::WriteQuadArray(int count, const int *indexTable)
{
	int quadIndex = 0;

	int lineCount = count >> 4;

	for (int i = 0; i < lineCount; i++)
	{
		IndentWrite("", 1);

		for (int j = 0; j < 15; j++)
		{
			WriteQuad(quadIndex, indexTable);
			Write(", ");
			quadIndex++;
		}

		WriteQuad(quadIndex, indexTable);
		quadIndex++;

		if (i * 16 < count - 16)
		{
			Write(",\n");
		}
		else
		{
			Write("\n");
		}
	}

	count &= 15;

	if (count != 0)
	{
		IndentWrite("", 1);

		for (int j = count - 2; j >= 0; j--)
		{
			WriteQuad(quadIndex, indexTable);
			Write(", ");
			quadIndex++;
		}

		WriteQuad(quadIndex, indexTable);
		Write("\n");
	}
}

void OpenGexExport::WriteNodeTable(const ObjectReference *objectRef)
{
	bool first = true;
	int nodeCount = (int)objectRef->nodeTable.size();
	const std::string *nodeTable = &objectRef->nodeTable.front();
	for (int k = 0; k < nodeCount; k++)
	{
		const char *name = nodeTable[k].c_str();
		if ((name) && (name[0] != 0))
		{
			if (first)
			{
				Write("\t\t// ");
			}
			else
			{
				Write(", ");
			}

			Write(name);
			first = false;
		}
	}
}

ExportVertex *OpenGexExport::DeindexMesh(int *exportPolyCount, unsigned int polyType, void *data)
{
	OpenGexExport *gex = static_cast<OpenGexExport *>(data);
	int polyCount = (int)gex->polyCounts.size();
	*exportPolyCount = polyCount;

	int start = 0;
	unsigned int polygonVertexCount = polyType == kPolyTypeQuad ? kPolyTypeQuad : kPolyTypeTriangle;
	int exportCount = polyCount * polygonVertexCount;

	// As MODO seems so bad at triangulating meshes, I've added this to be able to simply
	// triangulate a mesh. For example in a quad (if we are triangulating it), it 
	// takes the index [0] as the first index and then takes 0, 1, 2 for the first 
	// triangle, 0, 2, 3 for the second triangle etc
	// So the triangle count for a polygon will always be the polygon's vertex count - 2
	// Add these all together and we get the total polygon count for the new triangulated mesh.

	if (polyType == kPolyTypeMixed)
	{
		exportCount = 0;

		for (int i = 0; i < polyCount; i++)
		{
			int polyVertexCount = gex->polyCounts[i];
			exportCount += polyVertexCount - 2;
		}

		*exportPolyCount = exportCount;
		exportCount *= 3;
	}

	ExportVertex *exportVertex = new ExportVertex[exportCount];
	ExportVertex *ev = exportVertex;

	for (int i = 0; i < polyCount; i++)
	{
		int polyVertexCount = gex->polyCounts[i];
		
		int polygonStart = 0;
		int polygonEnd = 1;

		int firstVertexIndex = 0;
		std::string materialTag = gex->polyMaterial[i];

		if (polyType == kPolyTypeMixed)
		{
			// Mixed polygons are ngons or quads depending on if we allow triangulating quads.

			polygonStart = 2;
			polygonEnd = polyVertexCount;

			// Mixed polygons are triangulated, so change the polyVertexCount to 3 - kPolyTriangle

			polygonVertexCount = kPolyTypeTriangle;
		}

		for (int a = polygonStart; a < polygonEnd; a++)
		{

			// Associate the material with the polygon

			gex->materialIndexArray.push_back(materialTag);

			// We are still using polygonVertexCount as the polygon can be three or 4 vertices.

			for (unsigned int j = 0; j < polygonVertexCount; j++)
			{
				// just add one to the index for each vertex in a non modified polygon

				int index = start + j;

				if (polyType == kPolyTypeMixed)
				{
					// We are triangulating so always use the start value for the first vertex

					if (j == 0)
					{
						index = start;
					}
					else if (j == 1)
					{
						index = start + a - 1;
					}
					else if (j == 2)
					{
						index = start + a;
					}
				}

				unsigned int vertexIndex = gex->polyIndices[index];
				LXtPointID vertexID = gex->polyPoints[index];

				ev[0].index = vertexIndex;

				ev[0].position[0] = gex->points[vertexIndex * 3 + 0];
				ev[0].position[1] = gex->points[vertexIndex * 3 + 1];
				ev[0].position[2] = gex->points[vertexIndex * 3 + 2];

				if (gex->saveVertexNormals)
				{
					if (!gex->normalMapNames.empty())
					{
						// If there's a normal map then use it.

						std::string normalMapName = *(gex->normalMapNames.begin());
						bool mapWasSet = gex->SetMap(LXi_VMAP_NORMAL, normalMapName.c_str());
					}

					double nvec[3];
					if (gex->PolyNormal(nvec, vertexID))
					{
						ev[0].normal[0] = (float)nvec[0];
						ev[0].normal[1] = (float)nvec[1];
						ev[0].normal[2] = (float)nvec[2];
					}
				}

				if (gex->saveUVTextureCoordinates)
				{
					int k = 0;
					for (StringArray::const_iterator iter = gex->uvMapNames.begin(); iter != gex->uvMapNames.end(); ++iter)
					{
						if (k < kMaxTexcoordCount)
						{
							std::string uvMapName = *iter;

							if (!gex->FindTexture(uvMapName.c_str()))
							{
								gex->textureArray->push_back(TextureReference(uvMapName.c_str(), k));
							}

							if (gex->SetMap(LXi_VMAP_TEXTUREUV, uvMapName.c_str()))
							{
								float nvec[2];

								// For some reason in some imported meshes (using FBX) PolyMapValue is not working
								// need to look some more into this. I found triangulating the mesh in MODO seems to
								// resolve the issue.

								if (gex->PolyMapValue(nvec, vertexID))
								{
									ev[0].texcoord[k][0] = nvec[0];
									ev[0].texcoord[k][1] = nvec[1];
								}
							}
						}

						k++;
					}
				}

				if (gex->saveColors)
				{
					int colorCount = 0;

					for (StringArray::const_iterator iter = gex->colorRGBMapNames.begin(); iter != gex->colorRGBMapNames.end(); ++iter)
					{
						std::string colorRGBMapName = *iter;

						if (gex->SetMap(LXi_VMAP_RGB, colorRGBMapName.c_str()))
						{
							float nvec[3];
							if (gex->PolyMapValue(nvec, vertexID))
							{
								if (colorCount < kMaxVertexColorCount)
								{
									ev[0].color[colorCount][0] = nvec[0];
									ev[0].color[colorCount][1] = nvec[1];
									ev[0].color[colorCount][2] = nvec[2];
								}

								colorCount++;
							}
						}
					}

					for (StringArray::const_iterator iter = gex->colorRGBAMapNames.begin(); iter != gex->colorRGBAMapNames.end(); ++iter)
					{
						std::string colorRGBAMapName = *iter;

						if (gex->SetMap(LXi_VMAP_RGBA, colorRGBAMapName.c_str()))
						{
							float nvec[3];
							if (gex->PolyMapValue(nvec, vertexID))
							{
								if (colorCount < kMaxVertexColorCount)
								{
									ev[0].color[colorCount][0] = nvec[0];
									ev[0].color[colorCount][1] = nvec[1];
									ev[0].color[colorCount][2] = nvec[2];
								}

								colorCount++;
							}
						}
					}
				}

				ev++;
			}
		}

		// Although we have maybe split the polygon numerous times, we only move the start on
		// by the number of vertices in the polygon.

		start += gex->polyCounts[i];
	}

	ev = exportVertex;

	for (int i = 0; i < exportCount; i++)
	{
		ev[i].Hash();
	}

	return (exportVertex);
}

int OpenGexExport::FindExportVertex(const std::vector<int>& bucket, const ExportVertex *exportVertex, const ExportVertex& vertex)
{
	int size = (int)bucket.size();

	for (int i = 0; i < size; i++)
	{
		int index = bucket[i];

		if (exportVertex[index] == vertex)
		{
			return (index);
		}
	}

	return (-1);
}

int OpenGexExport::UnifyVertices(int vertexCount, const ExportVertex *exportVertex, ExportVertex *unifiedVertex, int *indexTable)
{
	// This function looks for identical vertices having exactly the same position, normal,
	// color, and texcoords. Duplicate vertices are unified, and a new index table is returned.

	int bucketCount = vertexCount >> 5;

	if (bucketCount > 1)
	{
		// Round down to nearest power of two by clearing the lowest
		// one bit until there's only a single one bit left. There are
		// faster ways of doing this, but they aren't as portable.

		for (;;)
		{
			int count = bucketCount & (bucketCount - 1);

			if (count == 0)
			{
				break;
			}

			bucketCount = count;
		}
	}
	else
	{
		bucketCount = 1;
	}

	std::vector<int> *hashTable = new std::vector<int>[bucketCount];
	int unifiedCount = 0;

	for (int i = 0; i < vertexCount; i++)
	{
		int exportVertexIndex = exportVertex[i].index;

		unsigned int bucket = exportVertex[i].hash & (bucketCount - 1);
		int index = FindExportVertex(hashTable[bucket], exportVertex, exportVertex[i]);

		if (index < 0)
		{
			indexTable[i] = unifiedCount;
			const ExportVertex ev = exportVertex[i];
			unifiedVertex[unifiedCount] = exportVertex[i];
			unifiedCount++;

			hashTable[bucket].push_back(i);
		}
		else
		{
			indexTable[i] = indexTable[index];
		}
	}

	delete[] hashTable;
	return (unifiedCount);
}

std::string OpenGexExport::EscapeURIReservedCharacters(const std::string& path)
{
	// Taken from the MODO COLLADA exporter

	std::string filteredStr;
	const char *text = path.c_str();

	const unsigned char *c;

	for (c = reinterpret_cast<const unsigned char *>(text); *c; ++c)
	{
		char tmp[8];

		// Per URI_BNF, allow path char set: alpha | digit | safe | extra [ / path ]
		
		bool legalChar =
			(*c >= 'a' && *c <= 'z') ||
			(*c >= 'A' && *c <= 'Z') ||
			// digit
			(*c >= '0' && *c <= '9') ||
			// safe
			(*c == '$' || *c == '-' || *c == '_' || *c == '@' || *c == '.' || *c == '&') ||
			// extra
			(*c == '!' || *c == '*' || *c == '"' || *c == '\'' || *c == '(' || *c == ')' || *c == ',');

		// path or volume separator

		legalChar = legalChar || (*c == '/' || *c == ':');

		if (legalChar)
		{
			filteredStr += *c;
		}
		else
		{
			filteredStr += "%";

#pragma warning(disable: 4996)
			sprintf(tmp, "%02X", *c);
			filteredStr.append(tmp);
		}
	}

	return (filteredStr);
}

std::string OpenGexExport::FilePathToAbsoluteURI(const std::string& fileName)
{
	// Taken from the MODO COLLADA exporter

	std::string filteredFileName(fileName);

	// Replace backslashes with forward slashes.
	
	replace(filteredFileName.begin(), filteredFileName.end(), '\\', '/');

	filteredFileName = EscapeURIReservedCharacters(filteredFileName);

	std::string prefix("/");

	if (filteredFileName[1] == ':')
	{
		// Strip off the drive volume and ":"

		filteredFileName = filteredFileName.substr(2, filteredFileName.length() - 2);
	}

	return (prefix + filteredFileName);
}

std::string OpenGexExport::FilePathToRelativeURI(const std::string& fileName)
{
	// Taken from the MODO COLLADA exporter

	std::string filteredFileName(fileName);

	// Replace backslashes with forward slashes.
	
	replace(filteredFileName.begin(), filteredFileName.end(), '\\', '/');

	// Find the position of the last forward slash.
	
	char separator = '/';
	size_t slashPos = filteredFileName.rfind(separator, filteredFileName.length());
		
	// If there was at least one forward slash, the path
	// is relative to an intermediate directory, otherwise
	// it's in the local scene directory.
	
	std::string relativePrefix;

	if (slashPos != std::string::npos)
	{
		relativePrefix = std::string("../");
	}
	else
	{
		relativePrefix = std::string("./");
	}

	filteredFileName = relativePrefix + filteredFileName;
	filteredFileName = EscapeURIReservedCharacters(filteredFileName);

	return (filteredFileName);
}

std::string OpenGexExport::NativePathToURI(const std::string& nativePath)
{
	// Taken from the MODO COLLADA exporter

	// We always get an absolute image file path from
	// MODO, so we need to check our AbsPath pref and
	// convert to relative as needed.
	
	std::string fileNamePath(nativePath);
	bool haveRelativePath(false);

	if (!prefs.GetUseAbsolutePath())
	{
		haveRelativePath = MakeFileRelative(fileNamePath);
	}

	std::string textureURI;

	if (!haveRelativePath)
	{
		textureURI = FilePathToAbsoluteURI(fileNamePath);
	}
	else
	{
		textureURI = FilePathToRelativeURI(fileNamePath);
	}

	// Remove the file extension if it is specified in the preferences

	if (prefs.GetTextureRemoveFileExtension())
	{
		int lastindex = (int)textureURI.find_last_of(ATTRVALUE_DOTSEPARATORSYMBOL);
		textureURI = textureURI.substr(0, lastindex);
	}

	return (textureURI);
}

std::string OpenGexExport::GetTextHintEncodedChannelValue(const CLxSceneSaver &saver, const std::string &channelName)
{
	std::string channelValueText;
	const unsigned MAX_HINT_TEXT_LENGTH = 2048;
	char hintText[MAX_HINT_TEXT_LENGTH];
	saver.ChanIntEncoded(channelName.c_str(), hintText, MAX_HINT_TEXT_LENGTH);
	channelValueText = std::string(hintText);

	return (channelValueText);
}

LXtTagInfoDesc OpenGexExport::descInfo[] =
{
	{ LXsSAV_OUTCLASS, LXa_SCENE },
	{ LXsSAV_DOSTYPE, "ogex" },
	{ LXsSRV_USERNAME, "OpenGex" },
	{ LXsSRV_LOGSUBSYSTEM, "io-status" },
	{ 0 }
};

void initialize()
{
	LXx_ADD_SERVER(Saver, OpenGexExport, "OpenGEX");
}
