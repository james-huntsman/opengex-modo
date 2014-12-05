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


#ifndef OpenGex_Modo_h
#define OpenGex_Modo_h

#include <lx_mesh.hpp>	
#include <lxu_scene.hpp>
#include <lx_deform.hpp>
#include <lxu_log.hpp>
#include <lxlog.h>
#include <lx_action.hpp>
#include <lxidef.h>
#include <list>
#include <map>
#include <set>
#include <algorithm>

#include <windows.h>
#include <vector>
#include <sstream>

typedef double Quaternion[4];

typedef enum en_RotationOrder
{
	ROTATION_ORDER_XYZ,
	ROTATION_ORDER_XZY,
	ROTATION_ORDER_YXZ,
	ROTATION_ORDER_YZX,
	ROTATION_ORDER_ZXY,
	ROTATION_ORDER_ZYX
} RotationOrder;

namespace OpenGex
{
	enum
	{
		kVertexColorMapTypeCount = 2,
		kMaxVertexColorCount = 2,
		kMaxTexcoordCount = 2
	};

	struct ExportVertex
	{
		unsigned int	hash;
		unsigned int	index;

		LXtFVector		position;
		LXtFVector		normal;
		float			color[kMaxVertexColorCount][3];
		float			texcoord[kMaxTexcoordCount][2];

		ExportVertex();

		bool operator ==(const ExportVertex& v) const;

		void Hash(void);
	};

	class Normal
	{
		public:

			double		vec[3];

			bool operator< (const Normal &x) const
			{
				double	 d;

				if (d = vec[0] - x.vec[0]) return (d > 0.0f);
				if (d = vec[1] - x.vec[1]) return (d > 0.0f);
				if (d = vec[2] - x.vec[2]) return (d > 0.0f);

				return (false);
			}
	};

	typedef std::map<Normal, unsigned> NormalMap;

	class Texcoord
	{
		public:

			double		vec[2];

			bool operator < (const Texcoord &x) const
			{
				double	 d;

				if (d = vec[0] - x.vec[0]) return (d > 0.0);
				if (d = vec[1] - x.vec[1]) return (d > 0.0);

				return (false);
			}
	};

	typedef std::map<Texcoord, unsigned> TexcoordMap;

	class ColorRGB
	{
		public:

			double		vec[3];

			bool operator < (const ColorRGB &x) const
			{
				double	 d;

				if (d = vec[0] - x.vec[0]) return (d > 0.0f);
				if (d = vec[1] - x.vec[1]) return (d > 0.0f);
				if (d = vec[2] - x.vec[2]) return (d > 0.0f);
				
				return (false);
			}
	};

	class ColorRGBA
	{
		public:

			double		vec[4];

			bool operator < (const ColorRGBA &x) const
			{
				double	 d;

				if (d = vec[0] - x.vec[0]) return (d > 0.0f);
				if (d = vec[1] - x.vec[1]) return (d > 0.0f);
				if (d = vec[2] - x.vec[2]) return (d > 0.0f);
				if (d = vec[3] - x.vec[3]) return (d > 0.0f);
				
				return (false);
			}
	};

	typedef struct en_Matrix4Wrap
	{
		LXtMatrix4 m;
	} Matrix4Wrap;
	
	const double METERS_PER_MICRON = 0.000001;
	const double METERS_PER_MILLIMETER = 0.001;
	const double METERS_PER_CENTIMETER = 0.01;
	const double METERS_PER_KILOMETER = 1000.0;
	const double METERS_PER_MEGAMETER = 1000000.0;

	const double METERS_PER_MIL = 0.0000254;
	const double METERS_PER_INCH = 0.0254;
	const double METERS_PER_FOOT = 0.3048;
	const double METERS_PER_MILE = 1609.344;

	typedef std::vector<float>								FloatArray;
	typedef std::set<float>									FloatSet;
	typedef std::map<LXtPointID, unsigned>					PointMap;
	typedef std::map<ColorRGB, unsigned>					ColorRGBMap;
	typedef std::map<ColorRGBA, unsigned>					ColorRGBAMap;
	typedef std::map<std::string, unsigned>					NodeIDIndexMap;
	typedef std::map<unsigned, float>						IndexFloatMap;
	typedef std::vector<std::string>						StringArray;
	typedef std::vector<bool>								BoolArray;
	typedef std::vector<Matrix4Wrap>						Matrix4Array;

	enum
	{
		kPolyTypeNone,
		kPolyTypeTriangle = 3,
		kPolyTypeQuad,
		kPolyTypeMixed
	};

	enum
	{
		kNodeTypeNone = -1,
		kNodeTypeNode,
		kNodeTypeBone,
		kNodeTypeGeometry,
		kNodeTypeLight,
		kNodeTypeCamera,
		kNodeTypeCount
	};

	typedef enum en_Interpolation {
		INTERPOLATION_LINEAR,
		INTERPOLATION_BEZIER,
		INTERPOLATION_BSPLINE,
		INTERPOLATION_HERMITE,
		INTERPOLATION_STEP
	} Interpolation;

	typedef std::vector<Interpolation> InterpArray;

	struct Envelope
	{
		unsigned		 interpolationType;

		FloatArray	 	 times;
		FloatArray	 	 outputs;
		FloatArray	 	 inTanControlX;
		FloatArray	 	 inTanControlY;
		FloatArray	 	 outTanControlX;
		FloatArray	 	 outTanControlY;
		InterpArray	 	 interpolations;
	};


	struct NodeReference
	{
		std::string					node; // Identity
		std::string					nodeName; // Node Name
		int							nodeType;
		std::string					structName;

		NodeReference(const char * n, const char * nn, int type, const char *name, const char *tag = "") : node(n), nodeName(nn), nodeType(type), structName(name) {}
		NodeReference(const NodeReference& nodeReference) : node(nodeReference.node), nodeName(nodeReference.nodeName), nodeType(nodeReference.nodeType), structName(nodeReference.structName) {}
	};


	struct ObjectReference
	{
		std::string					object;
		std::string					structName;
		std::vector<std::string>	nodeTable;

		ObjectReference(const char * obj, const char *name, const char * node) : object(obj), structName(name), nodeTable(1, node) {}
		ObjectReference(const ObjectReference& objectReference) : object(objectReference.object), structName(objectReference.structName), nodeTable(objectReference.nodeTable) {}
	};


	struct MaterialReference
	{
		std::string					material;
		std::string					structName;

		MaterialReference(const char * mat, const char *name) : material(mat), structName(name) {}
		MaterialReference(const MaterialReference& materialReference) : material(materialReference.material), structName(materialReference.structName) {}
	};


	struct TextureReference
	{
		std::string					texture;
		int							texcoord;

		TextureReference(const char * tex, int coord) : texture(tex), texcoord(coord) {}
		TextureReference(const TextureReference& textureReference) : texture(textureReference.texture), texcoord(textureReference.texcoord) {}
	};

	class OpenGexPreferences
	{
		private:

		public:
	
			OpenGexPreferences();
			virtual			~OpenGexPreferences();

			bool			GetUseAbsolutePath() const;
			bool			GetTextureRemoveFileExtension() const;

			int				GetUnitSystem() const;
			int				GetDefaultUnit() const;
			double			GetMetersPerGameUnit() const;

			int				GetUpAxis() const;

			bool			GetSaveHiddenItems() const;
			bool			GetSaveQuads() const;
			bool			GetTriangulateMesh() const;

			double			GetZNear() const;
			double			GetZFar() const;
	};

	class DeformerWeightToVertexVisitor : public CLxImpl_AbstractVisitor
	{
		private:

			CLxUser_SceneService						sceneSvc;
			CLxUser_Scene								scene;
			std::set<std::string>						*nodeIDs;
			CLxUser_ChannelRead							setupChannels;
			CLxUser_SceneGraph							sceneGraph;
			CLxUser_ItemGraph							itemGraph;
			bool										isLocatorPass;
			CLxUser_GroupDeformer						*groupDeformer;
			unsigned									meshIndex;
			CLxUser_Point								meshPoint;

			LxResult EvaluateLocatorPass();
			LxResult EvaluateJointWeightPass();

		public:

			NodeIDIndexMap								nodeIndexMap;
			Matrix4Wrap									skinBindMatrix;
		
			std::vector<unsigned>						boneCounts;
			std::vector<unsigned>						jointWeightIndices;
			
			Matrix4Array								bindMatrices;
			std::vector<float>							weights;

			typedef CLxUser_GroupDeformer::DeformerWeightArray DeformerWeightArray;
			DeformerWeightArray							deformerWeightList;

			DeformerWeightToVertexVisitor(CLxUser_Item& meshItem, CLxUser_GroupDeformer *groupDeformerID, unsigned groupDeformerMeshIndex, std::set<std::string> *jointNodeIDs);
			~DeformerWeightToVertexVisitor();

			void CalculateBindPoseMatrix(CLxUser_Item& locatorItem, LXtMatrix4& invBindMatrix);
			void FindLocators();
			LxResult Enumerate();
			virtual LxResult Evaluate() LXx_OVERRIDE;
	};

	class MeshMapVisitor : public CLxImpl_AbstractVisitor
	{
		private:
		
			CLxUser_MeshMap								*meshMap;
			StringArray									mapNames;

			virtual LxResult Evaluate()
			{
				const char *mapName;

				if (LXx_OK(meshMap->Name(&mapName)))
				{
					mapNames.push_back(std::string(mapName));
				}

				return (LXe_OK);
			}
	
		public:
		
			MeshMapVisitor(CLxUser_MeshMap *theMeshMap)
			{
				meshMap = theMeshMap;
			}

			void SortMapNames()
			{
				std::sort(mapNames.begin(), mapNames.end());
			}

			const StringArray& GetMapNames() const
			{
				return (mapNames);
			}
	};

	class OpenGexExport : public CLxSceneSaver, CLxLineFormat
	{
		private:

			typedef enum en_PolyPass
			{
				POLYPASS_BUILDMATERIALSET,
				POLYPASS_POLYGONS
			} PolyPass;

			HANDLE										exportFile;
			int											indentLevel;

			OpenGexPreferences							prefs;

			std::vector<OpenGex::NodeReference>			*nodeArray;
			std::vector<OpenGex::ObjectReference>		*geometryArray;
			std::vector<OpenGex::ObjectReference>		*lightArray;
			std::vector<OpenGex::ObjectReference>		*cameraArray;
			std::vector<OpenGex::MaterialReference>		*materialArray;
			std::vector<OpenGex::TextureReference>		*textureArray;

			StringArray									normalMapNames;
			StringArray									uvMapNames;
			StringArray									colorRGBMapNames;
			StringArray									colorRGBAMapNames;
			StringArray									morphMapNames;

			std::set<std::string>						jointNodeIDs;

			unsigned									pointIndex;
			PointMap									pointMap;
			FloatArray									points;

			typedef TexcoordMap							*TexcoordMapID;
			typedef std::vector<TexcoordMapID>			TexcoordMapIDArray;
			TexcoordMapIDArray							texcoordMaps;

			typedef ColorRGBMap							*ColorRGBMapID;
			typedef ColorRGBAMap						*ColorRGBAMapID;
			typedef std::vector<ColorRGBMapID>			ColorRGBMapIDArray;
			typedef std::vector<ColorRGBAMapID>			ColorRGBAMapIDArray;
			ColorRGBMapIDArray							colorRGBMaps;
			ColorRGBAMapIDArray							colorRGBAMaps;

			StringArray									materials;

			std::vector<unsigned>						polyCounts;
			std::vector<unsigned>						polyIndices;
			std::vector<LXtPointID>						polyPoints;
			StringArray									polyMaterial;
			StringArray									materialIndexArray;

			PolyPass									polyPass;

			void Write(const void *buffer, unsigned int size);
			void Write(const char *string);
			void Write(const wchar_t *string);
			void IndentWrite(const char *string, int extra = 0, bool newline = false);

			void WriteInt(int i);
			void WriteUnsignedInt(unsigned int i);
			void WriteFloat(double d);
			void WriteFloat(float f);
			void WriteHexFloat(double d);
			void WriteHexFloat(float f);

			void WriteMatrix(const LXtMatrix4& matrix);
			void WriteMatrixFlat(const LXtMatrix4& matrix);
			void WriteHexMatrixFlat(const LXtMatrix4& matrix);
			void WritePoint3(const LXtFVector& point);
			void WritePoint3(const LXtVector& point);
			void WriteHexPoint3(const LXtFVector& point);
			void WriteHexPoint3(const LXtVector& point);
			void WriteQuaternion(const Quaternion &quat);
			void WriteHexQuaternion(const Quaternion &quat);
			void WriteColor(const LXtFVector& color);
			void WriteFileName(const char * string);

			void WriteIntArray(int count, const int *value);
			void WriteDoubleArray(int count, const float *value);

			void WriteVertex(const LXtFVector2& vertex);
			void WriteVertex(const LXtFVector& vertex);
			template <class type> void WriteVertexArray(int count, const type *vertex, int stride);

			void WriteTriangle(int triangleIndex, const int *indexTable);
			void WriteTriangleArray(int count, const int *indexTable);

			void WriteQuad(int quadIndex, const int *indexTable);
			void WriteQuadArray(int count, const int *indexTable);

			void WriteNodeTable(const OpenGex::ObjectReference *objectRef);

			bool isRegistered;
			bool saveVertexNormals;
			bool saveUVTextureCoordinates;
			bool saveColors;

			bool haveAtLeastOneItem;
			bool haveAtLeastOneTransformAnimation;
			bool haveAtLeastOneMesh;
			bool haveAtLeastOneMeshAnimation;

			CLxUser_ItemGraph itemGraph;

			void GetItemTransform(CLxLoc_Item& item, LXtMatrix4& matrix);

			std::string UVMapName(CLxUser_Item&	imageMap);

			LxResult AddGeometryItems(void);
			NodeReference *FindNode(const char * node) const;
			OpenGex::MaterialReference *FindMaterial(const char * ident);
			static OpenGex::ObjectReference *FindObject(std::vector<OpenGex::ObjectReference> *array, const char * object);

			void ProcessObjects(void);

			void ExportNode(const char * ident);
			void ExportObjects(void);
			void ExportMaterials(void);
			bool ExportTexture(CLxUser_Item& material, CLxUser_Item& locator, const char * materialName, const char *attrib);
			void ExportMaterialRef(const char * ident, int index);
			OpenGex::TextureReference *FindTexture(const char * texture);

			void ExportSkin(CLxUser_GroupDeformer& groupDeformer, unsigned groupDeformerMeshIndex, CLxUser_Item& item, int vertexCount, const OpenGex::ExportVertex *exportVertex);
			void ExportGeometry(const OpenGex::ObjectReference *objectRef);
			void ExportLight(const OpenGex::ObjectReference *objectRef);
			void ExportCamera(const OpenGex::ObjectReference *objectRef);

			void ExportTransform(const char * ident);

			void ExportKeyTimes(const Envelope& animationEnvelope);
			void ExportKeyTimeControlPoints(const Envelope& animationEnvelope);
			void ExportKeyValues(const Envelope& animationEnvelope);
			void ExportKeyValueControlPoints(const Envelope& animationEnvelope);
			void ExportAnimationTrack(const Envelope& animationEnvelope, const char *target, bool newline);

			static OpenGex::ExportVertex *DeindexMesh(int *exportPolyCount, unsigned int polyType, void *data);
			static int FindExportVertex(const std::vector<int>& bucket, const OpenGex::ExportVertex *exportVertex, const OpenGex::ExportVertex& vertex);
			static int UnifyVertices(int vertexCount, const OpenGex::ExportVertex *exportVertex, OpenGex::ExportVertex *unifiedVertex, int *indexTable);
			
			bool FindGroupDeformer(CLxUser_Item &meshItem, CLxUser_GroupDeformer &groupDeformer, unsigned &groupDeformerMeshIndex);

			void ClearData(void);
			void VisitPolygons(PolyPass inPolyPass);
			void AddPolygon(void);

			bool AllTriangles() const;
			bool AllQuads() const;

			static std::string EscapeURIReservedCharacters(const std::string& path);

			std::string FilePathToAbsoluteURI(const std::string& fileName);
			std::string	FilePathToRelativeURI(const std::string& fileName);
			std::string NativePathToURI(const std::string &nativePath);

			std::string GetTextHintEncodedChannelValue(const CLxSceneSaver &saver, const std::string &channelName);
			LxResult GetChannelKeyframeTimes(const std::string &channelName, Envelope &animationEnvelope, FloatSet &times);
			bool GetKeyFrames(CLxUser_Envelope &envelope, Envelope &animationEnvelope);

			double SetUnitScale(int unitSystem, int unit);

		public:

			OpenGexExport();
			~OpenGexExport();

			virtual CLxFileFormat *	 ss_Format()	{ return this; }

			virtual void		ss_Verify();
			virtual LxResult	ss_Save();
			virtual void		ss_Point();
			virtual void		ss_Polygon();

			static LXtTagInfoDesc	 descInfo[];
	};
}

#endif
