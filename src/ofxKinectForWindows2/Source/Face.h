#pragma once

#include "ofMain.h"

#include "Body.h"
#include <Kinect.Face.h>


namespace ofxKinectForWindows2 {
	namespace Source {

		class Face : public Body {
		public:
			Face();
			string getTypeName() const override;
			void init(IKinectSensor *) override;

			void update();

			ofMesh& getObjectMesh();

			ofMesh imageMesh;
			vector<ofVec2f> imagePoints;
			vector<ofVec3f> objectPoints;
			ofVec3f origin;
			ofQuaternion orientation;
			ofMesh objectMesh;

		protected:
			IHighDefinitionFaceFrameSource* faceSource[BODY_COUNT];
			IHighDefinitionFaceFrameReader* faceReader[BODY_COUNT];
			IFaceModelBuilder* faceModelBuilder[BODY_COUNT];
			bool produce[BODY_COUNT] = { false };
			IFaceAlignment* faceAlignment[BODY_COUNT];
			IFaceModel* faceModel[BODY_COUNT];
			std::vector<std::vector<float>> deformations;
			uint32_t numVertices;

		};
	}

}