
#include "Face.h"
#define CHECK_OPEN if(!this->reader) { OFXKINECTFORWINDOWS2_ERROR << "Failed : Reader is not open"; }


namespace ofxKinectForWindows2 {
	namespace Source {

		Face::Face() {
			
		}

		string Face::getTypeName() const {
			return "Face";
		}

		void Face::init(IKinectSensor *sensor) {
			Body::init(sensor);


			deformations = std::vector<std::vector<float>>(BODY_COUNT, std::vector<float>(FaceShapeDeformations::FaceShapeDeformations_Count));

			try {
				for (int count = 0; count < BODY_COUNT; count++) {
					// Source
					if (FAILED(CreateHighDefinitionFaceFrameSource(sensor, &faceSource[count]))) {
						throw(Exception("Failed to initalise HDFaceFrame source"));
					}

					auto fc = faceSource[count];
					if (FAILED(fc->OpenReader(&faceReader[count]))) {
						throw(Exception("Failed to open face reader"));
					}

					if (FAILED(fc->OpenModelBuilder(FaceModelBuilderAttributes::FaceModelBuilderAttributes_None, &faceModelBuilder[count]))) {
						throw(Exception("Failed to open model builder"));
					}

					if (FAILED(faceModelBuilder[count]->BeginFaceDataCollection())) {
						throw(Exception("Failed to begin face data collection"));
					}

					if (FAILED(CreateFaceAlignment(&faceAlignment[count]))) {
						throw(Exception("Failed to create face alignment"));
					}

					if (FAILED(CreateFaceModel(1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, &deformations[count][0], &faceModel[count]))) {
						throw(Exception("Failed to create face model"));
					}
				}

				GetFaceModelVertexCount(&numVertices); // should be 1347
				imagePoints.resize(numVertices);
				objectPoints.resize(numVertices);
			}
			catch (std::exception &e) {
				for (int count = 0; count < BODY_COUNT; count++) {
					SafeRelease(faceReader[count]);
					SafeRelease(faceModelBuilder[count]);
				}
				throw(e);
			}

		}

		void Face::update() {
			
			CHECK_OPEN

			IBodyFrame* bodyFrame = nullptr;

			try {
				if (FAILED(this->reader->AcquireLatestFrame(&bodyFrame))) {
					return;
				}

				IBody* ppBodies[BODY_COUNT] = { 0 };
				if (FAILED(bodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies))) {
					throw Exception("Failed to refresh body data");
				}

				for (int count = 0; count < BODY_COUNT; count++) {
					BOOLEAN trackingValid = false;
					faceSource[count]->get_IsTrackingIdValid(&trackingValid);
					if (!trackingValid) {
						BOOLEAN tracked = false;
						ppBodies[count]->get_IsTracked(&tracked);
						if (tracked) {
							UINT64 trackingId;
							ppBodies[count]->get_TrackingId(&trackingId);
							faceSource[count]->put_TrackingId(trackingId);
						}
					}
				}

				for (int count = 0; count < BODY_COUNT; count++) {
					SafeRelease(ppBodies[count]);
				}
				SafeRelease(bodyFrame);

				for (int count = 0; count < BODY_COUNT; count++) {
					IHighDefinitionFaceFrame* faceFrame = nullptr;
					if (FAILED(faceReader[count]->AcquireLatestFrame(&faceFrame))) {
						continue;
					}
					BOOLEAN tracked = false;
					faceFrame->get_IsFaceTracked(&tracked);
					if (!tracked) {
						continue;
					}
					if (FAILED(faceFrame->GetAndRefreshFaceAlignmentResult(faceAlignment[count]))) {
						throw Exception("Failed to refresh face alignment");
					}
								
					vector<CameraSpacePoint> faceVertices(numVertices);
					if (FAILED(faceModel[count]->CalculateVerticesForAlignment(faceAlignment[count], numVertices, &faceVertices[0]))) {
						throw Exception("Can't get vertices for face");
					}

					ColorSpacePoint onImageVertex;
					int k = 0;
					for (auto &vert : faceVertices) {
						this->coordinateMapper->MapCameraPointToColorSpace(vert, &onImageVertex);
						imagePoints[k].x = onImageVertex.X; 
						imagePoints[k].y = onImageVertex.Y;
						objectPoints[k].x = vert.X;
						objectPoints[k].y = vert.Y;
						objectPoints[k].z = vert.Z;
						k++;
					}

					CameraSpacePoint o;
					faceAlignment[count]->get_HeadPivotPoint(&o);
					origin = ofVec3f(o.X, o.Y, o.Z);

					Vector4 fr;
					faceAlignment[count]->get_FaceOrientation(&fr);
					orientation = ofQuaternion(fr.x, fr.y, fr.z, fr.w);


					SafeRelease(faceFrame);
				}
			}
			catch (std::exception &e) {
				OFXKINECTFORWINDOWS2_ERROR << e.what();
			}

		}

		ofMesh& Face::getObjectMesh() {
			
			ofVec3f eulerFaceRotation = orientation.getEuler();
			float yaw = eulerFaceRotation.x;
			float pitch = eulerFaceRotation.y;
			float roll = eulerFaceRotation.z;

			auto op1 = objectPoints[HighDetailFacePoints::HighDetailFacePoints_NoseTip];
			auto op2 = objectPoints[HighDetailFacePoints::HighDetailFacePoints_ForeheadCenter];

			op1 -= origin; op2 -= origin;
			op1.rotate(-yaw, -pitch, -roll);
			op2.rotate(-yaw, -pitch, -roll);
			op1 += origin; op2 += origin;

			CameraSpacePoint kp1{ op1.x, op1.y, op1.z };
			CameraSpacePoint kp2{ op2.x, op2.y, op2.z };

			ColorSpacePoint cp1, cp2;
			coordinateMapper->MapCameraPointToColorSpace(kp1, &cp1);
			coordinateMapper->MapCameraPointToColorSpace(kp2, &cp2);

			op1.z = 0; op2.z = 0;
			ofVec3f ocp1(cp1.X, cp1.Y);
			ofVec3f ocp2(cp2.X, cp2.Y);

			auto baseRatio = (ocp1 - ocp2).length() / (op1 - op2).length();

			ColorSpacePoint cameraOrigin;
			CameraSpacePoint csorigin{ origin.x, origin.y, origin.z };
			coordinateMapper->MapCameraPointToColorSpace(csorigin, &cameraOrigin);

			auto scaleAdjustment = 0.0;
			auto renderer = ofGetCurrentRenderer();
			auto transform = renderer->getCurrentMatrix(OF_MATRIX_MODELVIEW) * renderer->getCurrentMatrix(OF_MATRIX_PROJECTION);


			objectMesh.clear();
			auto calculatedScale = false;
			for (auto &vertex : objectPoints) {

				auto p = vertex - origin;
				p.rotate(0, 180, 180);
				int d = 1.0f;

				if (!calculatedScale) {
					ofVec2f reference = imagePoints[0];
					float mind = FLT_MAX;

					for (float q = 0.9; q < 1.01; q += 0.01) {
						ofVec3f temp(p);
						temp *= baseRatio * q;
						temp += ofVec3f(cameraOrigin.X, cameraOrigin.Y);
						ofVec4f v4(temp.x, temp.y, temp.z, 1.0);
						auto projected = temp * transform;

						projected.x = (projected.x + 1.0f) / 2.0f * ofGetWidth();
						projected.y = (1.0f - projected.y) / 2.0f * ofGetHeight();

						auto dist = reference.distance(ofVec2f(projected.x, projected.y));
						if (dist < mind) {
							mind = dist;
							scaleAdjustment = q;
						}
					}
					calculatedScale = true;
				}
				//scale_adjustment = ofMap(mouseX, 0, ofGetWidth(), 0.9, 1.1);
				p *= baseRatio * scaleAdjustment;
				p += ofVec3f(cameraOrigin.X, cameraOrigin.Y);
				objectMesh.addVertex(p);

			}
			return objectMesh;
		}

	} // end namespace Source
} // end namespace ofxKinectForWindow2