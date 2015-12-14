
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
						k++;
					}
					SafeRelease(faceFrame);
				}
			}
			catch (std::exception &e) {
				OFXKINECTFORWINDOWS2_ERROR << e.what();
			}

		}

	} // end namespace Source
} // end namespace ofxKinectForWindow2