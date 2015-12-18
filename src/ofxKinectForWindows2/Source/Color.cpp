#include "Color.h"
#include "ofMain.h"


#define CHECK_OPEN if(!this->reader) { OFXKINECTFORWINDOWS2_ERROR << "Failed : Reader is not open"; }

namespace ofxKinectForWindows2 {
	namespace Source {
		//----------
		Color::Color() {
			this->exposure = 0;
			this->frameInterval = 0;
			this->gain = 0;
			this->gamma = 0;
			this->isFrameNewFlag = false;
		}

		//----------
		string Color::getTypeName() const {
			return "Color";
		}

		//----------
		void Color::init(IKinectSensor * sensor) {
			this->reader = NULL;
			try {
				IColorFrameSource * source = NULL;

				if (FAILED(sensor->get_ColorFrameSource(& source))) {
					throw(Exception("Failed to initialise Color source"));
				}

				if (FAILED(source->OpenReader(& this->reader))) {
					throw(Exception("Failed to initialise Color reader"));
				}

				SafeRelease(source);

				if (FAILED(sensor->get_CoordinateMapper(&this->coordinateMapper))) {
					throw(Exception("Failed to acquire coordinate mapper"));
				}


			} catch (std::exception & e) {
				SafeRelease(this->reader);
				throw (e);
			}
		}

		//----------
		void Color::update() {
			CHECK_OPEN

			this->isFrameNewFlag = false;
			IColorFrame * frame = NULL;
			IFrameDescription * frameDescription = NULL;
			try {
				//acquire frame
				if (FAILED(this->reader->AcquireLatestFrame(&frame))) {
					return; // we often throw here when no new frame is available
				}
				this->isFrameNewFlag = true;

				//allocate pixels and texture if we need to
				if (FAILED(frame->get_FrameDescription(&frameDescription))) {
					throw Exception("Failed to get frame description");
				}

				int width, height;
				if (FAILED(frameDescription->get_Width(&width)) || FAILED(frameDescription->get_Height(&height))) {
					throw Exception("Failed to get width and height of frame");
				}
				if (width != this->pixels.getWidth() || height != this->texture.getHeight()) {
					this->pixels.allocate(width, height, OF_IMAGE_COLOR_ALPHA);
					this->texture.allocate(this->pixels);
				}

				//update local assets
				if (FAILED(frame->CopyConvertedFrameDataToArray(this->pixels.size(), this->pixels.getPixels(), ColorImageFormat_Rgba))) {
					throw Exception("Couldn't pull pixel buffer");
				}
				registeredCacheDirty = true;

				if (this->useTexture) {
					this->texture.loadData(this->pixels);
				}

				//update field of view
				if (FAILED(frameDescription->get_HorizontalFieldOfView(&this->horizontalFieldOfView))) {
					throw Exception("Failed to get horizonal field of view");
				}
				if (FAILED(frameDescription->get_VerticalFieldOfView(&this->verticalFieldOfView))) {
					throw Exception("Failed to get vertical field of view");
				}
				if (FAILED(frameDescription->get_DiagonalFieldOfView(&this->diagonalFieldOfView))) {
					throw Exception("Failed to get diagonal field of view");
				}

				IColorCameraSettings * cameraSettings;
				if (FAILED(frame->get_ColorCameraSettings(&cameraSettings))) {
					throw Exception("Failed to get color camera settings");
				}
				cameraSettings->get_ExposureTime(&this->exposure);
				cameraSettings->get_FrameInterval(&this->frameInterval);
				cameraSettings->get_Gain(&this->gain);
				cameraSettings->get_Gamma(&this->gamma);
			} catch (std::exception & e) {
				OFXKINECTFORWINDOWS2_ERROR << e.what();
			}
			SafeRelease(frameDescription);
			SafeRelease(frame);

		}

		//----------
		bool Color::isFrameNew() const {
			return this->isFrameNewFlag;
		}
		
		//----------
		long int Color::getExposure() const {
			return this->exposure;
		}

		//----------
		long int Color::getFrameInterval() const {
			return this->frameInterval;
		}

		//----------
		float Color::getGain() const {
			return this->gain;
		}

		//----------
		float Color::getGamma() const {
			return this->gamma;
		}

		ofPixels& Color::getRegisteredPixels(shared_ptr<Depth> depth) {
			if (registeredPixels.getWidth() != 512) {
				registeredPixels.allocate(512, 424, ofPixelFormat::OF_PIXELS_RGB);
			}

			if (registeredCacheDirty) {

				ColorSpacePoint *depth2rgb = new ColorSpacePoint[512 * 424];

				auto len = registeredPixels.getWidth() * registeredPixels.getHeight();
				coordinateMapper->MapDepthFrameToColorSpace(len, depth->getPixels(), len, depth2rgb);

				auto *pix = pixels.getData();
				auto *rpix = registeredPixels.getData();
				int rpixi, pixi;
				ColorSpacePoint csp;

				for (int i = 0; i < len; i++) {
					csp = depth2rgb[i];
					rpixi = i * 3;
					if (csp.X < 0 || csp.Y < 0 || csp.X >= pixels.getWidth() || csp.Y >= pixels.getHeight()) {
						for (int j = 0; j < 3; j++) {
							rpix[rpixi + j] = 0;
						}
					}
					else {
						pixi = (int(csp.X) + pixels.getWidth() * int(csp.Y)) * 4;
						for (int j = 0; j < 3; j++) {
							rpix[rpixi + j] = pix[pixi + j];
 						}
					}

				}
				delete[] depth2rgb;
				registeredCacheDirty = false;

			}
			return registeredPixels;
		}
	}
}
