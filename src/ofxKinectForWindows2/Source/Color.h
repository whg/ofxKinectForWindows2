#pragma once

#include "BaseImage.h"
#include "../Utils.h"

#include "ofBaseTypes.h"
#include "ofPixels.h"
#include "ofTexture.h"

#include "Depth.h"


namespace ofxKinectForWindows2 {
	namespace Source {
		class Color : public BaseImage<unsigned char, IColorFrameReader, IColorFrame> {
		public:
			Color();
			string getTypeName() const override;
			void init(IKinectSensor *) override;

			void update() override;
			bool isFrameNew() const override;
			long int getExposure() const;
			long int getFrameInterval() const;
			float getGain() const;
			float getGamma() const;

			ofPixels& getRegisteredPixels(shared_ptr<Depth> depth);

		protected:
			TIMESPAN exposure;
			TIMESPAN frameInterval;
			float gain;
			float gamma;
			bool  isFrameNewFlag;

			ICoordinateMapper *coordinateMapper;
			ofPixels registeredPixels;
			bool registeredCacheDirty;
		};
	}
}