/**
 * @file WheelEncoder.h
 * @date Feb 24, 2011
 * @author tallred3
 * @brief Wheel Encoder Class Header
 */

#ifndef WHEELENCODER_H_
#define WHEELENCODER_H_

#include <mrpt/hwdrivers/CGenericSensor.h>
#include "YClopsSensor.h"

namespace mrpt {
	namespace hwdrivers {

		/**
		 * @brief Implements an interface with wheel encoders through the motor controller, will
		 * allow us to additionally generate higher level data such as turning speed and linear speed
		 */
		class WheelEncoder: public CGenericSensor {
			DEFINE_GENERIC_SENSOR(WheelEncoder);
		public:
			WheelEncoder();

			/**
			 * @brief load sensor specific values from the configuration file
			 */
			void loadConfig_sensorSpecific( const mrpt::utils::CConfigFileBase & config, const std::string & sectionName);

			/**
			 * @brief initialize the sensor
			 */
			void initialize();

			/**
			 * @brief collect data from the sensor and cache it
			 */
			void doProcess();

			/**
			 * @return SensorData* needs to be allocated on the heap to avoid splicing when we cast it to be a various sensor type
			 */
			SensorData * getData();

			/**
			 * @brief dumps data out to the parameter out
			 * @param[out] the stream to which to write the data
			 */
			void dumpData( std::ostream & out ) const;

			virtual ~WheelEncoder();
		protected:

			struct encoder {
				int count;				//!<The count of the encoder since the last reading
				int absoluteCount;		//!<The total count of the encoder
				int speed;				//!<The current speed of the encoder
				double pprPiDiameter;	//!<The Pi times the diameter divided by the pulses per revolution
				explicit encoder( int ppr, double diameter ): count(0), absoluteCount(0), speed(0), pprPiDiameter(M_PI*diameter/ppr) {};
				double getTotalDistance() const { return this->pprPiDiameter * this->absoluteCount; };
				double getDistance() const { return this->pprPiDiameter * this->count; };
				double getSpeed() const { return this->pprPiDiameter * this->speed; };
			};

			encoder left;
			encoder right;
			int encoderPPR;
		};
	}
}
#endif /* WHEELENCODER_H_ */
