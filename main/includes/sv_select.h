


























#ifndef SV_SELECT_H_
#define SV_SELECT_H_

#include "includes.h"

enum SV_SELECT_TYPE   //TODO TODO: consider: STRONG=indirect; MEDIUM=semi-direct; WEAK=direct feature detection and tracking
{
	ACQ_TYPE_STRONG,		//!< Strong acquisition
	ACQ_TYPE_MEDIUM,		//!< Medium acquisition
	ACQ_TYPE_WEAK			//!< Weak acquisition
};

/*! @ingroup CLASSES
	@brief SV_Select uses state information from the PVT/EKF and the almanac to
 * predict the state of the GPS constellation. This information is used to aid
 * the acquisition process. */
class SV_Select : public Threaded_Object
{

	private:


		Acq_Command_S		command;						//!< Interface to acquisition thread

		SV_Prediction_M 	sv_prediction[MAX_SV];			//!< Predicated delay/doppler visibility, etc
		int					type;							//!< WEAK or STRONG
		int					strong_sv;						//!< The current strong SV


	public:

		SV_Select();
		~SV_Select();
		void Start();					//!< Start the thread
		void Import();					//!< Get info from PVT
		void Export(int _sv);			//!< Export state info for the given SV
 		void UpdateState();				//!< Update acq type
 		void Acquire();					//!< Run the acquisition
		void SV_Predict(int _sv);		//!< Predict states of SVs
 		size_t SetupRequest(int _sv);	//!< Setup the acq request //xin//uint32 SetupRequest(int32 _sv);	//!< Setup the acq request


};

#endif /* SV_SELECT_H_ */
