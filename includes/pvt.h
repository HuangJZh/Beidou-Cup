
























/*----------------------------------------------------------------------------------------------*/

#ifndef _PVT_H
#define _PVT_H

#include "includes.h"
//#include "ephemeris.h"
//#include "channel.h"


















/*! @ingroup CLASSES
	@brief Performs least squares point solution.
*/
class PVT : public Threaded_Object
{

	private:














		/* Position and clock solutions */
		SPS_M				master_nav;							//!< Master nav sltn




		Preamble_2_PVT_S	preamble;							//!< Preamble from tracking isr
		PVT_2_TLM_S			tlm_s;								//!< Dump stuff to telemetry, sv_select, pps, and ekf












	public:

		PVT();
		~PVT();
		void Start();							//!< Start the thread
		void Import();							//!< Get data into the thread
		void Export();							//!< Get data out of the thread


		void Navigate();						//!< main navigation task, call the following tabbed functions





















		void Reset();							//!< Reset the navigation solution to naught values





};


#endif
