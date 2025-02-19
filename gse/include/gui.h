/*! \file GUI.h
	Define the wxWidgets GUI App and object
*/
/************************************************************************************************
Copyright 2008 Gregory W Heckler

This file is part of the GPS Software Defined Radio (GPS-SDR)

The GPS-SDR is free software; you can redistribute it and/or modify it under the terms of the
GNU General Public License as published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The GPS-SDR is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along with GPS-SDR; if not,
write to the:

Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
************************************************************************************************/

#ifndef GUI_H
#define GUI_H
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
/*Herein Lies Many Important Files*/
/*Note thy order is important!*///xin: very important!!
/*----------------------------------------------------------------------------------------------*/
#include "defines.h"			//!< GPS ICD-2000 defines
#include "config.h"				//!< Configuration Options
#include <gtsam/navigation/ImuBias.h>	//!< xin: namespace 'gtsam'
#include <Eigen/Dense>					//!< xin: namespace 'Eigen'
#include "sdr_structs.h"				//!< xin: struct definition of 'CamMeasurement' and 'ImuMeasurement', requested by 'SV_Prediction_M' (redefined by xin) in messages.h
#include "messages.h"			//!< RS422 messages
//xin//#include "commands.h"			//!< RS422 commands
#include <termios.h>			//!< Serial stuff
#include <signal.h>				//!< Pipe stuff
/*----------------------------------------------------------------------------------------------*/

/* wxWidgets headers */
/*----------------------------------------------------------------------------------------------*/
#include "wx/wx.h"
#include <wx/app.h>
#include <wx/statusbr.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/dcbuffer.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/frame.h>
#include <wx/toolbar.h>
#include <wx/log.h>
#include <wx/process.h>
/*----------------------------------------------------------------------------------------------*/

/* Include standard headers, OS stuff */
/*----------------------------------------------------------------------------------------------*/
#include "threaded_object.h"
//xin//#include "signaldef.h"
//xin//#include "macros.h"
#include "gui_classes.h"
//xin//#include "gui_object.h"
#include "gui_serial.h"
//xin//#include "gui_pvt.h"
//xin//#include "gui_ekf.h"
//xin//#include "gui_almanac.h"
//xin//#include "gui_ephemeris.h"
//xin//#include "gui_channel.h"
//xin//#include "gui_commands.h"
//xin//#include "gui_pseudo.h"
//#include "gui_eeprom.h"
//xin//#include "gui_health.h"
//xin//#include "gui_messages.h"
//xin//#include "gui_select.h"
//xin//#include "gui_speedo.h"
#include "gui_toplevel.h"
/*----------------------------------------------------------------------------------------------*/
//xin//double icn0_2_fcn0(uint32 _cn0);

//#define ID_EXIT  1000
#define ID_TIMER 9999
/*----------------------------------------------------------------------------------------------*/
class GUI_App: public wxApp
{

	GUI_Toplevel *pMain;

    bool render_loop_on;
    virtual bool OnInit();
    void onIdle(wxIdleEvent& evt);

public:

    void activateRenderLoop(bool on);
    bool getRenderLoop(){return(render_loop_on);}

};
/*----------------------------------------------------------------------------------------------*/

#endif
