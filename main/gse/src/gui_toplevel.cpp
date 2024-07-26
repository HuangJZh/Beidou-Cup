






#include "gui.h"

DECLARE_APP(GUI_App)

/*----------------------------------------------------------------------------------------------*/
BEGIN_EVENT_TABLE(GUI_Toplevel, wxFrame)
	EVT_MENU(ID_QUIT,				GUI_Toplevel::onQuit)
    EVT_MENU(ID_ABOUT,				GUI_Toplevel::onAbout)
	
	
	
	
//	EVT_MENU(ID_SERIAL		,		GUI_Toplevel::onSerial)
//	EVT_MENU(ID_NPIPE,				GUI_Toplevel::onNpipe)
    EVT_TIMER(ID_TIMER,				GUI_Toplevel::onTimer)

	
	
	
	
	
	
	
	
	
	
	
	EVT_PAINT(GUI_Toplevel::paintEvent)
    EVT_CLOSE(GUI_Toplevel::onClose)
END_EVENT_TABLE()
/*----------------------------------------------------------------------------------------------*/
		

//GUI_Toplevel::GUI_Toplevel():iGUI_Toplevel(NULL, wxID_ANY, wxT("FAIM"), wxDefaultPosition, wxSize( 1500,500 ), wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL, wxT("FAIM"));
GUI_Toplevel::GUI_Toplevel():iGUI_Toplevel(NULL, wxID_ANY, wxT("FAIM"), wxDefaultPosition, wxSize( 1500,500 ), wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL )
{

/*	wPVT 			= NULL;
	wEKF 			= NULL;
	wChannel 		= NULL;
	wPseudo 		= NULL;
	wEphemeris 		= NULL;
	wAlmanac 		= NULL;
	wHealth 		= NULL;
	wCommands 		= NULL;
	//wEEPROM 		= NULL;
	wSpeedo 		= NULL;
*/
	pSerial = new GUI_Serial;
	pSerial->setIO(0);
    pSerial->Start();
/*
    kB_sec = 0;
*/	last_tic = 0;

    timer = new wxTimer(this, ID_TIMER);
    timer->Start(100, wxTIMER_CONTINUOUS); //Shoot for 20 fps
    
//	log_filename.Clear();

}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
GUI_Toplevel::~GUI_Toplevel()
{
	pSerial->Stop();
	delete pSerial;
/*
	if(wPVT)
		delete wPVT;

	if(wEKF)
		delete wEKF;

	if(wChannel)
		delete wChannel;

	if(wPseudo)
		delete wPseudo;

	if(wAlmanac)
		delete wAlmanac;

	if(wEphemeris)
		delete wEphemeris;

	if(wCommands)
		delete wCommands;

	if(wSpeedo)
		delete wSpeedo;

//	if(wEEPROM)
//		delete wEEPROM;
*/
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Toplevel::onTimer(wxTimerEvent& evt)
{
	paintNow();
}
/*----------------------------------------------------------------------------------------------*/





/*----------------------------------------------------------------------------------------------*/
void GUI_Toplevel::onQuit(wxCommandEvent& WXUNUSED(event))
{

//	wxCommandEvent event;

    Close(TRUE);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Toplevel::onAbout(wxCommandEvent& WXUNUSED(event))
{
	wxString message;

//	message = wxT("FAIM: Fusion with Autonomous Integrity Monitoring\nCopyright 2022 Xin Zhang\nPath: ");
	message = wxT("FAIM: Fusion with Autonomous Integrity Monitoring\nCopyright 2023 Xin Zhang\nPath: ");
	message += wxGetCwd();

	wxMessageBox(message,wxT("About FAIM"), wxOK | wxICON_INFORMATION, this);
}
/*----------------------------------------------------------------------------------------------*/




















































































































/*----------------------------------------------------------------------------------------------*/
/*
void GUI_Toplevel::onSerial(wxCommandEvent& WXUNUSED(event))
{
	pSerial->Lock();
	pSerial->setIO(1);
	pSerial->Unlock();
}*/
/*----------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------*/
/*
void GUI_Toplevel::onNpipe(wxCommandEvent& WXUNUSED(event))
{
	pSerial->Lock();
	pSerial->setIO(0);
	pSerial->Unlock();
}*/
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Toplevel::onClose(wxCloseEvent& evt)
{

    wxGetApp().activateRenderLoop(false);
    evt.Skip(); // don't stop event, we still want window to close
    //Close(TRUE);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Toplevel::paintEvent(wxPaintEvent& evt)
{
    wxPaintDC dc(this);
    render(dc);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Toplevel::paintNow()
{
    wxClientDC dc(this);
    render(dc);
}
/*----------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------*/
void GUI_Toplevel::render(wxDC& dc)
{

//	int page, this_tic;
//	int update;
    wxString str;

	if(pSerial->TryLock() == 0)
	{
		if(pSerial->messages.sps.tic != last_tic)
		{
			memcpy(&messages, pSerial->GetMessages(), sizeof(Message_Struct));
			last_tic = pSerial->messages.sps.tic;
//			update = true;
		}
		else
		{
//			update = false;
		}
		pSerial->Unlock();
	}

	/* Render RS422 Panel */
//xin//	renderRS422();

//	if(update)//TODO 20220410
	{

		/* Render Task Panel */
//xin//		renderTask();

		str.Printf(wxT("Count: %d"),count++);
		str += status_str;
/*//xin//
		SetStatusText(str);

		// Render main window
		if(wPVT != NULL)
			wPVT->paintNow();

		// Render main window
		if(wEKF != NULL)
			wEKF->paintNow();

		// Render channel window
		if(wChannel != NULL)
			wChannel->paintNow();

		// Render channel window
		if(wPseudo != NULL)
			wPseudo->paintNow();

		// Render channel window
		if(wAlmanac != NULL)
			wAlmanac->paintNow();

		// Render channel window
		if(wSelect != NULL)
			wSelect->paintNow();

		// Render channel window
		if(wEphemeris != NULL)
			wEphemeris->paintNow();

		// Render SV Select window
		if(wHealth != NULL)
			wHealth->paintNow();

		// Display EEPROM
//		if(wEEPROM != NULL)
//			wEEPROM->paintNow();

		// Display Messages
		if(wMessages != NULL)
			wMessages->paintNow();

		// Display Speedo
		if(wSpeedo != NULL)
			wSpeedo->paintNow();*/
	
		/* Status Text */
		SV_Prediction_M * pSV = &pSerial->GetMessages()->sv_predictions[0];//sv_predictions[sv_id];
		
		wxString str2;
//		str2.Printf(wxT("  mvTimestamp from gps-sdr: %ld"), (pSV->mCamMeas).mvTimestamp);
		str2.Printf(wxT("  mvTimestamp from FAIM: %ld"), pSV->miTimestamp_Cam[CAM0]);
		str += str2;
		SetStatusText(str);


		/* Stereo Images */	
		////////////////////////// left cam //////////////////////////
		std::string img_for_gse_C0 = std::string(pSV->msImage[CAM0]);
		if (!img_for_gse_C0.empty())
		{
			wxPrintf("<><<><><><><> GUI_Toplevel: render()1                                  = %s\n", img_for_gse_C0.c_str());

			std::string img_for_gse_C0_front = img_for_gse_C0.substr(0,24);
			std::string img_for_gse_C0_end = img_for_gse_C0.substr(24);
			img_for_gse_C0 = img_for_gse_C0_front + "_faim" + img_for_gse_C0_end;

			wxBitmap imageL;
			imageL.LoadFile(img_for_gse_C0,
							wxBITMAP_TYPE_PNG);

			wxClientDC dc1(pLeftCam);
			dc1.DrawBitmap(imageL, 0, 0, false);
			wxPen penL(wxColour(255, 0, 0), //red
				    	1);                  //width
			dc1.SetPen(penL);
			
//			wxPrintf("muCnt_un_pts[CAM0] = %i\n", pSV->muCnt_un_pts[CAM0]);
			size_t iSize0 = pSV->muCnt_un_pts[CAM0];
			//for (size_t lcv = 0; lcv < pSV->muCnt_un_pts[CAM0]; lcv++)
			for (size_t lcv = 0; lcv < iSize0; lcv++)
			{
//				wxPrintf("(un_x, un_y) = (%f,%f)\n", pSV->mfCur_un_pts_x[CAM0][lcv], pSV->mfCur_un_pts_y[CAM0][lcv]);
				dc1.DrawCircle((wxCoord)pSV->mfCur_un_pts_x[CAM0][lcv],
							   (wxCoord)pSV->mfCur_un_pts_y[CAM0][lcv],
							   3);//radius
			}
//			wxPrintf("\n");

		}

		////////////////////////// right cam //////////////////////////
		std::string img_for_gse_C1 = std::string(pSV->msImage[CAM1]);
		if (!img_for_gse_C1.empty())
		{
			wxPrintf("<><<><><><><> GUI_Toplevel: render()2                                  = %s\n", img_for_gse_C1.c_str());

			std::string img_for_gse_C1_front = img_for_gse_C1.substr(0,24);
			std::string img_for_gse_C1_end = img_for_gse_C1.substr(24);
			img_for_gse_C1 = img_for_gse_C1_front + "_faim" + img_for_gse_C1_end;

			wxBitmap imageR;
			imageR.LoadFile(img_for_gse_C1,
							wxBITMAP_TYPE_PNG);

			wxClientDC dc2(pRightCam);
			dc2.DrawBitmap(imageR, 0, 0, false);
			wxPen penR(wxColour(255, 0, 0), //red
				    	1);                  //width
			dc2.SetPen(penR);
			
//			wxPrintf("muCnt_un_pts[CAM1] = %i\n", pSV->muCnt_un_pts[CAM1]);
			size_t iSize1 = pSV->muCnt_un_pts[CAM1];
			//for (size_t lcv = 0; lcv < pSV->muCnt_un_pts[CAM1]; lcv++)
			for (size_t lcv = 0; lcv < iSize1; lcv++)
			{
//				wxPrintf("(un_x, un_y) = (%f,%f)\n", pSV->mfCur_un_pts_x[CAM1][lcv], pSV->mfCur_un_pts_y[CAM1][lcv]);
				dc2.DrawCircle((wxCoord)pSV->mfCur_un_pts_x[CAM1][lcv],
							   (wxCoord)pSV->mfCur_un_pts_y[CAM1][lcv],
							   3);//radius
			}
//			wxPrintf("\n");

		}

    }

}
/*----------------------------------------------------------------------------------------------*/


