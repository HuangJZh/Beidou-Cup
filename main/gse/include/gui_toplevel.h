#ifndef GUI_TOPLEVEL_H
#define GUI_TOPLEVEL_H

#include "gui.h"

/*----------------------------------------------------------------------------------------------*/
class GUI_Toplevel: public iGUI_Toplevel
{

	private:

		/* Add all the "subwindows" here */
		int count;
		
		
		
		
		
		
		
		
		
		
		
		
		class GUI_Serial*pSerial;
		
		wxTimer 		*timer;
		wxString		status_str;	
		
		
		
		
		
		
		
		Message_Struct	messages;						//!< Hold all the messages

//		float 			kB_sec;
		uint32			last_tic;						//!< Only update when new info is available

	public:

		GUI_Toplevel();
		~GUI_Toplevel();

		void onTimer(wxTimerEvent& evt);
		void onClose(wxCloseEvent& evt);
		void onQuit(wxCommandEvent& event);
		void onAbout(wxCommandEvent& event);




//		void onSerial(wxCommandEvent& event);
//		void onNpipe(wxCommandEvent& event);
//		void onUSB(wxCommandEvent& event);












//		void onMessages(wxCommandEvent& event);


		void paintEvent(wxPaintEvent& evt);
	    void paintNow();

	    void render(wxDC& dc);
//	    void renderRS422();
//	    void renderTask();

		DECLARE_EVENT_TABLE()

};
/*----------------------------------------------------------------------------------------------*/

#endif
