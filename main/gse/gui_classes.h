///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Feb 16 2016)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __GUI_CLASSES_H__
#define __GUI_CLASSES_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/panel.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/statusbr.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

#define ID_ABOUT 1000
#define ID_QUIT 1001

///////////////////////////////////////////////////////////////////////////////
/// Class iGUI_Toplevel
///////////////////////////////////////////////////////////////////////////////
class iGUI_Toplevel : public wxFrame 
{
	private:
	
	protected:
		wxMenuBar* mMenuBar;
		wxMenu* mFile;
		wxPanel* pLeftCam;
		wxPanel* pRightCam;
		wxStatusBar* mStatus;
	
	public:
		
		iGUI_Toplevel( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("FAIM"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 1500,500 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		
		~iGUI_Toplevel();
	
};

#endif //__GUI_CLASSES_H__
