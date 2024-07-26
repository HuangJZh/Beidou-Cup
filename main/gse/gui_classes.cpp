///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Feb 16 2016)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "gui_classes.h"

///////////////////////////////////////////////////////////////////////////

iGUI_Toplevel::iGUI_Toplevel( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	mMenuBar = new wxMenuBar( 0 );
	mFile = new wxMenu();
	wxMenuItem* mAbout;
	mAbout = new wxMenuItem( mFile, ID_ABOUT, wxString( wxT("About") ) , wxEmptyString, wxITEM_NORMAL );
	mFile->Append( mAbout );
	
	wxMenuItem* mExit;
	mExit = new wxMenuItem( mFile, ID_QUIT, wxString( wxT("Exit") ) , wxEmptyString, wxITEM_NORMAL );
	mFile->Append( mExit );
	
	mMenuBar->Append( mFile, wxT("File") ); 
	
	this->SetMenuBar( mMenuBar );
	
	wxBoxSizer* sMain;
	sMain = new wxBoxSizer( wxHORIZONTAL );
	
	wxBoxSizer* bSizer115;
	bSizer115 = new wxBoxSizer( wxHORIZONTAL );
	
	wxStaticBoxSizer* sLeftCam;
	sLeftCam = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Left Camera") ), wxVERTICAL );
	
	pLeftCam = new wxPanel( sLeftCam->GetStaticBox(), wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	sLeftCam->Add( pLeftCam, 1, wxEXPAND, 5 );
	
	
	bSizer115->Add( sLeftCam, 1, wxALL|wxEXPAND, 5 );
	
	wxStaticBoxSizer* sRightCam;
	sRightCam = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Right Camera") ), wxVERTICAL );
	
	pRightCam = new wxPanel( sRightCam->GetStaticBox(), wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	sRightCam->Add( pRightCam, 1, wxEXPAND | wxALL, 5 );
	
	
	bSizer115->Add( sRightCam, 1, wxALL|wxEXPAND, 5 );
	
	
	sMain->Add( bSizer115, 1, wxEXPAND, 5 );
	
	
	this->SetSizer( sMain );
	this->Layout();
	mStatus = this->CreateStatusBar( 1, wxST_SIZEGRIP, wxID_ANY );
	
	this->Centre( wxBOTH );
}

iGUI_Toplevel::~iGUI_Toplevel()
{
}
