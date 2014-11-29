;OpenGex NSIS Kits Installer
;James Huntsman

;--------------------------------
;Include Modern UI

	!include "MUI2.nsh"
	!include "WinVer.nsh"
	!include "LogicLib.nsh"
	!include "x64.nsh"

;--------------------------------
;Custom Defines

	!define BuildSourcePath ".\OpenGex"
	!define BuildSourceExtra ".\Extra"
	!define ContentInstallDir "$DOCUMENTS\Luxology\Content\"
	!define productName "OpenGex"
	!define versionName "1.0 Beta"
	!define uninstallDir "$LOCALAPPDATA\Luxology\modo\OpenGex\"
	!define kitsDesc "OpenGex exporter for MODO"

	!define SetContent $8
	!define regKeyInstallDir $9

;--------------------------------

;General

  ;Name and file
  Name "OpenGex Exporter"
  OutFile "ModoOpenGexExporter.exe"
  BrandingText "MODO OpenGex Exporter"

  ;Default installation folder
  InstallDir "$DOCUMENTS\Luxology\Content\Kits\OpenGex\"
  
  ;Get installation folder from registry if available

  ;Request application privileges for Windows Vista
  RequestExecutionLevel none
  
;--------------------------------

;Interface Configuration

	!define MUI_HEADERIMAGE
	;!define MUI_HEADERIMAGE_BITMAP "${BuildSourceExtra}\PATH TO ART\MYIMAGE.BMP" ; optional
	;!define MUI_ICON "${BuildSourceExtra}\PATH TO ICONS\MyIcon128.ico"
  
	!define MUI_ABORTWARNING
	!define MUI_INSTFILESPAGE_COLORS "9db3c2 000000"
	

;--------------------------------

;Pages

	;!insertmacro MUI_PAGE_LICENSE "${BuildSourceExtra}\Common\EULA\EULA_501.rtf"
	!insertmacro MUI_PAGE_INSTFILES

;uninstall
	
	!insertmacro MUI_UNPAGE_INSTFILES
	
;--------------------------------

;Languages
 
	!insertmacro MUI_LANGUAGE "English"

;--------------------------------

;Sections
	
	Section "-My Install Section" SecContent
	SetShellVarContext current
	
	SetOutPath "$INSTDIR\"
	
	File /r "${BuildSourcePath}\*.*"
	
SetShellVarContext all
	
	;Create uninstaller
	CreateDirectory "${uninstallDir}"
	WriteUninstaller "${uninstallDir}uninstall.exe"
	
	SetOutPath "$SMPROGRAMS\Luxology\modo\OpenGex"
	CreateShortCut "$SMPROGRAMS\Luxology\modo\OpenGex\Uninstall.lnk" "${uninstallDir}uninstall.exe"
	
	WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\MODOOpenGex" "DisplayName" "${productName}"
	
SectionEnd

;--------------------------------

;Descriptions

	LangString DESC_SecContent ${LANG_ENGLISH} "OpenGex"

  
;--------------------------------

;Uninstaller Section

Section "un.Uninstall Content" uncontent
	SetShellVarContext current
	
	;ADD YOUR OWN FILES HERE...
	RMDir /r "$DOCUMENTS\Luxology\Content\Kits\OpenGex\"

	SetShellVarContext all
	DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\MODOOpenGex"
	
	Delete "${uninstallDir}uninstall.exe"
	
	Delete "$SMPROGRAMS\Luxology\modo\OpenGex\Uninstall.lnk"
	RMDir "$SMPROGRAMS\Luxology\modo\OpenGex\"
SectionEnd

;--------------------------------

;functions
Function .onInit
	
	InitPluginsDir
   
FunctionEnd