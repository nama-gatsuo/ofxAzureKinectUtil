
meta:
	ADDON_NAME = ofxAzureKinectUtil
	ADDON_DESCRIPTION = Use Azure Kinect inside openFrameworks.
	ADDON_AUTHOR = Ayumu Nagamatsu
	ADDON_TAGS = "computer vision" "3D sensing" "kinect"
	ADDON_URL = https://github.com/nama-gatsuo/ofxAzureKinectUtil

common:

vs:
	ADDON_INCLUDES += $(AZUREKINECT_SDK)\sdk\include
	ADDON_INCLUDES += $(AZUREKINECT_BODY_SDK)\sdk\include
	ADDON_LIBS += $(AZUREKINECT_SDK)\sdk\windows-desktop\amd64\release\lib\k4a.lib
	ADDON_LIBS += $(AZUREKINECT_BODY_SDK)\sdk\windows-desktop\amd64\release\lib\k4abt.lib
	
linux64: 
	
linux:

linuxarmv6l:

linuxarmv7l:
	
msys2:

android/armeabi:	
	
android/armeabi-v7a:	

ios:


