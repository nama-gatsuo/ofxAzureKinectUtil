# ofxAzureKinectUtil

[openFrameworks](https://openframeworks.cc/) addon for bridging [Azure Kinect](https://azure.microsoft.com/en-us/services/kinect-dk/). This addon is kind a child of [ofxAzureKinect](https://github.com/prisonerjohn/ofxAzureKinect) and succeeds main architecture from it. Yet I had to use some extra features like playback recordings in second thread and so on.

## Feature
* **Streaming** from `ofxAzureKinectUtil::Device`
* **Playback** of `.mkv` file via `ofxAzureKinectUtil::Playback`
* **Multithreading** to process sensor inputs
* **Pointcloud** as `ofVboMesh` and texture binding
* Raw data (depth/color/IR) access as `ofTexture`s
* Device orientation estimation from IMU

## Setup
* Install the [Azure Kinect Sensor SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/sensor-sdk-download).
* Install the [Azure Kinect Body Tracking SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/body-sdk-download).
* Add an environment variable for `AZUREKINECT_SDK` and set it to the Sensor SDK installation path (no trailing slash). The default is `C:\Program Files\Azure Kinect SDK v1.3.0` (versions might be changed).
* Add an environment variable for `AZUREKINECT_BODY_SDK` and set it to the Body SDK installation path (no trailing slash). The default is `C:\Program Files\Azure Kinect Body Tracking SDK`.
* Add the path to the Sensor SDK `bin` folder to the `PATH` variable. The default is `%AZUREKINECT_SDK%\sdk\windows-desktop\amd64\release\bin`.
* Add the path to the Body SDK `bin` folder to the `PATH` variable. The default is `%AZUREKINECT_BODY_SDK%\sdk\windows-desktop\amd64\release\bin`.
* Add the path to the Body SDK `tools` folder to the `PATH` variable. The default is `%AZUREKINECT_BODY_SDK%\tools`.
* Clone this repository in your openFrameworks `addons` folder.
* You can then use the OF Project Generator to generate projects with the appropriate headers and libraries included.
* Note that if you want to use body tracking, you will need to copy the cuDNN model file `dnn_model_2_0.onnx` from the Body SDK `tools` folder into your project's `bin` folder!
