#include "htc_vive_tracker_alg.h"

HtcViveTrackerAlgorithm::HtcViveTrackerAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

HtcViveTrackerAlgorithm::~HtcViveTrackerAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void HtcViveTrackerAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}


// HtcViveTrackerAlgorithm Public API
bool HtcViveTrackerAlgorithm::InitVR(bool verbose){
	return this->htc_vive_.InitializeVR(verbose);
}
bool HtcViveTrackerAlgorithm::GetDevicePositionQuaternion (const std::string & device_name, double (&pose)[3], double (&quaternion)[4]){
  if (this->htc_vive_.IsDeviceDetected(device_name)){
	return this->htc_vive_.GetDevicePoseQuaternion (device_name, pose, quaternion);
  }
  else return false;
}

bool HtcViveTrackerAlgorithm::GetDevicePositionEuler (const std::string & device_name, double (&pose)[3], double & roll, double & pitch, double & yaw) {
  if (this->htc_vive_.IsDeviceDetected(device_name)){
	return this->htc_vive_.GetDevicePoseEuler (device_name, pose, roll, pitch, yaw);
  }
  else return false;

}
bool HtcViveTrackerAlgorithm::GetDeviceVelocity (const std::string & device_name, double (&linear_v)[3], double (&angular_v)[3]){
  if (this->htc_vive_.IsDeviceDetected(device_name)){
	return this->htc_vive_.GetDeviceVelocity (device_name, linear_v,angular_v);
  }
  else return false;
}
bool HtcViveTrackerAlgorithm::GetDeviceNames(std::vector<std::string>&device_names){

	device_names = this->htc_vive_.GetAllDeviceNames();
	return true;
}

bool HtcViveTrackerAlgorithm::GetChaperoneDimensions(std::vector<std::vector<float> > & corners, float & sizeX, float & sizeY){
	
	return this->htc_vive_.GetChaperoneDimensions(corners,sizeX,sizeY);
}
bool HtcViveTrackerAlgorithm::PollEvents(){
	return this->htc_vive_.EventPolling();
}

float HtcViveTrackerAlgorithm::GetBatteryLevel(const std::string & device_name){
	return this->htc_vive_.GetBatteryLevel(device_name);
}

ButtonFlags HtcViveTrackerAlgorithm::GetPressedButton(){
	return this->htc_vive_.GetLastButtonPressed();
}
bool HtcViveTrackerAlgorithm::TriggerHapticPulse(const std::string & device_name, uint32_t strength){
	return this->htc_vive_.HapticPulse(device_name,0,strength);
}
