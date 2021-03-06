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
Velocity HtcViveTrackerAlgorithm::GetDeviceVelocity(const std::string & device_name) {
  if (this->htc_vive_.IsDeviceDetected(device_name)){
	return this->htc_vive_.GetDeviceVelocity (device_name);
  }
  else {
	Velocity v;
	return v;
  }
}
bool HtcViveTrackerAlgorithm::GetDeviceNames(std::vector<std::string>&device_names){

	device_names = this->htc_vive_.GetAllDeviceNames();
	return true;
}

bool HtcViveTrackerAlgorithm::IsDeviceDetected (const std::string device_name){

	std::vector<std::string>device_names = this->htc_vive_.GetAllDeviceNames();
	for (int i = 0; i < device_names.size(); ++i) {
		if (device_names[i] == device_name) return true;
	}
	return false;

}
Dimension HtcViveTrackerAlgorithm::GetChaperoneDimensions(){
	
	return this->htc_vive_.GetChaperoneDimensions();
}
bool HtcViveTrackerAlgorithm::PollEvents(){
	return this->htc_vive_.EventPolling();
}

float HtcViveTrackerAlgorithm::GetBatteryLevel(const std::string & device_name){
	return this->htc_vive_.GetBatteryLevel(device_name);
}

vr::EVRButtonId HtcViveTrackerAlgorithm::GetPressedButton(const std::string & device_name){
	return this->htc_vive_.GetLastButtonPressedEnum(device_name);
}
bool HtcViveTrackerAlgorithm::TriggerHapticPulse(const std::string & device_name, uint32_t strength){
	return this->htc_vive_.HapticPulse(device_name,0,strength);
}
