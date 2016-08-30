// Author: Felix Duvallet on 8/30/16.

#ifndef PROJECT_LOADSHAREPARAMS_H
#define PROJECT_LOADSHAREPARAMS_H

namespace load_share_estimation {

class LoadShareParameters {

 public:
  LoadShareParameters() { }

  bool fromParamServer() {
    return false;
  }

  std::string calibration_orientation_param_name;

};

}   // namespace load_share_estimation

#endif //PROJECT_LOADSHAREPARAMS_H
