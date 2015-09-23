#include <nodelet/nodelet.h>
#include <memory>
#include "monocular_pose_estimator/monocular_pose_estimator.h"

namespace monocular_pose_estimator {

class MPENodelet : public nodelet::Nodelet {
public:
  virtual void onInit();
protected:
  std::shared_ptr<MPENode> mpe_node;
};

}
