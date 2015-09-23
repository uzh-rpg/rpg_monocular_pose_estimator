// This file is part of RPG-MPE - the RPG Monocular Pose Estimator
//
// RPG-MPE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RPG-MPE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with RPG-MPE.  If not, see <http://www.gnu.org/licenses/>.

#include <memory>
#include <pluginlib/class_list_macros.h>

#include "monocular_pose_estimator/nodelet.h"
#include "monocular_pose_estimator/monocular_pose_estimator.h"

namespace monocular_pose_estimator {

void MPENodelet::onInit()
{
  mpe_node = std::make_shared<MPENode>(getNodeHandle(), getPrivateNodeHandle());
  NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
}

} // namespace sample_nodelet

PLUGINLIB_EXPORT_CLASS(monocular_pose_estimator::MPENodelet, nodelet::Nodelet)
