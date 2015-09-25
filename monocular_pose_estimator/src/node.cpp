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

/*
 * node.cpp
 *
 * Created on: Jul 29, 2013
 * Author: Karl Schwabe
 */

#include "monocular_pose_estimator/monocular_pose_estimator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "monocular_pose_tracker");

  monocular_pose_estimator::MPENode mpe_node;

  ros::spin();

  return 0;
}
