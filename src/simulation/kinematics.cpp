#include "simulation/kinematics.h"

#include "Eigen/Dense"
#include <iostream>
#include "acclaim/bone.h"
#include "util/helper.h"
#include <queue>
#include <unordered_map>

namespace kinematics {
void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* root) {
    // TODO#1 (FK)
    // You should set these variables:
    //     bone->start_position = Eigen::Vector4d::Zero();
    //     bone->end_position = Eigen::Vector4d::Zero();
    //     bone->rotation = Eigen::Matrix4d::Zero();
    // The sample above just set everything to zero
    // Hint:
    //   1. posture.bone_translations, posture.bone_rotations
    // Note:
    //   1. This function will be called with bone == root bone of the skeleton
    //   2. we use 4D vector to represent 3D vector, so keep the last dimension as "0"
    //   3. util::rotate{Degree | Radian} {XYZ | ZYX}
    //      e.g. rotateDegreeXYZ(x, y, z) means:
    //      x, y and z are presented in degree rotate z degrees along z - axis first, then y degrees along y - axis, and then x degrees along x - axis 

    root->start_position = posture.bone_translations[root->idx]; 
    root->end_position = posture.bone_translations[root->idx]; 
	Eigen::Vector4d euler = posture.bone_rotations[root->idx];
	Eigen::Quaterniond quat = util::rotateDegreeZYX(euler.x(), euler.y(), euler.z());
    root->rotation.linear() = quat.normalized().toRotationMatrix();

    std::queue<acclaim::Bone*> q;
    q.push(root);

    while (!q.empty()) {
        acclaim::Bone* bone = q.front();
        q.pop();
        if (bone != root) {
			bone->start_position = bone->parent->end_position;
			euler = posture.bone_rotations[bone->idx];
            quat = util::rotateDegreeZYX(euler.x(), euler.y(), euler.z());
            bone->rotation.linear() = quat.normalized().toRotationMatrix();
            bone->rotation = bone->parent->rotation * bone->rot_parent_current * bone->rotation;
            Eigen::Vector4d V = bone->dir * bone->length + posture.bone_translations[bone->idx];
            bone->end_position = bone->rotation * V + bone->start_position;
        }
        bone = bone->child;
        while (bone) {
            q.push(bone);
            bone = bone->sibling;
        }
    }
}

std::vector<acclaim::Posture> timeWarper(const std::vector<acclaim::Posture>& postures, int allframe_old, int allframe_new) {
	// TODO#2 (Time warping)
	// original: |--------------|
	// new     : |----------------------|
	// OR
	// original: |--------------|
	// new     : |-------|
	// You should set these variables:
	//     new_postures[i].bone_translations[j] = Eigen::Vector4d::Zero();
	//     new_postures[i].bone_rotations[j] = Eigen::Vector4d::Zero();
	// The sample above just set everything to zero
	// Hint:
	//   1. Scale the frames.
	//   2. You can use linear interpolation with translations.
	//   3. You should use spherical linear interpolation for rotations.

    int total_frames = static_cast<int>(postures.size());
    int total_bones = static_cast<int>(postures[0].bone_rotations.size());
    std::vector<acclaim::Posture> new_postures;
    for (int i = 0; i < allframe_new; ++i) {
        acclaim::Posture new_poseture(total_bones);
        float ratio = (float)i / allframe_new * allframe_old;
        int a = floor(ratio);
        int b = a + 1;
        ratio = ratio - a;
        for (int j = 0; j < total_bones; ++j) {
            new_poseture.bone_translations[j] =
                (1 - ratio) * postures[a].bone_translations[j] + ratio * postures[b].bone_translations[j];
            Eigen::Vector4d euler_a = postures[a].bone_rotations[j];
            Eigen::Vector4d euler_b = postures[b].bone_rotations[j];
            Eigen::Quaterniond quat_a = util::rotateDegreeXYZ(euler_a.x(), euler_a.y(), euler_a.z()).normalized();
            Eigen::Quaterniond quat_b = util::rotateDegreeXYZ(euler_b.x(), euler_b.y(), euler_b.z()).normalized();
            Eigen::Quaterniond quat = quat_a.slerp(ratio, quat_b);
            Eigen::Vector3d euler = quat.normalized().toRotationMatrix().eulerAngles(0, 1, 2) * 180 / EIGEN_PI;
            new_poseture.bone_rotations[j][0] = euler.x();
            new_poseture.bone_rotations[j][1] = euler.y();
            new_poseture.bone_rotations[j][2] = euler.z();
        }
        new_postures.push_back(new_poseture);
    }
    return new_postures;
}
}  // namespace kinematics
