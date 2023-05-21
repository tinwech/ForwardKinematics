#include "acclaim/motion.h"
#include <fstream>
#include <iostream>
#include <utility>

#include "simulation/kinematics.h"
#include "util.h"

namespace acclaim {
Motion::Motion(const util::fs::path &amc_file, std::unique_ptr<Skeleton> &&_skeleton) noexcept
    : skeleton(std::move(_skeleton)) {
    postures.reserve(1024);
    posturesBackup.reserve(1024);
    if (!this->readAMCFile(amc_file)) {
        std::cerr << "Error in reading AMC file, this object is not initialized!" << std::endl;
        std::cerr << "You can call readAMCFile() to initialize again" << std::endl;
        postures.resize(0);
        posturesBackup.resize(0);
    }
}
Motion::Motion(const Motion &a, const Motion &b) noexcept
    : skeleton(std::make_unique<Skeleton>(*a.skeleton)),
      postures(a.postures),
      posturesBackup(a.posturesBackup) {
    postures = motionBlend(a.postures, b.postures);
}

const std::unique_ptr<Skeleton> &Motion::getSkeleton() const { return skeleton; }

Motion::Motion(const Motion &other) noexcept
    : skeleton(std::make_unique<Skeleton>(*other.skeleton)),
      postures(other.postures),
      posturesBackup(other.posturesBackup) {}

Motion::Motion(Motion &&other) noexcept
    : skeleton(std::move(other.skeleton)),
      postures(std::move(other.postures)),
      posturesBackup(std::move(other.postures)) {}

Motion &Motion::operator=(const Motion &other) noexcept {
    if (this != &other) {
        skeleton.reset();
        skeleton = std::make_unique<Skeleton>(*other.skeleton);
        postures = other.postures;
        posturesBackup = other.posturesBackup;
    }
    return *this;
}

Motion &Motion::operator=(Motion &&other) noexcept {
    if (this != &other) {
        skeleton = std::move(other.skeleton);
        postures = std::move(other.postures);
        posturesBackup = std::move(other.posturesBackup);
    }
    return *this;
}

int Motion::getFrameNum() const { return static_cast<int>(postures.size()); }

void Motion::setBoneTransform(int frame_idx) {
    kinematics::forwardSolver(postures[frame_idx], skeleton->getBonePointer(0));
    skeleton->setModelMatrices();
}

void Motion::timeWarper(int oldframe, int newframe) { postures = kinematics::timeWarper(posturesBackup, oldframe, newframe); }

std::vector<Posture> Motion::motionBlend(const std::vector<Posture>& a, const std::vector<Posture>& b) {
    
    int total_bones = static_cast<int>(a[0].bone_rotations.size());

    int blend_frames = 50;
    std::vector<Eigen::Vector4d> translation_offsets(total_bones);
	for (int i = 0; i < total_bones; i++) {
		translation_offsets[i] = a[a.size() - blend_frames].bone_translations[i] - b[0].bone_translations[i];
	}

    std::vector<Posture> new_postures;
    for (int i = 0; i < a.size() - blend_frames; i++) {
        new_postures.push_back(a[i]); 
    }
    for (int i = 0; i < blend_frames; i++) {
        Posture posture_A = a[a.size() - blend_frames + i];
        Posture posture_B = b[i];
        float ratio = (float)(i + 1) / blend_frames;

        Posture new_posture(total_bones);
        for (int j = 0; j < total_bones; j++) {
            posture_B.bone_translations[j] += translation_offsets[j];

			new_posture.bone_translations[j] =
				(1 - ratio) * posture_A.bone_translations[j] + ratio * posture_B.bone_translations[j];

			Eigen::Vector4d euler_a = posture_A.bone_rotations[j];
			Eigen::Vector4d euler_b = posture_B.bone_rotations[j];
			Eigen::Quaterniond quat_a = util::rotateDegreeXYZ(euler_a.x(), euler_a.y(), euler_a.z()).normalized();
			Eigen::Quaterniond quat_b = util::rotateDegreeXYZ(euler_b.x(), euler_b.y(), euler_b.z()).normalized();

			Eigen::Quaterniond quat = quat_a.slerp(ratio, quat_b);
			Eigen::Vector3d euler = quat.normalized().toRotationMatrix().eulerAngles(0, 1, 2) * 180 / EIGEN_PI;

			new_posture.bone_rotations[j][0] = euler.x();
			new_posture.bone_rotations[j][1] = euler.y();
			new_posture.bone_rotations[j][2] = euler.z();
        }
        new_postures.push_back(new_posture);
    }

    for (int i = blend_frames; i < b.size(); i++) {
        Posture new_posture(total_bones);
        for (int j = 0; j < total_bones; j++) {
            new_posture.bone_translations[j] = b[i].bone_translations[j] + translation_offsets[j];
            new_posture.bone_rotations[j] = b[i].bone_rotations[j];
        }
        new_postures.push_back(new_posture); 
    }
    return new_postures;
}

bool Motion::readAMCFile(const util::fs::path &file_name) {
    // Open AMC file
    std::ifstream input_stream(file_name);
    // Check if file successfully opened
    if (!input_stream) {
        std::cerr << "Failed to open " << file_name << std::endl;
        return false;
    }
    // There are (NUM_BONES_IN_ASF_FILE - 2) moving bones and 2 dummy bones (lhipjoint and rhipjoint)
    if (skeleton)
        std::cout << "skeleton exist" << std::endl;
    else
        std::cout << "skeleton is null" << std::endl;
    int movable_bones = skeleton->getMovableBoneNum();
    // Ignore header
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    int frame_num;
    std::string bone_name;
    while (input_stream >> frame_num) {
        auto &&current_posture = postures.emplace_back(skeleton->getBoneNum());
        for (int i = 0; i < movable_bones; ++i) {
            input_stream >> bone_name;
            const Bone &bone = *skeleton->getBonePointer(bone_name);
            int bone_idx = bone.idx;
            Eigen::Vector4d bone_rotation = Eigen::Vector4d::Zero();
            Eigen::Vector4d bone_translation = Eigen::Vector4d::Zero();
            if (bone.doftx) {
                input_stream >> bone_translation[0];
            }
            if (bone.dofty) {
                input_stream >> bone_translation[1];
            }
            if (bone.doftz) {
                input_stream >> bone_translation[2];
            }
            if (bone.dofrx) {
                input_stream >> bone_rotation[0];
            }
            if (bone.dofry) {
                input_stream >> bone_rotation[1];
            }
            if (bone.dofrz) {
                input_stream >> bone_rotation[2];
            }
            current_posture.bone_rotations[bone_idx] = std::move(bone_rotation);
            current_posture.bone_translations[bone_idx] = std::move(bone_translation);
            if (bone_idx == 0) {
                current_posture.bone_translations[bone_idx] *= skeleton->getScale();
            }
        }
    }
    input_stream.close();
    std::cout << frame_num << " samples in " << file_name.string() << " are read" << std::endl;
    posturesBackup = postures;
    return true;
}
}  // namespace acclaim
