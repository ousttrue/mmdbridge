#ifdef WITH_VMD

#include "d3d9.h"
#include "d3dx9.h"

#include "bridge_parameter.h"
#include "UMStringUtil.h"
#include "UMPath.h"
#include "UMMath.h"
#include "UMVector.h"
#include "UMMatrix.h"

#include <map>
#include <vector>

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <ImathMatrix.h>
#include <ImathQuat.h>

#include <EncodingHelper.h>
#include <Pmd.h>
#include <Pmx.h>
#include <Vmd.h>

#include "MMDExport.h"

template <class T> std::string to_string(T value)
{
	return umbase::UMStringUtil::number_to_string(value);
}

typedef std::shared_ptr<pmd::PmdModel> PMDPtr;
typedef std::shared_ptr<pmx::PmxModel> PMXPtr;
typedef std::shared_ptr<vmd::VmdMotion> VMDPtr;

class FileDataForVMD {
public:
	FileDataForVMD(int i):index(i) {}
	~FileDataForVMD() {}
	FileDataForVMD(const FileDataForVMD &) = delete;
	FileDataForVMD& operator=(const FileDataForVMD &) = delete;

	int index;
	VMDPtr vmd;
	PMDPtr pmd;
	PMXPtr pmx;
	std::map<int, int> parent_index_map;
	std::map<int, std::string> bone_name_map;
	std::map<int, int> physics_bone_map;
	std::map<int, int> ik_bone_map;
	std::map<int, int> ik_frame_bone_map;
	std::map<int, int> fuyo_bone_map;
	std::map<int, int> fuyo_target_map;

	std::vector<D3DMATRIX> bone_matrices;

	/*
	FileDataForVMD(const FileDataForVMD& data) {
		this->vmd = data.vmd;
		this->pmd = data.pmd;
		this->pmx = data.pmx;
		this->parent_index_map = data.parent_index_map;
		this->bone_name_map = data.bone_name_map;
		this->physics_bone_map = data.physics_bone_map;
		this->ik_bone_map = data.ik_bone_map;
		this->ik_frame_bone_map = data.ik_frame_bone_map;
		this->fuyo_bone_map = data.fuyo_bone_map;
		this->fuyo_target_map = data.fuyo_target_map;
	}
	*/

	void SaveMFB(const std::wstring &path)
	{
		std::ofstream os(path.c_str(), std::ios::binary);

		// magic
		os.write("MFB ", 4);

		// bonenames
		int bone_num = ExpGetPmdBoneNum(index);
		os.write((const char*)&bone_num, sizeof(bone_num));
		for (int i = 0; i < bone_num; ++i) {
			const char* bone_name = ExpGetPmdBoneName(index, i);
			auto len = strlen(bone_name);
			os.write(bone_name, len); // cp932
		}

		// frames
		int matrix_count = bone_matrices.size();
		os.write((const char *)&matrix_count, sizeof(matrix_count));
		os.write((const char *)bone_matrices.data(), bone_matrices.size() * sizeof(D3DMATRIX));
	}
};


static UMMat44d to_ummat(const D3DMATRIX& mat)
{
	UMMat44d ummat;
	for (int n = 0; n < 4; ++n)
	{
		for (int m = 0; m < 4; ++m)
		{
			ummat[n][m] = mat.m[n][m];
		}
	}
	return ummat;
}


// from imath
static UMVec4d extractQuat(const UMMat44d &mat)
{
	UMMat44d rot;

	double tr, s;
	double q[4];
	int    i, j, k;
	UMVec4d   quat;

	int nxt[3] = { 1, 2, 0 };
	tr = mat[0][0] + mat[1][1] + mat[2][2];

	// check the diagonal
	if (tr > 0.0) {
		s = sqrt(tr + double(1.0));
		quat.w = s / double(2.0);
		s = double(0.5) / s;

		quat.x = (mat[1][2] - mat[2][1]) * s;
		quat.y = (mat[2][0] - mat[0][2]) * s;
		quat.z = (mat[0][1] - mat[1][0]) * s;
	}
	else {
		// diagonal is negative
		i = 0;
		if (mat[1][1] > mat[0][0])
			i = 1;
		if (mat[2][2] > mat[i][i])
			i = 2;

		j = nxt[i];
		k = nxt[j];
		s = sqrt((mat[i][i] - (mat[j][j] + mat[k][k])) + double(1.0));

		q[i] = s * double(0.5);
		if (s != double(0.0))
			s = double(0.5) / s;

		q[3] = (mat[j][k] - mat[k][j]) * s;
		q[j] = (mat[i][j] + mat[j][i]) * s;
		q[k] = (mat[i][k] + mat[k][i]) * s;

		quat.x = q[0];
		quat.y = q[1];
		quat.z = q[2];
		quat.w = q[3];
	}

	return quat;
}


static umstring to_umpath(const char* path)
{
	const int size = ::MultiByteToWideChar(CP_ACP, 0, (LPCSTR)path, -1, NULL, 0);
	wchar_t* utf16 = new wchar_t[size];
	::MultiByteToWideChar(CP_ACP, 0, (LPCSTR)path, -1, (LPWSTR)utf16, size);
	std::wstring wstr(utf16);
	delete[] utf16;
	return umbase::UMStringUtil::wstring_to_utf16(wstr);
}


class VMDArchive {
public:
	
	// singleton
	static VMDArchive& instance() {
		static VMDArchive instance;
		return instance; 
	}
	VMDArchive(const VMDArchive &) = delete;
	VMDArchive& operator=(const VMDArchive &) = delete;

	std::vector<std::unique_ptr<FileDataForVMD>> m_data;
	std::wstring m_output_path;
	int m_export_mode;

	bool start_vmd_export(
		const std::string& directory_path,
		int export_mode)
	{
		m_data.clear();
		m_output_path.clear();

		BridgeParameter::instance().is_exporting_without_mesh = true;
		if (BridgeParameter::instance().export_fps <= 0)
		{
			return false;
		}

		m_output_path = umbase::UMStringUtil::utf16_to_wstring(umbase::UMStringUtil::utf8_to_utf16(directory_path));
		if (m_output_path.empty())
		{
			m_output_path = BridgeParameter::instance().base_path + L"out\\";
		}

		m_export_mode = export_mode;
		const int pmd_num = ExpGetPmdNum();
		for (int i = 0; i < pmd_num; ++i) {
			const char* filename = ExpGetPmdFilename(i);
			/*
			if (m_data.find(filename) != m_data.end()) {
				// found
				continue;
			}
			*/

			auto data = std::make_unique<FileDataForVMD>(i);
			PMDPtr pmd = pmd::PmdModel::LoadFromFile(filename);
			if (pmd)
			{
				data->pmd = pmd;
			}
			else
			{
				PMXPtr pmx = PMXPtr(new pmx::PmxModel());
				std::ifstream stream(filename, std::ios_base::binary);
				if (stream.good())
				{
					pmx->Init();
					pmx->Read(&stream);
				}

				data->pmx = pmx;
			}
			m_data.push_back(std::move(data));
		}

		return true;
	}

	int m_lastFrame;
	bool execute_vmd_export(int currentframe)
	{
		m_lastFrame = currentframe;
		BridgeParameter::instance().is_exporting_without_mesh = true;

		const int pmd_num = ExpGetPmdNum();
		oguna::EncodingConverter converter;

		if (currentframe == BridgeParameter::instance().start_frame)
		{
			for (int i = 0; i < pmd_num; ++i)
			{
				auto &file_data = m_data[i];
				init_file_data(file_data);

				file_data->vmd = std::make_unique<vmd::VmdMotion>();
				if (file_data->pmd)
				{
					file_data->vmd->model_name = file_data->pmd->header.name;
				}
				else if (file_data->pmx)
				{
					converter.Utf16ToCp932(file_data->pmx->model_name.c_str(), file_data->pmx->model_name.length(), &file_data->vmd->model_name);
				}
			}
		}

		for (int i = 0; i < pmd_num; ++i)
		{
			auto &file_data = m_data[i];
			const int bone_num = ExpGetPmdBoneNum(i);
			auto a = 0;
			for (int k = 0; k < bone_num; ++k)
			{
				const char* bone_name = ExpGetPmdBoneName(i, k);
				if (file_data->bone_name_map.find(k) == file_data->bone_name_map.end()) {
					// not found
					continue;
				}

				if (file_data->bone_name_map[k] != bone_name) {
					// ?
					continue;
				}

				// export mode
				auto is_physics = file_data->physics_bone_map.find(k) != file_data->physics_bone_map.end();
				auto is_ik = file_data->ik_bone_map.find(k) != file_data->ik_bone_map.end();
				auto is_fuyo = file_data->fuyo_bone_map.find(k) != file_data->fuyo_bone_map.end();

				if (m_export_mode == 0)
				{
					// physics + ik + fuyo
					if(!is_physics && !is_ik && !is_fuyo){
						continue;
					}
				}
				else if (m_export_mode == 1)
				{
					// physics only
					if (!is_physics) {
						continue;
					}
				}
				else
				{
					// all (buggy)
				}

				push_bone_frame(file_data, currentframe, i, k);
			}


			if (currentframe == BridgeParameter::instance().start_frame)
			{
				vmd::VmdIkFrame ik_frame;
				ik_frame.frame = currentframe;
				ik_frame.display = true;
				for (std::map<int, int>::iterator it = file_data->ik_frame_bone_map.begin();
					it != file_data->ik_frame_bone_map.end();
					++it)
				{
					if (file_data->bone_name_map.find(it->first) != file_data->bone_name_map.end())
					{
						vmd::VmdIkEnable ik_enable;
						ik_enable.ik_name = file_data->bone_name_map[it->first];
						ik_enable.enable = false;
						ik_frame.ik_enable.push_back(ik_enable);
					}
				}
				file_data->vmd->ik_frames.push_back(ik_frame);
			}
		}

		return true;
	}

	bool write()
	{
		BridgeParameter::instance().is_exporting_without_mesh = true;
		const int pmd_num = ExpGetPmdNum();
		oguna::EncodingConverter converter;

		for (int i = 0; i < pmd_num; ++i)
		{
			const char* filename = ExpGetPmdFilename(i);
			auto &file_data = m_data[i];
			if (file_data->vmd)
			{
				std::string dst;
				converter.Cp932ToUtf8(filename, strnlen(filename, 4096), &dst);
				const umstring umstr = umbase::UMStringUtil::utf8_to_utf16(dst);
				umstring filename = umbase::UMPath::get_file_name(umstr);
				const umstring extension = umbase::UMStringUtil::utf8_to_utf16(".vmd");
				filename.replace(filename.size() - 4, 4, extension);

				std::wstringstream ss;
				ss << m_output_path << i << L"_" << umbase::UMStringUtil::utf16_to_wstring(filename);

				auto path = ss.str();
				file_data->vmd->SaveToFile(path);

				const int bone_num = ExpGetPmdBoneNum(i);
				auto count = bone_num * (m_lastFrame + 1);

				file_data->SaveMFB(path + L".mfb");
			}
		}

		return true;
	}

private:
	void init_file_data(const std::unique_ptr<FileDataForVMD> &data)
	{
		if (data->pmd) {
			const std::vector<pmd::PmdBone>& bones = data->pmd->bones;
			const std::vector<pmd::PmdRigidBody>& rigids = data->pmd->rigid_bodies;
			std::map<int, int> bone_to_rigid_map;
			for (int i = 0, isize = static_cast<int>(bones.size()); i < isize; ++i)
			{
				const pmd::PmdBone& bone = bones[i];
				const int parent_bone = bone.parent_bone_index;
				data->parent_index_map[i] = parent_bone;
				data->bone_name_map[i] = bone.name;
				if (bone.bone_type == pmd::BoneType::IkEffectable)
				{
					data->ik_bone_map[i] = 1;
				}
				if (bone.bone_type == pmd::BoneType::IkEffector)
				{
					data->ik_frame_bone_map[i] = 1;
				}
			}

			for (int i = 0, isize = static_cast<int>(rigids.size()); i < isize; ++i)
			{
				const pmd::PmdRigidBody& rigid = rigids[i];
				const uint16_t bone_index = rigid.related_bone_index;
				bone_to_rigid_map[bone_index] = i;
				if (rigid.rigid_type != pmd::RigidBodyType::BoneConnected)
				{
					if (data->bone_name_map.find(bone_index) != data->bone_name_map.end())
					{
						if (rigid.rigid_type == pmd::RigidBodyType::ConnectedPhysics)
						{
							data->physics_bone_map[bone_index] = 2;
						}
						else
						{
							data->physics_bone_map[bone_index] = 1;
						}
					}
				}
			}
			// expect for rigid_type == BoneConnected
			{
				std::vector<int> parent_physics_bone_list;
				for (int i = 0, isize = static_cast<int>(rigids.size()); i < isize; ++i)
				{
					const pmd::PmdRigidBody& rigid = rigids[i];
					const int target_bone = rigid.related_bone_index;
					const int parent_bone = data->parent_index_map[target_bone];
					if (data->physics_bone_map.find(target_bone) != data->physics_bone_map.end())
					{
						if (bone_to_rigid_map.find(parent_bone) != bone_to_rigid_map.end())
						{
							parent_physics_bone_list.push_back(parent_bone);
						}
					}
				}
				for (int i = 0, size = parent_physics_bone_list.size(); i < size; ++i) {
					const int parent_bone = parent_physics_bone_list[i];
					const pmd::PmdRigidBody& parent_rigid = rigids[bone_to_rigid_map[parent_bone]];
					if (parent_rigid.rigid_type == pmd::RigidBodyType::BoneConnected) {
						data->physics_bone_map[parent_bone] = 0;
					}
				}
			}
		}
		else if (data->pmx)
		{
			oguna::EncodingConverter converter;
			const int bone_count = data->pmx->bones.size();
			for (int i = 0; i < bone_count; ++i)
			{
				const pmx::PmxBone& bone = data->pmx->bones[i];
				const int parent_bone = bone.parent_index;
				data->parent_index_map[i] = parent_bone;
				converter.Utf16ToCp932(bone.bone_name.c_str(), bone.bone_name.length(), &data->bone_name_map[i]);
				for (int k = 0; k < bone.ik_link_count; ++k)
				{
					const pmx::PmxIkLink& link = bone.ik_links[k];
					data->ik_bone_map[link.link_target] = 1;
					data->ik_frame_bone_map[i] = 1;
				}
				if ((bone.bone_flag & 0x0100) || (bone.bone_flag & 0x0200)) {
					data->fuyo_target_map[bone.grant_parent_index] = 1;
					data->fuyo_bone_map[i] = 1;
				}
			}

			const int rigid_count = data->pmx->rigid_bodies.size();
			std::map<int, int> bone_to_rigid_map;
			for (int i = 0; i < rigid_count; ++i)
			{
				const pmx::PmxRigidBody& rigid = data->pmx->rigid_bodies[i];
				bone_to_rigid_map[rigid.target_bone] = i;
				if (rigid.physics_calc_type != 0)
				{
					uint16_t bone_index = rigid.target_bone;
					if (data->bone_name_map.find(bone_index) != data->bone_name_map.end())
					{
						if (rigid.physics_calc_type == 2) {
							data->physics_bone_map[bone_index] = 2;
						}
						else
						{
							data->physics_bone_map[bone_index] = 1;
						}
					}
				}
			}
			// expect for physics_calc_type == 0
			{
				std::vector<int> parent_physics_bone_list;
				for (int i = 0; i < rigid_count; ++i)
				{
					const pmx::PmxRigidBody& rigid = data->pmx->rigid_bodies[i];
					const int target_bone = rigid.target_bone;
					const int parent_bone = data->parent_index_map[target_bone];
					if (data->physics_bone_map.find(target_bone) != data->physics_bone_map.end())
					{
						if (bone_to_rigid_map.find(parent_bone) != bone_to_rigid_map.end())
						{
							parent_physics_bone_list.push_back(parent_bone);
						}
					}
				}
				for (int i = 0, size = parent_physics_bone_list.size(); i < size; ++i) {
					const int parent_bone = parent_physics_bone_list[i];
					const pmx::PmxRigidBody& parent_rigid = data->pmx->rigid_bodies[bone_to_rigid_map[parent_bone]];
					if (parent_rigid.physics_calc_type == 0) {
						data->physics_bone_map[parent_bone] = 0;
					}
				}
			}
		}
	}

	void push_bone_frame(const std::unique_ptr<FileDataForVMD> &file_data, int currentframe, int model_index, int bone_index)
	{
		// get initial world position
		UMVec3f initial_trans;
		if (file_data->pmd)
		{
			pmd::PmdBone& bone = file_data->pmd->bones[bone_index];
			initial_trans[0] = bone.bone_head_pos[0];
			initial_trans[1] = bone.bone_head_pos[1];
			initial_trans[2] = bone.bone_head_pos[2];
		}
		else if (file_data->pmx)
		{
			pmx::PmxBone& bone = file_data->pmx->bones[bone_index];
			initial_trans[0] = bone.position[0];
			initial_trans[1] = bone.position[1];
			initial_trans[2] = bone.position[2];
		}

		auto m = ExpGetPmdBoneWorldMat(model_index, bone_index);

		file_data->bone_matrices.push_back(m);

		UMMat44d world = to_ummat(m);
		UMMat44d local = world;
		UMVec3d parent_offset;
		int parent_index = file_data->parent_index_map[bone_index];
		if (parent_index != 0xFFFF && file_data->parent_index_map.find(parent_index) != file_data->parent_index_map.end()) {
			UMMat44d parent_world = to_ummat(ExpGetPmdBoneWorldMat(model_index, parent_index));
			local = world * parent_world.inverted();
		}
		local[3][0] -= initial_trans[0];
		local[3][1] -= initial_trans[1];
		local[3][2] -= initial_trans[2];

		vmd::VmdBoneFrame bone_frame;
		bone_frame.frame = currentframe;
		bone_frame.name = ExpGetPmdBoneName(model_index, bone_index);
		bone_frame.position[0] = static_cast<float>(local[3][0]);
		bone_frame.position[1] = static_cast<float>(local[3][1]);
		bone_frame.position[2] = static_cast<float>(local[3][2]);
		local[3][0] = local[3][1] = local[3][2] = 0.0;
		UMVec4d quat = extractQuat(local);
		bone_frame.orientation[0] = static_cast<float>(quat[0]);
		bone_frame.orientation[1] = static_cast<float>(quat[1]);
		bone_frame.orientation[2] = static_cast<float>(quat[2]);
		bone_frame.orientation[3] = static_cast<float>(quat[3]);
		for (int n = 0; n < 4; ++n) {
			for (int m = 0; m < 4; ++m) {
				bone_frame.interpolation[n][0][m] = 20;
				bone_frame.interpolation[n][1][m] = 20;
			}
			for (int m = 0; m < 4; ++m) {
				bone_frame.interpolation[n][2][m] = 107;
				bone_frame.interpolation[n][3][m] = 107;
			}
		}

		/*
		// constraints
		if (file_data->pmd)
		{
			pmd::PmdBone& bone = file_data->pmd->bones[bone_index];
			if (bone.bone_type == pmd::BoneType::Rotation)
			{
				bone_frame.position[0] = 0.0f;
				bone_frame.position[1] = 0.0f;
				bone_frame.position[2] = 0.0f;
			}
		}
		else if (file_data->pmx)
		{
			pmx::PmxBone& bone = file_data->pmx->bones[bone_index];
			if (!(bone.bone_flag & 0x0004))
			{
				bone_frame.position[0] = 0.0f;
				bone_frame.position[1] = 0.0f;
				bone_frame.position[2] = 0.0f;
			}
			if (!(bone.bone_flag & 0x0002))
			{
				bone_frame.orientation[0] = 0.0f;
				bone_frame.orientation[1] = 0.0f;
				bone_frame.orientation[2] = 0.0f;
				bone_frame.orientation[3] = 1.0f;
			}
			if (file_data->physics_bone_map.find(bone_index) != file_data->physics_bone_map.end()) {
				if (file_data->physics_bone_map[bone_index] == 2)
				{
					bone_frame.position[0] = 0.0f;
					bone_frame.position[1] = 0.0f;
					bone_frame.position[2] = 0.0f;
				}
			}
			// expect for fuyo
			if (file_data->fuyo_target_map.find(bone_index) != file_data->fuyo_target_map.end()) {
				bone_frame.position[0] = 0.0f;
				bone_frame.position[1] = 0.0f;
				bone_frame.position[2] = 0.0f;
				bone_frame.orientation[0] = 0.0f;
				bone_frame.orientation[1] = 0.0f;
				bone_frame.orientation[2] = 0.0f;
				bone_frame.orientation[3] = 1.0f;
			}

			// expect for rigid_type == BoneConnected
			const int parent_bone = file_data->parent_index_map[bone_index];
			if (file_data->physics_bone_map.find(parent_bone) != file_data->physics_bone_map.end()) {
				if (file_data->physics_bone_map[parent_bone] == 0) {
					bone_frame.position[0] = 0.0f;
					bone_frame.position[1] = 0.0f;
					bone_frame.position[2] = 0.0f;
				}
			}
		}
		*/

		file_data->vmd->bone_frames.push_back(bone_frame);
	}

	VMDArchive() {}
};


// ---------------------------------------------------------------------------
PYBIND11_PLUGIN(mmdbridge_vmd) {
	py::module m("mmdbridge_vmd");
	m.def("start_vmd_export", [](const std::string &path, int mode) { 
		VMDArchive::instance().start_vmd_export(path, mode); 
	});
	m.def("end_vmd_export", []() { 
		VMDArchive::instance().write();
	});
	m.def("execute_vmd_export", [](int currentframe) { 
		VMDArchive::instance().execute_vmd_export(currentframe); 
	});
	return m.ptr();
}

#endif //WITH_VMD


// ---------------------------------------------------------------------------
#ifdef WITH_VMD
void InitVMD()
{
	PyImport_AppendInittab("mmdbridge_vmd", PyInit_mmdbridge_vmd);
}
void DisposeVMD()
{
}
#else
void InitVMD() {}
void DisposeVMD() {}
#endif //WITH_VMD
