#include "Mesh.h"
#include "../Core/SOUtils.h"


Mesh::Mesh()
{

}

Mesh::Mesh(const std::vector<Mesh*>& meshes)
{
	int numVertices = 0;
	int numFaces = 0;
	int numFaceColors = 0;
	for (int i = 0; i < meshes.size(); i++)
	{
		//vertex positions
		numVertices += meshes[i]->vertices().size() / 3;
		//face indices
		numFaces += meshes[i]->faces().size();
		//face color
		numFaceColors += meshes[i]->faceColors().size();
	}

	m_x = Eigen::VectorXd(numVertices * 3);
	m_f.resize(numFaces);
	m_faceColors = Eigen::VectorXd(numFaceColors);

	//combine meshes
	int start_x_index = 0;
	int faceInex = 0;
	int start_faceColor_index = 0;
	for (int i = 0; i < meshes.size(); i++)
	{
		//face indices
		for (int j = 0; j < meshes[i]->faces().size(); j++)
		{
			m_f[faceInex] = meshes[i]->faces()[j] + start_x_index / 3;
			faceInex++;
		}
		//vertex positions
		int numVerticesDoFs_currentMesh = meshes[i]->vertices().size();
		m_x.segment(start_x_index, numVerticesDoFs_currentMesh) = meshes[i]->vertices();
		start_x_index += numVerticesDoFs_currentMesh;
		//face color
		int numFaceColors_currentMesh = meshes[i]->faceColors().size();
		m_faceColors.segment(start_faceColor_index, numFaceColors_currentMesh) = meshes[i]->faceColors();
		start_faceColor_index += numFaceColors_currentMesh;
	}
}

void Mesh::translate(const Eigen::Vector3d& translation)
{
	for (int i = 0; i < numVertices(); i++)
	{
		m_x.segment<3>(3 * i) += translation;
	}
	m_dirty = true;
}

void Mesh::translate(double dx, double dy, double dz)
{

	for (int i = 0; i < numVertices(); i++)
	{
		m_x[3 * i] += dx;
		m_x[3 * i + 1] += dy;
		m_x[3 * i + 2] += dz;
	}
	m_dirty = true;
}

void Mesh::rotate(double rotation[3], int rotationOrder[3])
{
	//rotate
	{
		//rotation matrices
		Eigen::Matrix3d yaw; yaw.setZero();
		Eigen::Matrix3d pitch; pitch.setZero();
		Eigen::Matrix3d roll; roll.setZero();

		double rot_x = rotation[0] / 180.0 * M_PI;
		double rot_y = rotation[1] / 180.0 * M_PI;
		double rot_z = rotation[2] / 180.0 * M_PI;
		//z rotation
		yaw(0, 0) = cos(rot_z);	yaw(0, 1) = -sin(rot_z);
		yaw(1, 0) = sin(rot_z);	yaw(1, 1) = cos(rot_z);
		yaw(2, 2) = 1.0;
		//y rotation
		pitch(0, 0) = cos(rot_y); pitch(0, 2) = sin(rot_y);
		pitch(1, 1) = 1.0;
		pitch(2, 0) = -sin(rot_y); pitch(2, 2) = cos(rot_y);
		//x rotation
		roll(0, 0) = 1.0;
		roll(1, 1) = cos(rot_x); roll(1, 2) = -sin(rot_x);
		roll(2, 1) = sin(rot_x); roll(2, 2) = cos(rot_x);

		Eigen::Matrix3d rotation[3] = { roll, pitch, yaw };

		auto rotateWithMatrix = [](const Eigen::Matrix3d& rotationMatrix, Eigen::VectorXd& x)
		{
			for (int i = 0; i < x.size() / 3; i++)
			{
				x.segment<3>(i * 3) = rotationMatrix * x.segment<3>(i * 3);
			}
			
		};

		for (int order_i = 0; order_i < 3; order_i++)
		{
			for (int i = 0; i < 3; i++)
			{
				if (rotationOrder[i] == order_i)
				{
					rotateWithMatrix(rotation[i], m_x);
					break;
				}
			}
		}

		//vertices = yaw * pitch * roll * vertices;
	}
}
void Mesh::rotate(const Eigen::Quaternion<double>& q)
{
	for (int i = 0; i < m_x.size() / 3; i++)
	{
		m_x.segment<3>(i * 3) = q * m_x.segment<3>(i * 3);
	}
}


void Mesh::scale(double scale[3])
{
	//scale the mesh w.r.t its center point
	V3D centerPos(0, 0, 0);
	for (int i = 0; i < m_x.size() / 3; i++)
	{
		centerPos += m_x.segment<3>(3 * i);
	}
	centerPos /= (double)(m_x.size() / 3);

	for (int i = 0; i < m_x.size() / 3; i++)
	{
		V3D translated = (m_x.segment<3>(3 * i) - centerPos);
		for (int d = 0; d < 3; d++)
			translated[d] *= scale[d];
		m_x.segment<3>(3 * i) = translated + centerPos;
	}
}

void Mesh::setFaceColor(const V3D& vc)
{
	for (int i = 0; i < m_faceColors.size() / 3; i++)
		soutil::set3D(i, vc, m_faceColors);
}

void Mesh::setFaceColor(const V3D& vc, int i)
{
	soutil::set3D(i, vc, m_faceColors);
}

void Mesh::clear()
{
	m_x = Eigen::VectorXd();
	m_f.clear();
	m_faceColors = Eigen::VectorXd();
	m_dirty = true;
}
