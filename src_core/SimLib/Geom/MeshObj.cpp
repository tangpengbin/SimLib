#include "MeshObj.h"

#include <string>
#include <cerrno>
#include <sstream>
#include <iostream>
#include <iomanip>

#include "../Core/File.h"
#include <fstream>

bool get_line(const std::string &s, int *idx, StringSubsequence *sequence)
{
	int length = (int)s.size();
	if (length == 0) return false;
	if (*idx >= length) return false;
	else
	{
		for (int i = *idx; i < length; i++)
		{
			if (s[i] == '\n')
			{
				sequence->set(&s, *idx, i + 1);
				(*idx) = i + 1;
				return true;
			}
		}

		sequence->set(&s, *idx, length);
		(*idx) = length;
		return true;
	}
}

template<typename T>
std::vector<T> readNumberList(std::istringstream &iss)
{
	std::vector<T> numbers;
	T num = 0;
	while (iss >> num || !iss.eof())
	{
		if (iss.fail())
		{
			iss.clear();
			std::string dummy;
			iss >> dummy;
			continue;
		}
		else
		{
			numbers.push_back(num);
		}
	}
	return numbers;
}

bool MeshObj::readVertex(const StringSubsequence &subsequence)
{
	std::istringstream iss( subsequence.toString() );

	std::vector<double> numbers = readNumberList<double>(iss);

	if (numbers.size() != 3 && numbers.size() != 4)
	{
		return false;
	}

	if (numbers.size() == 3)
	{
		numbers.push_back(1.0);
	}
	m_vertices.emplace_back(numbers[0], numbers[1], numbers[2], numbers[3]);

	return true;
}
bool MeshObj::readNormal(const StringSubsequence &subsequence)
{
	StringSubsequence sub = subsequence.trim();
	std::istringstream iss(sub.toString());

	std::vector<double> numbers = readNumberList<double>(iss);

	if (numbers.size() != 3)
	{
		return false;
	}

	m_normals.emplace_back(numbers[0], numbers[1], numbers[2]);

	return true;
}
bool MeshObj::readLineElement(const StringSubsequence &subsequence)
{
	std::istringstream iss(subsequence.toString());

	std::vector<int> numbers = readNumberList<int>(iss);

	if (numbers.size() < 2)
	{
		return false;
	}

	for (int &i : numbers)  i -= 1;

	m_lines.emplace_back(std::move(numbers));

	return true;
}
bool MeshObj::readFaceElement(const StringSubsequence &subsequence)
{
	StringSubsequence sub = subsequence.trim();
	std::vector<StringSubsequence> sequences = sub.split(' ');
	std::vector<int> vtxIdcs, textureIdcs, normalIdcs;
	for (int i = 0; i < sequences.size(); i++)
	{
		std::vector<StringSubsequence> s = sequences[i].split('/');
		if (s.size() >= 1)
		{
			vtxIdcs.push_back(s[0].toInt() - 1);
		}
		else
		{
			return false;
		}
		
		if (s.size() >= 2)
		{
			if (s[1].length() > 0)
			{
				textureIdcs.push_back(s[1].toInt() - 1);
			} else {
				textureIdcs.push_back(-1);
			}
		}
		else
		{
			textureIdcs.push_back(-1);
		}


		if (s.size() >= 3)
		{
			if (s[2].length() > 0)
			{
				normalIdcs.push_back(s[2].toInt() - 1);
			}
			else
			{
				normalIdcs.push_back(-1);
			}
		}
		else
		{
			normalIdcs.push_back(-1);
		}

		if (s.size() > 3)
		{
			return false;
		}
	}

	m_faces.emplace_back(std::move(vtxIdcs), std::move(textureIdcs), std::move(normalIdcs));

	return true;
}

MeshObj::MeshObj()
{

}
MeshObj::~MeshObj()
{

}

void MeshObj::clear()
{
	m_vertices.clear();
	m_lines.clear();
	m_faces.clear();
}
void MeshObj::addVertex(MeshObjVertex &&vertex)
{
	m_vertices.emplace_back(std::move(vertex));
}
void MeshObj::addLine(MeshObjLine &&line)
{
	m_lines.emplace_back(std::move(line));
}
void MeshObj::addFace(MeshObjFace&& face)
{
	m_faces.emplace_back(std::move(face));
}
bool MeshObj::load(const std::string &fileContents, std::string &errorMessage)
{
	clear();

	errorMessage = "";

	StringSubsequence sequence;
	int idx = 0;
	int lineIdx = 0;
	while (get_line(fileContents, &idx, &sequence))
	{
		lineIdx += 1;

		if (sequence.length() < 2)
		{
			continue;
		}
		char firstC = sequence[0];
		char secondC = sequence[1];
		switch (firstC)
		{
		case '#':
			break;
		case 'v':
			switch (secondC)
			{
			case ' ':
				sequence.set(&fileContents, sequence.startIndex() + 2, sequence.endIndex());
				if (!readVertex(sequence))
				{
					errorMessage = std::string("could not read vertex on line ") + std::to_string(lineIdx);
					return false;
				}
				break;
			case 'p':
				std::cout << '\'' << firstC << secondC << "' sequence ignored when loading obj " << std::endl;
				break; // not implemented currently
			case 't':
				std::cout << '\'' << firstC << secondC << "' sequence ignored when loading obj " << std::endl;
				break; // not implemented currently
			case 'n':
				sequence.set(&fileContents, sequence.startIndex() + 2, sequence.endIndex()); //adding +3 could result in negative length sequence, instead we trim inside readNormal
				if (!readNormal(sequence))
				{
					errorMessage = std::string("could not read normal on line ") + std::to_string(lineIdx);
					return false;
				}
				break;
			default:
				std::cout << '\'' << firstC << secondC << "' sequence ignored when loading obj " << std::endl;
				break; // not implemented currently
			}
			break;
		case 'l':
			if (!isspace(secondC))
			{
				errorMessage = std::string("unexpected character on line ") + std::to_string(lineIdx);
				return false;
			}
			sequence.set(&fileContents, sequence.startIndex() + 2, sequence.endIndex());
			if (!readLineElement(sequence))
			{
				errorMessage = std::string("could not read line element on line ") + std::to_string(lineIdx);
				return false;
			}
			break;
		case 'f':
			if (!isspace(secondC))
			{
				errorMessage = std::string("unexpected character on line ") + std::to_string(lineIdx);
				return false;
			}
			sequence.set(&fileContents, sequence.startIndex() + 2, sequence.endIndex());
			if (!readFaceElement(sequence))
			{
				errorMessage = std::string("could not read line element on line ") + std::to_string(lineIdx);
				return false;
			}
			break;
		default:
			std::cout << '\'' << firstC << "' sequence ignored when loading obj " << std::endl;
			break;
		}

	}
	return true;
}
bool MeshObj::loadFromFile(const std::string &filepath, std::string &errorMessage)
{
	std::string contents;
	bool success = get_file_contents(filepath.c_str(), contents, errorMessage);
	if (!success) return false;
	return load(contents, errorMessage);
}
bool MeshObj::writeToFile(const std::string &filepath, std::string &errorMessage)
{
	std::ofstream out(filepath);
	out << std::setprecision(std::numeric_limits<double>::max_digits10);
	//if (m_faces.size() != 0) throw std::logic_error("not implemented");

	for (int i = 0; i < m_vertices.size(); i++)
	{
		out << "v " << m_vertices[i].x() << ' ' << m_vertices[i].y() << ' ' << m_vertices[i].z();
		if (m_vertices[i].w() != 1.0) out << ' ' << m_vertices[i].w();
		out << '\n';
	}
	for (int i = 0; i < m_lines.size(); i++)
	{
		out << 'l';
		for (int k = 0; k < m_lines[i].getVertices().size(); k++)
		{
			int objIdx = m_lines[i].getVertices()[k] + 1;
			out << ' ' << objIdx;
		}
		out << '\n';
	}
	for (int i = 0; i < m_faces.size(); i++)//we only output faces
	{
		out << 'f';
		for (int k = 0; k < m_faces[i].getVertices().size(); k++)
		{
			int objIdx = m_faces[i].getVertices()[k] + 1;
			out << ' ' << objIdx;
		}
		out << '\n';
	}

	return true;
}

void MeshObj::centeringVertices()
{
	Eigen::Vector3d center(0, 0, 0);
	for (int i = 0; i < m_vertices.size(); i++)
	{
		center += Eigen::Vector3d(m_vertices[i].x(), m_vertices[i].y(), m_vertices[i].z());
	}
	center /= (double)m_vertices.size();

	for (int i = 0; i < m_vertices.size(); i++)
	{
		Eigen::Vector3d afterCentering = Eigen::Vector3d(m_vertices[i].x(), m_vertices[i].y(), m_vertices[i].z()) - center;

		m_vertices[i] = MeshObjVertex(afterCentering[0], afterCentering[1], afterCentering[2]);
	}
}

Eigen::MatrixXd MeshObj::getVerticesAsEigenCols() const
{
	Eigen::MatrixXd vertices(3, m_vertices.size());
	for (int i = 0; i < m_vertices.size(); i++)
	{
		vertices.col(i) = Eigen::Vector3d(m_vertices[i].x(), m_vertices[i].y(), m_vertices[i].z());
	}
	return vertices;
}
Eigen::MatrixXi MeshObj::getFacesAsEigenCols() const
{
	Eigen::MatrixXi faces(3, m_faces.size());
	for (int i = 0; i < m_faces.size(); i++)
	{
		if (m_faces[i].getVertices().size() != 3) std::cout << " warning unsupported number of vertices in face" << std::endl;
		faces.col(i) = Eigen::Vector3i(m_faces[i].getVertices()[0], m_faces[i].getVertices()[1], m_faces[i].getVertices()[2]);
	}
	return faces;
}