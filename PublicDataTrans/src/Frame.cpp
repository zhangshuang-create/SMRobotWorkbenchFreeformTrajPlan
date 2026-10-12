#include <Frame2/Frame.h>

namespace PublicData
{

	Frame::Frame()
	{
	}

	Frame::~Frame()
	{
	}

	imodel_spointer Frame::get_frame(iscene_spointer sp_scene)
	{
		//	4.3. Color Line Element
		imodel_spointer sp_colorline_element = sp_scene->get_new_model(MODELTYPE::MODELTYPE_COLORLINE_ELEMENT);
		MeshData colorline_data_element;
		colorline_data_element.m_vertices = {
			0.0f, 0.0f, 0.0f,				1.0f, 0.0f, 0.0f, 1.0f,
			10000.0f, 0.0f, 0.0f,			1.0f, 0.0f, 0.0f, 1.0f,
			0.0f, 0.0f, 0.0f,			    0.0f, 1.0f, 0.0f, 1.0f,
			0.0f, 10000.0f, 0.0f,		    0.0f, 1.0f, 0.0f, 1.0f,
			0.0f, 0.0f, 0.0f,			    0.0f, 0.0f, 1.0f, 1.0f,
			0.0f, 0.0f, 10000.0f,			0.0f, 0.0f, 1.0f, 1.0f
		};

		colorline_data_element.m_indices = {
			0, 1, 2, 3, 4, 5,
		};

		sp_colorline_element->import(colorline_data_element);
		return sp_colorline_element;
	}

}