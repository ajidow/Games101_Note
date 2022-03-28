#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
		-eye_pos[2], 0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the model matrix for rotating the triangle around the Z axis.
	// Then return it.

	float rotation_angle_arc = rotation_angle / 180 * MY_PI;
	model << cos(rotation_angle_arc), -sin(rotation_angle_arc), 0, 0, sin(rotation_angle_arc), cos(rotation_angle_arc), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

	return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
									  float zNear, float zFar)
{
	// Students will implement this function

	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the projection matrix for the given parameters.
	// Then return it.

	float t = tan(eye_fov / 2) * abs(zNear);
	float r = aspect_ratio * t;
	projection << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -zNear * zFar, 0, 0, 1, 0;
	Eigen::Matrix4f orthographic = Eigen::Matrix4f::Identity();
	orthographic << 1 / r, 0, 0, 0, 0, 1 / t, 0, 0, 0, 0, 2 / (zNear - zFar), -(zNear + zFar) / 2, 0, 0, 0, 1;
	projection = orthographic * projection;

	return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
	angle = angle / 180.0f * MY_PI;

	Eigen::Matrix3f k;
	k << 0.0f, -axis[2], axis[1],
		axis[2], 0.0f, -axis[0],
		-axis[1], axis[0], 0;

	Eigen::Matrix3f E = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f rotation;
	rotation << E * std::cos(angle) + (1 - std::cos(angle)) * axis * axis.transpose() + std::sin(angle) * k;
	Eigen::Matrix4f model;
	// for(int i = 0; i < 3; ++i)
	// {
	// 	for(int j = 0; j < 3; ++j)
	// 	{
	// 		model(i,j)=rotation(i,j);
	// 		model(3,j) = 0;
	// 	}
	// 	model(i,3)=0;
	// }
	// model(3,3)=1;

	model << rotation(0, 0), rotation(0, 1), rotation(0, 2), 0,
		rotation(1.0), rotation(1, 1), rotation(1, 2), 0,
		rotation(2, 0), rotation(2, 1), rotation(2, 2), 0,
		0, 0, 0, 1;

	return model;
}

int main(int argc, const char **argv)
{
	float anglex = 0, angley = 0, anglez = 0;
	bool command_line = false;
	std::string filename = "output.png";

	if (argc >= 3)
	{
		command_line = true;
		anglez = std::stof(argv[2]); // -r by default
		if (argc == 4)
		{
			filename = std::string(argv[3]);
		}
		else
			return 0;
	}

	rst::rasterizer r(700, 700);

	Eigen::Vector3f eye_pos = {0, 0, 5};

	std::vector<Eigen::Vector3f> pos{{1, 0, -1}, {0, 1, -1}, {-1, 0, -1}};

	std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);

	int key = 0;
	int frame_count = 0;

	if (command_line)
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(anglez));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);

		cv::imwrite(filename, image);

		return 0;
	}

	Vector3f axis;

	char pre;

	while (key != 27) // 27 is esc...
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		// r.set_model(get_model_matrix(angle));
		// r.set_model(get_rotation(axis, angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);
		key = cv::waitKey(10);

		std::cout << "frame count: " << frame_count++ << '\n';

		if (key == 'x')
		{
			axis << 1.0f, 0.0f, 0.0f;
			anglex += 10;
			r.set_model(get_rotation(axis, anglex));
		}
		else if (key == 'y')
		{

			axis << 0.0f, 1.0f, 0.0f;
			angley += 10;
			r.set_model(get_rotation(axis, angley));
		}
		else if (key == 'z')
		{
			axis << 0.0f, 0.0f, 1.0f;
			anglez += 10;
			r.set_model(get_rotation(axis, anglez));
		}

		// if (angle >= 360.0f || angle <= -360.0f)
		// 	angle = 0.0f;
		pre = key;
	}

	return 0;
}