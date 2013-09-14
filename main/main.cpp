#include "checkpmvspoint.hpp"

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		std::cout << "Usage: backimagecoor <pmvs工作目录>\n"
			<< "eg: backimagecoor E:\\3D_reconstruction\\result\\pmvs\n";
		return -1;
	}

	cmpvsp Mycmpvsp(argv[1]);
	int inputitem;
	int pi,ii;
	char c;
	while(1)
	{
		inputitem = 0;
		pi = 0;
		ii = 0;
		std::cout << "\n1:查询相机P矩阵.\n"
				  <<  "2:查询点世界坐标.\n"
			      << "3:查询点point在所有图片的坐标.\n"
				  << "4:输出点point在所有图片缩略图.\n"
			      << "0:退出.\n";
		std::cin.clear();
		std::cin >> c;
		std::cin.sync();
		if (isdigit(c))
		{
			inputitem = atoi(&c);
			if (inputitem == 1)
			{
				std::cout << "\n输入相机P矩阵文件号eg: 要查00000003.txt的,输入3\n";
				std::cin >> pi;
				std::cin.sync();
				Mycmpvsp.GetCameraParameter(pi);
			}
			else if (inputitem == 2)
			{
				std::cout << "\n输入点名称,(序号从0开始)\n";
				std::cin >> pi;
				std::cin.sync();
				Mycmpvsp.GetPointWorldCoord(pi);
			}
			else if (inputitem == 3)
			{
				std::cout << "\n输入点名称,(序号从0开始)\n";
				if (std::cin >> pi)
				{
					Mycmpvsp.GetPointImageCoord(pi);
				}
				std::cin.sync();
			}
			else if (inputitem == 4)
			{
				std::cout << "\n输入点名称,(序号从0开始)\n";
				if (std::cin >> pi)
				{
					Mycmpvsp.CutOutPointVisibleImage(pi);
				}
				std::cin.sync();
			}
			else if(inputitem == 0)
			{
				exit(0);							   
			}
		}
	}
	return 0;
}