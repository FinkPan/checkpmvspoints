#include "checkpmvspoint.hpp"

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		std::cout << "Usage: backimagecoor <pmvs����Ŀ¼>\n"
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
		std::cout << "\n1:��ѯ���P����.\n"
				  <<  "2:��ѯ����������.\n"
			      << "3:��ѯ��point������ͼƬ������.\n"
				  << "4:�����point������ͼƬ����ͼ.\n"
			      << "0:�˳�.\n";
		std::cin.clear();
		std::cin >> c;
		std::cin.sync();
		if (isdigit(c))
		{
			inputitem = atoi(&c);
			if (inputitem == 1)
			{
				std::cout << "\n�������P�����ļ���eg: Ҫ��00000003.txt��,����3\n";
				std::cin >> pi;
				std::cin.sync();
				Mycmpvsp.GetCameraParameter(pi);
			}
			else if (inputitem == 2)
			{
				std::cout << "\n���������,(��Ŵ�0��ʼ)\n";
				std::cin >> pi;
				std::cin.sync();
				Mycmpvsp.GetPointWorldCoord(pi);
			}
			else if (inputitem == 3)
			{
				std::cout << "\n���������,(��Ŵ�0��ʼ)\n";
				if (std::cin >> pi)
				{
					Mycmpvsp.GetPointImageCoord(pi);
				}
				std::cin.sync();
			}
			else if (inputitem == 4)
			{
				std::cout << "\n���������,(��Ŵ�0��ʼ)\n";
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