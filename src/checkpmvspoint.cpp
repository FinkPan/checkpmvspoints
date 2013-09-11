#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <iterator>
#include <Eigen/Dense>
#include <cctype>	  //isdigit()	
#include <cstdlib>    //atoi()

#include <atlimage.h>
#include <Gdiplusimaging.h>

//#include <afxwin.h>

using namespace Eigen;

typedef int imageID;
typedef int pointID;

//ͼƬ��Ϣ(�Ե����)
struct pointorder_coord
{
    int imageID;
    double x;     //��x��������
    double y;     //��y��������
};

//ͼƬ��Ϣ(��ͼƬ������)
struct imageorder_coord
{
    int pointID;
    double x;     //��x��������
    double y;     //��y��������
};

//��ȡ���P����,�������P����·��,���map<ͼƬ��,P����>;
void ReadCameraParameter(std::string filepath,std::map<int,MatrixXd> &map_camera_parameter_Matrix)
{
    std::string str_CONTOUR;
    std::string filename = filepath;
    std::string file_index;
    file_index.resize(8);  //pmvs�ļ��涨Ϊ8Ϊ����
    std::ifstream inputfile;
    MatrixXd P(3,4);

    int image_index = 0; //ͼƬ���
    while (1)
    {
        sprintf(&file_index[0], "%08d",image_index); //��ʽ���ļ���
        filename =  filename + file_index + ".txt";
        inputfile.open(filename,std::ios_base::in);
        if (!inputfile) //��ȡ�ļ�ʧ��
        {
            ++image_index;
            std::cout << "�޷����ļ�: " << filename << std::endl;
            break;
        }
        inputfile >> str_CONTOUR;
        if (str_CONTOUR != "CONTOUR") //����ļ���ʽ
        {
            std::cout << "CONTOUR�ļ���ʽ����!" << std::endl;
            continue;
        }

        //д��P����
        inputfile >> P(0,0) >> P(0,1) >> P(0,2) >> P(0,3)
                  >> P(1,0) >> P(1,1) >> P(1,2) >> P(1,3)
                  >> P(2,0) >> P(2,1) >> P(2,2) >> P(2,3);

        inputfile.close();//�ر��ļ�
        inputfile.clear();//���״̬
        map_camera_parameter_Matrix.insert(std::make_pair(image_index,P)); //���뵽map<ͼƬ��,P����>

        //����״̬,��ȡ��һ�����P����
        filename = filepath;
        ++image_index;
    }
}

//��ȡpmvs�ؽ���Ϣ�ļ�option.patch, ����option.patch·��,map<ͼƬ��,P����>;; ���multimap<����,ͼƬ��Ϣ>;
void ReadOption_Patch(const std::string &filepath, 
                      const std::map<int,MatrixXd> &map_camera_parameter_Matrix,
                      std::multimap<imageID,imageorder_coord> &image_points_coord,
                      std::multimap<pointID,pointorder_coord> &point_images_coord,
                      std::vector<imageorder_coord> &vecpoint3Dcoord)
{
    std::string str_PATCHES, str_PATCHS;
    int total_good_image = 0; //photometric consistency score associated with the point
    int good_image_index = 0;
    int point_index = 0;
    int total_point = 0;

    pointorder_coord pc;
    imageorder_coord ic;
    imageorder_coord ictemp;

    Vector4d point3Dcoord;
    MatrixXd imageXYcoord;
    MatrixXd P(3,4);

    std::string filename = filepath + "option.patch";
    std::ifstream inputfile(filename);
    if (inputfile)
    {
        inputfile >> str_PATCHES;
        if (str_PATCHES != "PATCHES")
        {
            std::cout << "option.patch ��ʽ����: " << std::endl;
            exit(-1);
        }
        inputfile >> total_point;
        for (point_index; point_index != total_point; ++point_index)
        {
            ic.pointID = point_index; //��ȡ����(�����Զ����)
            inputfile >> str_PATCHS;
//             if (str_PATCHS != "PATCHS")
//             {
//                 std::cout << "��ʽ����: " << std::endl;
//                 exit(-1);
//             }
            inputfile >> point3Dcoord(0) >> point3Dcoord(1) >> point3Dcoord(2) >> point3Dcoord(3);
            ictemp.pointID = point_index;
            ictemp.x = point3Dcoord(0);
            ictemp.y = point3Dcoord(1);

            vecpoint3Dcoord.push_back(ictemp);
            inputfile.ignore(1024,'\n'); //����point3Dcoord��ʣ���ַ�ֱ���س�.
            inputfile.ignore(1024,'\n'); //���Է�����ֱ���س�.
            inputfile.ignore(1024,'\n'); //����"photometric consistency score associated with the point"ֱ���س�.
            inputfile >> total_good_image;
            for (int i = 0; i != total_good_image; ++i) //��ȡ���������
            {
                inputfile >> good_image_index; 
                P = map_camera_parameter_Matrix.at(good_image_index);
                imageXYcoord  =  P * point3Dcoord;
                imageXYcoord /=  imageXYcoord(2);

                pc.imageID = good_image_index; //��ȡͼƬ��
                ic.x = pc.x       = imageXYcoord(0,0);
                ic.y = pc.y       = imageXYcoord(1,0);

                image_points_coord.insert(std::make_pair(good_image_index,ic));
                point_images_coord.insert(std::make_pair(point_index,pc));
            }
            inputfile.ignore(1024,'\n'); //����good_image_index��ʣ���ַ�ֱ���س�.
            inputfile.ignore(1024,'\n'); //����total_worse_imageֱ���س�.
            inputfile.ignore(1024,'\n'); //����worse_image_index��ֱ���س�.
            inputfile.ignore(1024,'\n'); //���Կջس�.
        }
    }
    else
        std::cout << "�޷���ȡ: " << filename << std::endl;
}

//�������ͼƬ����������Ϣ����ǰĿ¼��;
void SavePointOrderCoord( const std::string savefilepath,
                          const std::multimap<pointID,pointorder_coord> &point_images_coord)
{
    std::string filename = savefilepath + "option_PointOrderCoord.txt";
    int count_point_index = 0;

    std::ofstream outfile(filename);
    if (outfile)
    {
        std::multimap<pointID,pointorder_coord>::const_iterator iter = point_images_coord.begin();
        for (iter; iter != point_images_coord.end();)
        {
            count_point_index =  point_images_coord.count(iter->first); //ÿ�����ж�����ͼƬ;
            outfile << "P" << iter->first << "\n"; //д�����
            for (int i = 0; i != count_point_index; ++i) //д��ͼƬ��,���ڸ�ͼ��������x,���ڸ�ͼ��������y,
            {
                outfile.unsetf(std::ios::floatfield);
                outfile.precision(14);
                outfile << "I" << iter->second.imageID << " "
                        << iter->second.x       << " -" //��Global Mapper��ʱ�Ӹ�����
                        << iter->second.y       << "\n";
                ++iter; //����iterator
            }
        }
    outfile.close();
    }
    else
        std::cout << "�����ļ�ʧ��!" << std::endl;
}


void SaveImageOrederCoord( const std::string savefilepath,
                           const std::multimap<imageID,imageorder_coord> &image_points_coord)
{
    std::string str_point_index,filename;
    int count_image_index = 0;

    std::stringstream ss;
    
    std::multimap<imageID,imageorder_coord>::const_iterator iter = image_points_coord.begin();
    for (iter; iter != image_points_coord.end();)
    {
        ss.clear();
        count_image_index =  image_points_coord.count(iter->first); //ÿ�����ж�����ͼƬ;
        ss << iter->first;
        ss >> str_point_index;
        filename = savefilepath + str_point_index + ".txt";
        std::ofstream outfile(filename);
        if (outfile)
        {
            outfile << "I" << iter->first << "\n"; //д�����
            for (int i = 0; i != count_image_index; ++i) //д��ͼƬ��,���ڸ�ͼ��������x,���ڸ�ͼ��������y,
            {
                outfile.unsetf(std::ios::floatfield);
                outfile.precision(14);
                outfile << "P" << iter->second.pointID << " "
                    << iter->second.x       << " -" //��Global Mapper��ʱ�Ӹ�����
                    << iter->second.y       << "\n";
                ++iter; //����iterator
            }
        }
        else
            std::cout << "�����ļ�ʧ��!" << std::endl;
        outfile.close();
    }
        
    
}

void GetCameraParameter(int CameraID,const std::map<int,MatrixXd> &P)
{
    std::map<int,MatrixXd>::const_iterator iter = P.find(CameraID);
    if(iter != P.end())
        std::cout << iter->second;
    else
        std::cout << "û�и����P����\n";

}

void GetPoint3Dcoord(int Pointname,const std::vector<imageorder_coord> &vecpoint3Dcoord)
{
    if (Pointname < 0 || Pointname > vecpoint3Dcoord.size() || vecpoint3Dcoord.size() == 0)
    {
        std::cout << "û�иõ�: " << Pointname << std::endl;
       // exit(-1);
    }
    else
	{
			std::cout << vecpoint3Dcoord[Pointname].pointID << "\n"
                      << vecpoint3Dcoord[Pointname].x       << "\n";
	}

}

void GetPointImageCoord(const int pointname,
                        const std::multimap<pointID,pointorder_coord> &point_images_coord)
{
	std::multimap<pointID,pointorder_coord>::const_iterator iter = point_images_coord.find(pointname);
	if (iter != point_images_coord.end())
	{
		int cntimage = point_images_coord.count(iter->first);
		std::cout << "point: " << pointname << "\n";
		for (int i = 0; i != cntimage; ++i)
		{
			std::cout << "image: " << iter->second.imageID << " coord: " << iter->second.x << " " << iter->second.y << "\n";
		}
	}
	else
		std::cout << "û�иõ���Ϣ.\n";
}

int main(int argc, char** argv) 
{
    if (argc != 2)
    {
        std::cout << "Usage: backimagecoor <pmvs����Ŀ¼>\n"
                  << "eg: backimagecoor E:\\3D_reconstruction\\result\\pmvs\n";
        return -1;
    }

    //pmvs����Ŀ¼
    std::string pmvs_path = argv[1];
    //models����Ŀ¼
    std::string pmvs_models_path = pmvs_path + "\\models\\";
    //�������txt����Ŀ¼
    std::string pmvs_txt_path = pmvs_path + "\\txt\\";
    //visualize����Ŀ¼
    std::string pmvs_visualize_path = pmvs_path + "\\visualize\\";

    //��ȡ�������
    std::map<int,MatrixXd> map_camera_parameter_Matrix;
    ReadCameraParameter(pmvs_txt_path,map_camera_parameter_Matrix);

    //��ȡoption.patch�ļ�
    std::multimap<imageID,imageorder_coord> image_points_coord;
    std::multimap<pointID,pointorder_coord> point_images_coord;
    std::vector<imageorder_coord> vecpoint3Dcoord;
    ReadOption_Patch(pmvs_models_path,map_camera_parameter_Matrix,image_points_coord,point_images_coord,vecpoint3Dcoord);
   
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
				GetCameraParameter(66,map_camera_parameter_Matrix);
			}
			else if (inputitem == 2)
			{
				std::cout << "\n���������,(��Ŵ�0��ʼ)\n";
				std::cin >> pi;
				std::cin.sync();
				GetPoint3Dcoord(pi,vecpoint3Dcoord);
			}
			else if (inputitem == 3)
			{
				std::cout << "\n���������,(��Ŵ�0��ʼ)\n";
            
				if (std::cin >> pi)
				{
					GetPointImageCoord(pi,point_images_coord);
				}
				std::cin.sync();
			}
			else if(inputitem == 0)
			{
				exit(0);							   
			}
		}
    }

    //����(�Ե����)��Ϣ;
   // SavePointOrderCoord(pmvs_models_path,point_images_coord);

    //����(��ͼƬ����)��Ϣ;
   // SaveImageOrederCoord(pmvs_models_path,image_points_coord);

    std::cout << "���!\n";

    return 0;
}