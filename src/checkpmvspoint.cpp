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

//图片信息(以点分类)
struct pointorder_coord
{
    int imageID;
    double x;     //点x像素坐标
    double y;     //点y像素坐标
};

//图片信息(以图片名分类)
struct imageorder_coord
{
    int pointID;
    double x;     //点x像素坐标
    double y;     //点y像素坐标
};

//读取相机P矩阵,输入相机P矩阵路径,输出map<图片名,P矩阵>;
void ReadCameraParameter(std::string filepath,std::map<int,MatrixXd> &map_camera_parameter_Matrix)
{
    std::string str_CONTOUR;
    std::string filename = filepath;
    std::string file_index;
    file_index.resize(8);  //pmvs文件规定为8为宽度
    std::ifstream inputfile;
    MatrixXd P(3,4);

    int image_index = 0; //图片序号
    while (1)
    {
        sprintf(&file_index[0], "%08d",image_index); //格式化文件名
        filename =  filename + file_index + ".txt";
        inputfile.open(filename,std::ios_base::in);
        if (!inputfile) //读取文件失败
        {
            ++image_index;
            std::cout << "无法打开文件: " << filename << std::endl;
            break;
        }
        inputfile >> str_CONTOUR;
        if (str_CONTOUR != "CONTOUR") //检查文件格式
        {
            std::cout << "CONTOUR文件格式错误!" << std::endl;
            continue;
        }

        //写入P矩阵
        inputfile >> P(0,0) >> P(0,1) >> P(0,2) >> P(0,3)
                  >> P(1,0) >> P(1,1) >> P(1,2) >> P(1,3)
                  >> P(2,0) >> P(2,1) >> P(2,2) >> P(2,3);

        inputfile.close();//关闭文件
        inputfile.clear();//清空状态
        map_camera_parameter_Matrix.insert(std::make_pair(image_index,P)); //插入到map<图片名,P矩阵>

        //更新状态,读取另一个相机P矩阵
        filename = filepath;
        ++image_index;
    }
}

//读取pmvs重建信息文件option.patch, 输入option.patch路径,map<图片名,P矩阵>;; 输出multimap<点名,图片信息>;
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
            std::cout << "option.patch 格式错误: " << std::endl;
            exit(-1);
        }
        inputfile >> total_point;
        for (point_index; point_index != total_point; ++point_index)
        {
            ic.pointID = point_index; //获取点名(程序自动编号)
            inputfile >> str_PATCHS;
//             if (str_PATCHS != "PATCHS")
//             {
//                 std::cout << "格式错误: " << std::endl;
//                 exit(-1);
//             }
            inputfile >> point3Dcoord(0) >> point3Dcoord(1) >> point3Dcoord(2) >> point3Dcoord(3);
            ictemp.pointID = point_index;
            ictemp.x = point3Dcoord(0);
            ictemp.y = point3Dcoord(1);

            vecpoint3Dcoord.push_back(ictemp);
            inputfile.ignore(1024,'\n'); //忽略point3Dcoord行剩余字符直到回车.
            inputfile.ignore(1024,'\n'); //忽略发向量直到回车.
            inputfile.ignore(1024,'\n'); //忽略"photometric consistency score associated with the point"直到回车.
            inputfile >> total_good_image;
            for (int i = 0; i != total_good_image; ++i) //读取世界坐标点
            {
                inputfile >> good_image_index; 
                P = map_camera_parameter_Matrix.at(good_image_index);
                imageXYcoord  =  P * point3Dcoord;
                imageXYcoord /=  imageXYcoord(2);

                pc.imageID = good_image_index; //获取图片名
                ic.x = pc.x       = imageXYcoord(0,0);
                ic.y = pc.y       = imageXYcoord(1,0);

                //image_points_coord.insert(std::make_pair(good_image_index,ic));
                point_images_coord.insert(std::make_pair(point_index,pc));
            }
            inputfile.ignore(1024,'\n'); //忽略good_image_index行剩余字符直到回车.
            inputfile.ignore(1024,'\n'); //忽略total_worse_image直到回车.
            inputfile.ignore(1024,'\n'); //忽略worse_image_index行直到回车.
            inputfile.ignore(1024,'\n'); //忽略空回车.
        }
    }
    else
        std::cout << "无法读取: " << filename << std::endl;
}

//保存点在图片像素坐标信息到当前目录下;
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
            count_point_index =  point_images_coord.count(iter->first); //每个点有多少张图片;
            outfile << "P" << iter->first << "\n"; //写入点名
            for (int i = 0; i != count_point_index; ++i) //写入图片名,点在该图像素坐标x,点在该图像素坐标y,
            {
                outfile.unsetf(std::ios::floatfield);
                outfile.precision(14);
                outfile << "I" << iter->second.imageID << " "
                        << iter->second.x       << " -" //放Global Mapper暂时加个符号
                        << iter->second.y       << "\n";
                ++iter; //递增iterator
            }
        }
    outfile.close();
    }
    else
        std::cout << "保存文件失败!" << std::endl;
}


void SaveImageOrederCoord( const std::string savefilepath,
                           const std::multimap<imageID,imageorder_coord> &image_points_coord,
                           const int imagename)
{
    std::string str_point_index,filename;
    int cntpoint = 0;

    std::stringstream ss;
    
    std::multimap<imageID,imageorder_coord>::const_iterator iter = image_points_coord.find(imagename);
    if (iter != image_points_coord.end())
    {
        ss.clear();
        cntpoint =  image_points_coord.count(iter->first); //每个点有多少张图片;
        ss << imagename;
        ss >> str_point_index;
        filename = savefilepath + "image" + str_point_index + ".txt";
        std::ofstream outfile(filename);
        if (outfile)
        {
            for (int i = 0; i != cntpoint; ++i) //写入图片名,点在该图像素坐标x,点在该图像素坐标y,
            {
                outfile.unsetf(std::ios::floatfield);
                outfile.precision(14);
                outfile << "P" << iter->second.pointID << " "
                    << iter->second.x       << " -" //放Global Mapper暂时加个符号
                    << iter->second.y       << "\n";
                ++iter; //递增iterator
            }

        }
        else
            std::cout << "无法打开: "<< filename << std::endl;
        outfile.close();
    }//if (iter != image_points_coord.end())  
    else
        std::cout << "无法找到图片:" << imagename << "P矩阵信息!\n";
}

void GetCameraParameter(int CameraID,const std::map<int,MatrixXd> &P)
{
    std::map<int,MatrixXd>::const_iterator iter = P.find(CameraID);
    if(iter != P.end())
        std::cout << iter->second;
    else
        std::cout << "没有该相机P矩阵\n";

}

void GetPoint3Dcoord(int Pointname,const std::vector<imageorder_coord> &vecpoint3Dcoord)
{
    if (Pointname < 0 || Pointname > vecpoint3Dcoord.size() || vecpoint3Dcoord.size() == 0)
    {
        std::cout << "没有该点: " << Pointname << std::endl;
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
		std::cout << "没有该点信息.\n";
}

void CutOutPointImage( const std::string image_in_path,
                       const std::string image_out_path,
                       const std::multimap<pointID,pointorder_coord> &point_images_coord,
                       const int pointname)
{
    CImage Img,OutImage;
    CString CFileName,COutImage;
    //const pointID pid;
    int image_width;
    int image_heigth;
    int pixelx, pixely;
    int cntpoint = 0;
    int image_index = 0;
    std::string filename,outjpgfilename,str_image_index,str_point_index;
    std::string str_file_index;
    str_file_index.resize(8);  //pmvs文件规定为8为宽度
    COLORREF color = RGB(255,0,0);
    std::multimap<pointID,pointorder_coord>::const_iterator iter = point_images_coord.find(pointname);
    if (iter != point_images_coord.end())
    {
        cntpoint = point_images_coord.count(iter->first);
        for (int i = 0; i != cntpoint; ++i)
        {
            //处理输入图片路径名称
            image_index = iter->second.imageID;
            sprintf(&str_file_index[0], "%08d",image_index); //格式化文件名
            filename =  image_in_path + str_file_index + ".jpg";

            //处理输出图片路径名称
            std::stringstream ss;
            ss << pointname;
            ss >> str_point_index;
            outjpgfilename = image_out_path + "pointcheck\\" + str_point_index + "_" + str_file_index + "_out.bmp";
            COutImage = (LPCTSTR)(outjpgfilename).c_str();

            CFileName = (LPCTSTR)(filename).c_str();
            Img.Load(CFileName);
            if (Img.IsNull())
            {
                std::cout << "无法打开图片: " << filename << std::endl;
                //exit(-1);
            }

            image_width = Img.GetWidth();
            image_heigth = Img.GetHeight();
            OutImage.Create(500,500,24);
            pixelx = static_cast<int>(iter->second.x);
            pixely = static_cast<int>(iter->second.y);

            //复制500x500到新图片
            for (int xo = 0; xo != 500; ++xo)
            {
                for (int yo = 0; yo != 500; ++yo)
                {
                    if (pixelx - 250 + xo < 0 ||
                        pixely - 250 + yo < 0 ||
                        pixelx - 250 + xo >= image_width ||
                        pixely - 250 + yo >= image_heigth)
                    {
                        color = RGB(0,0,0);
                        OutImage.SetPixel(xo,yo,color);
                    }
                    else
                    {
                        color = Img.GetPixel(pixelx-250 + xo,pixely - 250 + yo);
                        OutImage.SetPixel(xo,yo,color);
                    }

                }
            }

            //在中心画十字
            for (int xa = 200; xa != 300; ++xa)
            {
                OutImage.SetPixel(xa,250,RGB(255,0,0));
            }
            for (int ya = 200; ya != 300; ++ya)
            {
                OutImage.SetPixel(250,ya,RGB(255,0,0));
            }
            OutImage.Save(COutImage);
            OutImage.Destroy();
            Img.Destroy();


            ++iter; //更新iter
        }

    }


}

int main(int argc, char** argv) 
{
    if (argc != 2)
    {
        std::cout << "Usage: backimagecoor <pmvs工作目录>\n"
                  << "eg: backimagecoor E:\\3D_reconstruction\\result\\pmvs\n";
        return -1;
    }

    //pmvs工作目录
    std::string pmvs_path = argv[1];
    //models工作目录
    std::string pmvs_models_path = pmvs_path + "\\models\\";
    //相机参数txt工作目录
    std::string pmvs_txt_path = pmvs_path + "\\txt\\";
    //visualize工作目录
    std::string pmvs_visualize_path = pmvs_path + "\\visualize\\";
    //result工作目录
    std::string pmvs_result_path = pmvs_path + "\\result\\";
    

    //读取相机参数
    std::map<int,MatrixXd> map_camera_parameter_Matrix;
    ReadCameraParameter(pmvs_txt_path,map_camera_parameter_Matrix);

    //读取option.patch文件
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
        std::cout << "\n1:查询相机P矩阵.\n"
				  << "2:查询点世界坐标.\n"
				  << "3:查询点point在所有图片的坐标.\n"
                  << "4:打印point在所有图片的截图,并保存到result\\pointcheck里.\n"
                  << "5:保存第N张图片上所有点坐标文件\n"
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
				GetCameraParameter(66,map_camera_parameter_Matrix);
			}
			else if (inputitem == 2)
			{
				std::cout << "\n输入点名称,(序号从0开始)\n";
				std::cin >> pi;
				std::cin.sync();
				GetPoint3Dcoord(pi,vecpoint3Dcoord);
			}
			else if (inputitem == 3)
			{
				std::cout << "\n输入点名称,(序号从0开始)\n";
            
				if (std::cin >> pi)
				{
					GetPointImageCoord(pi,point_images_coord);
				}
				std::cin.sync();
			}
            else if (inputitem == 4)
            {
                std::cout << "\n输入点名称,(序号从0开始)\n";
                std::cin >> pi;
                std::cin.sync();
                CutOutPointImage(pmvs_visualize_path,pmvs_result_path,point_images_coord,pi);
            }
            else if (inputitem == 5)
            {
                std::cout << "\n输入图片名称,(序号从0开始)\n";
                std::cin >> pi;
                std::cin.sync();
                SaveImageOrederCoord(pmvs_result_path,image_points_coord,pi);
            }
			else if(inputitem == 0)
			{
				exit(0);							   
			}
		}
    }

    //保存(以点分类)信息;
   // SavePointOrderCoord(pmvs_models_path,point_images_coord);

    //保存(以图片分类)信息;
   // SaveImageOrederCoord(pmvs_models_path,image_points_coord);

    std::cout << "完成!\n";

    return 0;
}