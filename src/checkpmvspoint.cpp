#include "checkpmvspoint.hpp"

#ifdef max
#undef max
#endif

void cmpvsp::init(std::string pmvspath)
{
	if (!pmvspath.empty())
	{
		char c = pmvspath[pmvspath.size()-1];
		if (c == '\\')
		{
			pmvspath_ = pmvspath;
		}
		else
			pmvspath_ = pmvspath + "\\";
	}
	else
	{
		std::cout << "pmvs路径为空.\n";
		exit(-1);
	}
	pmvsmodels_		= pmvspath_ + "models\\";
	pmvstxtpath_	= pmvspath_ + "txt\\";
	pmvsvisualize_	= pmvspath_ + "visualize\\";

}

//读取相机P矩阵,输入相机P矩阵路径,输出map<图片名,P矩阵>;
void cmpvsp::ReadCameraParameter()
{
    std::string str_CONTOUR;
    std::string filename = pmvstxtpath_;
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
            std::cout << "读取 " << image_index << " 个相机参数文件." << std::endl;
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
        cameraMatrix_.push_back(P); //插入到map<图片名,P矩阵>

        //更新状态,读取另一个相机P矩阵
        filename = pmvstxtpath_;
        ++image_index;
    }
}

//读取pmvs重建信息文件option.patch, 输入option.patch路径,map<图片名,P矩阵>;; 输出multimap<点名,图片信息>;
void cmpvsp::ReadOptionPatch()
{
    std::string str_PATCHES, str_PATCHS;
    int total_point = 0;
	int total_good_image = 0;
    int good_image_index = 0;
    int point_index = 0;

	point_info mypoint; 
	Vector4d point_world_coord;
	RowVector3d imageXYcoord;
    MatrixXd P(3,4);

    std::string filename = pmvsmodels_ + "option.patch";
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
			inputfile >> str_PATCHS;
			inputfile >> point_world_coord(0) >> point_world_coord(1) >> point_world_coord(2) >> point_world_coord(3);
			mypoint.pointID = point_index;
			mypoint.wx = point_world_coord(0);
			mypoint.wy = point_world_coord(1);
			mypoint.wz = point_world_coord(2);

			inputfile.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
			inputfile.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
			inputfile.ignore(std::numeric_limits<std::streamsize>::max(),'\n');

			inputfile >> total_good_image;
			mypoint.pointvisibleimages = total_good_image;
			for (int i = 0; i != total_good_image; ++i)
			{
				inputfile >> good_image_index;
				P = cameraMatrix_.at(good_image_index);
				imageXYcoord  =  P * point_world_coord;
				imageXYcoord /=  imageXYcoord(2);

				mypoint.imageID = good_image_index;
				mypoint.px		= imageXYcoord(0);
				mypoint.py		= imageXYcoord(1);
				point_.push_back(mypoint);

			}

			inputfile.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
			inputfile.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
			inputfile.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
		}
		std::cout << "读取了 " << point_index << " 个点坐标.\n";
	}
	else
		std::cout << "无法读取: " << filename << std::endl;
}

void cmpvsp::GetCameraParameter(size_t CameraID)
{
    if (cameraMatrix_.size() != 0 || CameraID < cameraMatrix_.size())
	{
		std::cout.unsetf(std::ios::floatfield);
		std::cout.precision(14);
        std::cout << cameraMatrix_[CameraID] << std::endl;
	}
    else
        std::cout << "没有该相机P矩阵\n";

}

void cmpvsp::GetPointWorldCoord(size_t PointID)
{
    if (point_.size() != 0 || PointID < point_.size())
    {
        Point::const_iterator iter = point_.begin();
		for (iter; iter != point_.end();)
		{
			if (iter -> pointID == PointID)
			{
				std::cout.unsetf(std::ios::floatfield);
				std::cout.precision(14);
				std::cout << "\nP" 
						  << iter->pointID << "\n" 
						  << iter->wx      << " "
						  << iter->wy      << " "
						  << iter->wz      << std::endl;
				break;
			}
			else
				iter += iter->pointvisibleimages;
		}
		if (iter == point_.end())
			std::cout << "没有点:P" << PointID << std::endl;
    }
    else
	{
		std::cout << "没有点:P" << PointID << std::endl;
	}

}

Point cmpvsp::GetPointImageCoord(size_t PointID)
{
	point_info point;
	Point pointvisibleimages;
	if (point_.size() != 0 || PointID < point_.size())
	{
		Point::const_iterator iter = point_.begin();
		for (iter; iter != point_.end();)
		{
			if (iter -> pointID == PointID)
			{
				std::cout << "point: P" << iter->pointID << std::endl;
				for(size_t i = 0; i != iter->pointvisibleimages; ++i)
				{
					point = *iter;
					pointvisibleimages.push_back(point);
					std::cout << "image: I" << iter->imageID << "\n";
					std::cout.unsetf(std::ios::floatfield);
					std::cout.precision(14);
					std::cout << iter->px << " "
							  << iter->py << "\n";
					++iter;
				}
				return pointvisibleimages;
			}
			else
				iter += iter->pointvisibleimages;	
		}
	}
	else
	{
		std::cout << "没有该点信息.\n";
		return pointvisibleimages;
	}
	return pointvisibleimages;
}

void cmpvsp::CutOutPointVisibleImage(size_t PointID)
{
    CImage Img,OutImage;
    CString CFileName,COutImage;
    int image_width;
    int image_heigth;
    int pixelx, pixely;
    int cntpoint = 0;
    int image_index = 0;
    std::string filename,outjpgfilename,str_image_index,str_point_index;
    std::string str_file_index;
    str_file_index.resize(8);  //pmvs文件规定为8为宽度
    COLORREF color = RGB(255,0,0);

	Point pointvisibleimages;
	pointvisibleimages = GetPointImageCoord(PointID);
    
	Point::const_iterator iter = pointvisibleimages.begin();

	std::stringstream ss;
	ss << iter->pointID;
	ss >> str_point_index;

    for (iter; iter != pointvisibleimages.end(); ++iter)
    {
        //处理输入图片路径名称
        image_index = iter->imageID;
        sprintf(&str_file_index[0], "%08d",image_index); //格式化文件名
        filename =  pmvsvisualize_ + str_file_index + ".jpg";

        //处理输出图片路径名称

        outjpgfilename = pmvspath_ + "pointcheck\\" + str_point_index + "_" + str_file_index + "_out.bmp";
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
        pixelx = static_cast<int>(iter->px);
        pixely = static_cast<int>(iter->py);

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
    }
}