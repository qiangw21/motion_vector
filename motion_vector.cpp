//匹配误差
int matchErr(const cv::Mat &preImg, const cv::Mat &curImg, int preX, int preY, int curX, int curY, 
    const int blockHeight=8, const int blockWidth=8)
{
    int blockErr=0;
    for(int i=0;i<blockHeight;++i){
        const uchar* preData=preImg.ptr<uchar>(preY+i);
        const uchar* curData=curImg.ptr<uchar>(curY+i);
        for(int j=0;j<blockWidth;++j){
            blockErr+=abs(curData[curX+j]-preData[preX+j]);
        }
    }
    return blockErr;
}
//三步法
void TSSSearch(const cv::Mat &preImg, const cv::Mat &curImg, int preX, int preY, int curX, int curY,Point &moveVector,
    const int searchLength=4, const int blockHeight=8, const int blockWidth=8)
{
    std::vector<Point> searchPoint{{-searchLength,searchLength},{0,searchLength},{searchLength,searchLength},{-searchLength,0},
                                   {searchLength,0},{-searchLength,-searchLength},{0,-searchLength},{searchLength,-searchLength},{0,0}};
    
    int blockmatcherr=0;
    int flagMin=8;//记录当前匹配坐标
    int minMatch=matchErr(preImg,curImg,preX,preY,curX,curY,blockHeight,blockWidth);
    for(int i=0;i<8;++i){
        int searchPreX=preX+searchPoint[i].x;
        int searchPreY=preY+searchPoint[i].y;
        if(searchPreX>=0&&searchPreX<=preImg.cols-blockWidth&&searchPreY>=0&&searchPreY<=preImg.rows-blockHeight){
            blockmatcherr=matchErr(preImg,curImg,searchPreX,searchPreY,curX,curY,blockHeight,blockWidth);
        }else{
            blockmatcherr=-1;
        }
        if(blockmatcherr!=-1&&blockmatcherr<minMatch){
            minMatch=blockmatcherr;
            flagMin=i;
        }
    }
    moveVector.x-=searchPoint[flagMin].x;
    moveVector.y-=searchPoint[flagMin].y;
    if(searchLength==1){
        return;
    }else
    {
        TSSSearch(preImg,curImg,preX+searchPoint[flagMin].x,preY+searchPoint[flagMin].y,curX,curY, 
                    moveVector,searchLength/2,blockHeight,blockWidth);
    }
    
}
//菱形法
void diamondSearch(const cv::Mat &preImg, const cv::Mat &curImg, int preX, int preY, int curX, int curY,Point &moveVector, 
    const int blockHeight=8, const int blockWidth=8){
	int blockmatcherr=0;
	while(true){
		std::vector<Point> searchPoint{{-1,0},{1,0},{0,-1},{0,1},{0,0}};
		int minMatch=matchErr(preImg,curImg,preX,preY,curX,curY,blockHeight,blockWidth);
		int flagMin=4;//记录当前匹配坐标	
		for(int i=0;i<4;++i){
			int searchPreX=preX+searchPoint[i].x;
			int searchPreY=preY+searchPoint[i].y;
			if(searchPreX>=0&&searchPreX<=preImg.cols-blockWidth&&searchPreY>=0&&searchPreY<=preImg.rows-blockHeight){
				blockmatcherr=matchErr(preImg,curImg,searchPreX,searchPreY,curX,curY,blockHeight,blockWidth);
			}else{
				blockmatcherr=-1;
			}
			if(blockmatcherr!=-1&&blockmatcherr<minMatch){
           		minMatch=blockmatcherr;
            	flagMin=i;
        	}
		}
		if(flagMin!=4){
			preX+=searchPoint[flagMin].x;
			preY+=searchPoint[flagMin].y;
		}else{
			break;
		}
	}
	moveVector.x=curX-preX;
  moveVector.y=curY-preY;
}
//计算运动矢量
void motionEstimation(const cv::Mat &preImg, const cv::Mat &curImg, cv::Mat &motionVector,
    const int searchLength, const int blockHeight, const int blockWidth){
    if(preImg.type()!=CV_8UC1||curImg.type()!=CV_8UC1){
        std::cerr<<"Input Image must be Gray!"<<std::endl;
        return;
    }
    if(preImg.size()!=curImg.size()){
        std::cerr<<"preImg and curImg must have same size."<<std::endl;
        return;
    }

    int width=curImg.cols;
    int height=curImg.rows;
    motionVector=cv::Mat::zeros(height,width,CV_32FC2);
    int blockX=width/blockWidth;
    int blockY=height/blockHeight;
    for(int y=0;y<blockY;++y){
        for(int x=0;x<blockX;++x){
            Point curVector{0,0};
            //TSSSearch(preImg,curImg,x*blockWidth,y*blockHeight,x*blockWidth,y*blockHeight,curVector,searchLength,blockHeight,blockWidth);
			      diamondSearch(preImg,curImg,x*blockWidth,y*blockHeight,x*blockWidth,y*blockHeight,curVector,blockHeight,blockWidth);
            for(int i=y*blockHeight;i<y*blockHeight+blockHeight;++i){
				        float* motionData=motionVector.ptr<float>(i);
                for(int j=x*blockWidth;j<x*blockWidth+blockWidth;++j){
					        motionData[2*j]=curVector.x;
					        motionData[2*j+1]=curVector.y;    
                }
            }
        }
		} 
}
