#include "topics.hpp"
#include <common/srv/get_color.hpp>




//github





// user define begin
//********************************************************************************************************************************//

struct pid_type
{
    float Kp,Ki,Kd;
    float sumClamp,outClamp;
    float curError=0,lastError=0,sumError=0;
    float target;
    float output;
    bool isEnabled;
};

void pidCal(pid_type& obj,float val);

#define SGN(x) ((x)>0?(1):((x)<0?(-1):(0)))

//********************************************************************************************************************************//

cv::Mat& imageProcess(cv::Mat& image,rclcpp::Node::SharedPtr playerNode);//update:ballInView & ballFoundFlag ; goalInView & goalFoundFlag

//********************************************************************************************************************************//

// cv::Mat resultImg;

cv::Vec3f ballInView;   // col,row,radius
bool ballFoundFlag;
int ballNotFoundElapse=0;

cv::RotatedRect goalInView;
bool goalFoundFlag;

const int timer=400;
int secondFlag=0;
bool knowDirection=true;
bool ballInControl=true;
bool bodyToGoal=true;
bool isOnLine=true;
    
int imgRow=0,imgCol=0;

pid_type horiFollow,vertFollow,bodyFollow;

//********************************************************************************************************************************//
// user define end










enum RobotColor {
    COLOR_INVALID,
    COLOR_RED,
    COLOR_BLUE
};

RobotColor GetRobotColor(const std::string &name)
{
    if (name.find("red") != std::string::npos) {
        return COLOR_RED;
    } else if (name.find("blue") != std::string::npos) {
        return COLOR_BLUE;
    } else {
        return COLOR_INVALID;
    }
}

int GetRobotId(const std::string &name)
{
    return static_cast<int>(name.back() - '0');
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string robotName = "maxwell";

    std::string tmp = std::string(argv[1]);
    std::string team = tmp.substr(0, tmp.find_last_of('_'));

    auto playerNode = std::make_shared<rclcpp::Node>(tmp + "_player");
    rclcpp::Client<common::srv::GetColor>::SharedPtr client =
        playerNode->create_client<common::srv::GetColor>("gamectrl/get_color");
    while (!client->wait_for_service(std::chrono::duration<long long>(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(playerNode->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(playerNode->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<common::srv::GetColor::Request>();
    request.get()->team = team;
    auto result = client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(playerNode, result);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->color == "invalid") {
            RCLCPP_ERROR(playerNode->get_logger(), "Not supportted team name. Exiting.");
            return 0;
        }
        robotName = result.get()->color + tmp.substr(tmp.find_last_of('_'));
        RCLCPP_INFO(playerNode->get_logger(), "robotName: %s", robotName.c_str());
    } else {
        RCLCPP_ERROR(playerNode->get_logger(), "Exiting.");
        return 0;
    }

    RobotColor myColor = GetRobotColor(robotName);
    int myId = GetRobotId(robotName);
    common::msg::BodyTask btask;
    common::msg::HeadTask htask;
    common::msg::GameData gameData;
    btask.type = btask.TASK_WALK;
    btask.count = 2;
    btask.step = 0.03;
    htask.yaw = 0.0;
    htask.pitch = 45.0;
    auto bodyTaskNode = std::make_shared<BodyTaskPublisher>(robotName);
    auto headTaskNode = std::make_shared<HeadTaskPublisher>(robotName);
    auto imageSubscriber = std::make_shared<ImageSubscriber>(robotName);
    auto imuSubscriber = std::make_shared<ImuDataSubscriber>(robotName);
    auto headSubscriber = std::make_shared<HeadAngleSubscriber>(robotName);
    auto resImgPublisher = std::make_shared<ResultImagePublisher>(robotName);
    auto gameSubscriber = std::make_shared<GameDataSubscriber>(robotName);
    rclcpp::WallRate loop_rate(10.0);
    float initYaw = 0.0;

    while (rclcpp::ok()) {
        rclcpp::spin_some(bodyTaskNode);
        rclcpp::spin_some(headTaskNode);
        rclcpp::spin_some(imageSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(headSubscriber);
        rclcpp::spin_some(resImgPublisher);
        rclcpp::spin_some(gameSubscriber);
        gameData = gameSubscriber->GetData();
        auto imuData = imuSubscriber->GetData();
        auto image = imageSubscriber->GetImage().clone();
        auto headAngle = headSubscriber->GetData();

        // ----------------- 可以修改的部分 begin--------------------
        // 郑重声明：以下说明仅供参考，实际情况以实际为准
        // 下面提供的是一些可能会用到的相关信息接口说明和示例，可以根据自己的需要去使用，不必要纠结于现有的代码结构
        if (gameData.state == gameData.STATE_INIT){
            // 每次开球时都会进入这里
            // 例如：RCLCPP_INFO(playerNode->get_logger(), "%d", gameData.blue_score);
            // INII状态时，1号机器人位于己方这边的中圈上，2号机器人位于己方的禁区线上
            initYaw = imuData.yaw; // 获取init状态时，机器人的朝向，此时的方向即为自己进攻的方向


            
            secondFlag=1;
            // 每次开球时都会进入这里
            // 例如：RCLCPP_INFO(playerNode->get_logger(), "%d", gameData.blue_score);

            ballInView={0,0,0};
            goalInView={cv::Point2f(0,0),cv::Size2f(0,0),0};

            if((!imgRow||!imgCol)&&!image.empty())
            {
                imgRow=image.rows;
                imgCol=image.cols;
                RCLCPP_INFO(playerNode->get_logger(),"%s--got image size:",argv[1]);
                RCLCPP_INFO(playerNode->get_logger(),"rows=%d,cols=%d",imgRow,imgCol);
            }

            horiFollow.Kp=0.01;
            horiFollow.Ki=0.05;
            horiFollow.Kd=0;
            horiFollow.sumClamp=100;
            horiFollow.outClamp=100;
            horiFollow.isEnabled=true;

            vertFollow.Kp=0.016;
            vertFollow.Ki=0.05;
            vertFollow.Kd=0;
            vertFollow.sumClamp=100;
            vertFollow.outClamp=100;
            vertFollow.isEnabled=true;

            bodyFollow.Kp=0.1;
            bodyFollow.Ki=0.05;
            bodyFollow.Kd=0;
            bodyFollow.sumClamp=100;
            bodyFollow.outClamp=100;
            bodyFollow.isEnabled=false;

            btask.step=0;
            htask.yaw=0.0;
            htask.pitch=45.0;

        }

        if(gameData.state==gameData.STATE_PLAY)
            secondFlag++;




        if (!image.empty())
        {
            resImgPublisher->Publish(imageProcess(image,playerNode)); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
        }





        // 将机器人的当前朝向减去init时的朝向，即可得到机器人相对于初始位置的方向，
        // 这样可以保证自己不管是红色或者蓝色，其yaw都是在面朝对方球门时为0
        // 下面提供这种转换的方法，需要该转换的可以把注释符号去掉
        // imuData.yaw = imuData.yaw - initYaw; 

        // IMU数据里的yaw信息是统一的，属于绝对方向，但是红色机器人和蓝色机器人的进球方向是不一样的，
        // 假如红色的进球方向是0度，则蓝色的进球方向就是180度
        // 比赛时使用什么颜色的机器人是现场决定的，所以对于两种颜色的机器人，都需要考虑如何
        // 如果使用了上面的转换方法，则不需要再关心不同颜色机器人的方向问题
        if (myColor == COLOR_RED) {
            // 使用的红色的机器人
        } else if (myColor == COLOR_BLUE) {
            // 使用的蓝色的机器人
        }

        // 复赛每个组使用两台机器人，需要根据不同的机器人id制定策略的队伍，可以利用myId进行区分
        // 机器人出界后，会将出界的机器人放回己方半场靠近中线的约0.5米处，此时该机器人会有30s的惩罚
        // 由于有两台机器人，每一天机器人重生的位置都是在己方固定的某一边，并不是从哪边出去就从哪边重生，
        // 具体哪一台从哪边重生，请自行运行仿真查看
        if (myId == 1) {
            
            // btask.type=btask.TASK_WALK;
            if(ballFoundFlag||ballNotFoundElapse<0*10)
            {
                if(!ballFoundFlag)
                    ballNotFoundElapse++;
                else
                    ballNotFoundElapse=0;
                    


                
                horiFollow.target=imgCol/2;
                pidCal(horiFollow,ballInView[0]);
                htask.yaw-=horiFollow.output;
                // htask.yaw=htask.yaw<0?htask.yaw+360:htask.yaw;
                // htask.yaw=htask.yaw>360?htask.yaw-360:htask.yaw;
                
                vertFollow.target=imgRow/2;
                pidCal(vertFollow,ballInView[1]);
                htask.pitch+=vertFollow.output;
                htask.pitch=htask.pitch<0?0:htask.pitch;
                htask.pitch=htask.pitch>90?90:htask.pitch;

                // bodyFollow.target=headAngle.yaw;
                // while(bodyFollow.target>=180)bodyFollow.target-=360;
                // while(bodyFollow.target<-180)bodyFollow.target+=360;
                bodyFollow.target=0;
                pidCal(bodyFollow,headAngle.yaw);
                btask.turn=bodyFollow.output;
                if(abs(bodyFollow.curError)<5)
                    bodyFollow.isEnabled=false;


                RCLCPP_INFO(playerNode->get_logger(),"%lf",btask.turn);


                // btask.step=-1;
                

                // static bool hfocusedflag,vfocusedflag;

                // // body yaw
                // if(ballInView[0]<imgCol/2+25&&ballInView[0]>imgCol/2-25)
                // {
                //     btask.turn=0;
                //     btask.step=5;
                //     RCLCPP_INFO(playerNode->get_logger(),"%s--forward",argv[1]);
                //     hfocusedflag=true;
                // }
                // else
                // {
                //     btask.turn=(-0.05)*(imgCol/2-ballInView[0]);
                //     btask.lateral=(0.5)*(imgCol/2-ballInView[0]);
                //     btask.step=-0.02;
                //     RCLCPP_INFO(playerNode->get_logger(),"%s--turn/lateral",argv[1]);ros2 launch player player_launch.py color:=red
                //     hfocusedflag=false;
                // }

                // // head pitch
                // if(ballInView[1]<imgRow/2+25&&ballInView[1]>imgRow/2-25)
                // {
                //     RCLCPP_INFO(playerNode->get_logger(),"%s--focused",argv[1]);
                //     vfocusedflag=true;
                // }
                // else
                // {
                //     htask.pitch+=(-0.02)*(imgRow/2-ballInView[1]);
                //     htask.pitch=htask.pitch<0?0:htask.pitch;
                //     htask.pitch=htask.pitch>90?90:htask.pitch;
                //     RCLCPP_INFO(playerNode->get_logger(),"%s--focusing",argv[1]);
                //     vfocusedflag=false;ros2 launch player player_launch.py color:=red
                // {
                //     btask.type=btask.TASK_ACT;
                //     if(imuData.yaw>0)
                //     {
                //         btask.actname="left_kick";
                //     }
                //     else
                //     {
                //         btask.actname="right_kick";
                //     }
                // }

                RCLCPP_INFO(playerNode->get_logger(),"secondFlag=%d",secondFlag);
                RCLCPP_INFO(playerNode->get_logger(),"knowDirection=%d",knowDirection);
                

                if(abs(headAngle.yaw)>15)//保持对球控制初始化
                    ballInControl=false;
                else
                    ballInControl=true;

                // if((secondFlag%timer==0))//找到球门方向初始化
                //     knowDirection=false;


                static int alignedThreshold=30;
                if(goalFoundFlag&&goalInView.size.area()>100000)
                    alignedThreshold=60;
                // else
                //     alignedThreshold=25;

                RCLCPP_INFO(playerNode->get_logger(),"------%d,%f",alignedThreshold,goalInView.size.area());
                    

                if(knowDirection&&ballInControl&&isOnLine)
                {
                    if(ballFoundFlag&&abs(headAngle.yaw)<15)
                    {
                        RCLCPP_INFO(playerNode->get_logger(),"Go Ahead!");
                        btask.turn=0;
                        btask.lateral=0;
                        btask.step=1;
                    }
                    else if (ballFoundFlag==false)
                    {
                        RCLCPP_INFO(playerNode->get_logger(),"I lose the ball!");
                    }
                    else
                    {
                        RCLCPP_INFO(playerNode->get_logger(),"I am Tring to Follow the Ball!");
                        //btask.turn=(-0.01)*(imgCol/2-ballInView[0]);
                        btask.lateral=(0.5)*SGN(headAngle.yaw);
                        btask.step=-0.01;
                    }







                    // head pitch
                    // if(ballInView[1]<imgRow/2+25&&ballInView[1]>imgRow/2-25)
                    // {
                    //     RCLCPP_INFO(playerNode->get_logger(),"the Ball is In My Eyes!");
                    // }
                    // else
                    // {
                    //     RCLCPP_INFO(playerNode->get_logger(),"I am Finding the Ball!");
                    //     htask.pitch+=(-0.02)*(imgRow/2-ballInView[1]);
                    //     htask.pitch=htask.pitch<0?0:htask.pitch;
                    //     htask.pitch=htask.pitch>90?90:htask.pitch;
                    // }

                    // if(180-abs(imuData.yaw)>5)
                    // {
                    //     RCLCPP_INFO(playerNode->get_logger(),"imuData.yaw=%f",imuData.yaw);
                    //     if(imuData.yaw>0)
                    //     {
                    //         btask.lateral=-0.1;
                    //         btask.step=-0.1;
                    //         btask.turn=0.03;
                    //     }
                    //     else
                    //     {
                    //         btask.lateral=0.1;
                    //         btask.step=-0.1;
                    //         btask.turn=-0.03;
                    //     }
                    // }
                }


                if(!ballInControl)//失去对球控制退后重新控制
                {
                    RCLCPP_INFO(playerNode->get_logger(),"I am tring to control the ball again!");
                    btask.lateral=(0.5)*SGN(headAngle.yaw);
                    btask.step=-0.01;
                    // btask.turn=-0.01*(htask.yaw);
                }

                if(abs(headAngle.yaw)>70)
                {
                    btask.turn=0;
                    btask.lateral=0;
                    btask.step=-1;
                }


                // if(!knowDirection)
                // {
                //     RCLCPP_INFO(playerNode->get_logger(),"Direction Finding```");
                //     isOnLine=false;
                //     btask.step=0;
                //     btask.turn=0;
                //     btask.lateral=0;
                //     htask.pitch=0;
                //
                //     if(goalFoundFlag&&goalInView.center.x<imgCol/2+25&&goalInView.center.x>imgCol/2-25)
                //     {
                //         knowDirection=true;
                //         bodyFollow.isEnabled=false;
                //     }
                //
                //     else if(goalFoundFlag==false)//找球门
                //     {
                //         htask.yaw+=SGN(imuData.yaw)*10;
                //     }
                //     else//修正方向
                //     {
                //         bodyFollow.isEnabled=true;
                //         //根据球门修正方向
                //     }
                // }

                // if(!isOnLine)
                // {
                //     htask.pitch-=5;
                //     if(ballFoundFlag&&ballInView[0]<imgCol/2+25&&ballInView[0]>imgCol/2-25)
                //     {
                //         isOnLine=true;
                //     }
                //     else if(!ballFoundFlag)
                //     {
                //         btask.turn=0;
                //         btask.step=0;
                //         btask.lateral=0;
                //         RCLCPP_INFO(playerNode->get_logger(),"%s--stop",argv[1]);
                //         RCLCPP_INFO(playerNode->get_logger(),"%s--finding the ball!",argv[1]);
                //         htask.yaw+=SGN(ballInView[0]-imgCol/2)*5;
                //         htask.pitch=20*sin(secondFlag*0.1)+20;
                //         //找球
                //     }
                //     else
                //     {
                //         btask.step=-0.5;
                //         btask.lateral=SGN(htask.yaw)*1;
                //     }
                // }

                if(180-abs(imuData.yaw)>10)
                {
                    RCLCPP_INFO(playerNode->get_logger(),"imuData.yaw=%f",imuData.yaw);
                    btask.step=0;
                    btask.lateral=0;
                    btask.turn=5*SGN(imuData.yaw);
                      
                }
            }
            
            else
            {

                if(abs(headAngle.yaw)<45)
                {
                    btask.turn=0;
                    btask.step=1;
                    btask.lateral=0;
                    RCLCPP_INFO(playerNode->get_logger(),"%s--ball lost,rush!!!!!!",argv[1]);
                    htask.pitch=60;
                }
                else
                {
                    btask.turn=0;
                    btask.step=-1;
                    btask.lateral=0;
                    RCLCPP_INFO(playerNode->get_logger(),"%s--ball lost,finding the ball!",argv[1]);
                    htask.yaw=75*SGN(headAngle.yaw);
                    htask.pitch=45;
                }
                
                
            }
            RCLCPP_INFO(playerNode->get_logger(),"yaw=%f,pitch=%f,roll=%f",imuData.yaw,imuData.pitch,imuData.roll);
            RCLCPP_INFO(playerNode->get_logger(),"headyaw=%f,pitch=%f",headAngle.yaw,headAngle.pitch);


        } else if (myId == 2) {
            // 2号机器人
        }

        // 郑重声明：以上说明仅供参考，实际情况以实际为准
        // ----------------- 可以修改的部分 end--------------------

        // 下面的代码不能动
        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        loop_rate.sleep();
    }
    return 0;
}










// function implementation begin
//********************************************************************************************************************************//

void pidCal(pid_type& obj,float val)
{
    if(obj.isEnabled)
    {
        obj.lastError=obj.curError;
        obj.curError=val-obj.target;

        obj.sumError+=obj.curError;
        obj.sumError=obj.sumError>obj.sumClamp?obj.sumClamp:obj.sumError;
        obj.sumError=obj.sumError<-obj.sumClamp?-obj.sumClamp:obj.sumError;

        obj.output=obj.Kp*(obj.curError+obj.Ki*obj.sumError+obj.Kd*(obj.curError-obj.lastError));
        obj.output=obj.output>obj.outClamp?obj.outClamp:obj.output;
        obj.output=obj.output<-obj.outClamp?-obj.outClamp:obj.output;
    }
    else
    {
        obj.output=0;
    }
    

}

//********************************************************************************************************************************//

cv::Mat& imageProcess(cv::Mat& image,rclcpp::Node::SharedPtr playerNode)
{
    static cv::Mat grayImg,hsvImg,filteredImg1,filteredImg2,filteredImg3,resultImg;
    static std::vector<cv::Mat> hsvSplit;
    static std::vector<cv::Vec3f> circleSeq;
    static std::vector<std::vector<cv::Point>> frameContours;
    static std::vector<cv::Vec4i> frameHierachy;


    cv::cvtColor(image,grayImg,cv::COLOR_BGR2GRAY);
    cv::medianBlur(grayImg,grayImg,3);

    cv::cvtColor(image,hsvImg,cv::COLOR_BGR2HSV);
    // cv::split(hsvImg,hsvSplit);
    // cv::equalizeHist(hsvSplit[2],hsvSplit[2]);
    // cv::merge(hsvSplit,hsvImg);

    // ball & net white
    cv::inRange(hsvImg,cv::Scalar(0,0,225),cv::Scalar(179,30,255),filteredImg1);//ballwhite->white
    cv::medianBlur(filteredImg1,filteredImg1,3);
    cv::GaussianBlur(filteredImg1,filteredImg1,cv::Size(3,3),0,0);

    // frame gray
    cv::inRange(hsvImg,cv::Scalar(0,0,130),cv::Scalar(179,30,200),filteredImg2);//framegray->white
    // cv::inRange(hsvImg,cv::Scalar(0,30,30),cv::Scalar(30,255,255),filteredImg3);//blue->white
    // cv::medianBlur(filteredImg3,filteredImg3,15);
    // // wipe off blue and surrounding sh!t
    // for(int r=0;r<imgRow;r++)
    // {
    //     for(int c=0;c<imgCol;c++)
    //     {
    //         if(filteredImg3.at<uchar>(r,c))
    //             filteredImg3.at<uchar>(r,c)=0;
    //         else
    //             filteredImg3.at<uchar>(r,c)=255;
    //     }
    // }
    // cv::Mat element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    // cv::morphologyEx(filteredImg3,filteredImg3,cv::MORPH_OPEN,element);
    // cv::morphologyEx(filteredImg3,filteredImg3,cv::MORPH_CLOSE,element);
    // filteredImg2.copyTo(filteredImg2,filteredImg3);
    cv::medianBlur(filteredImg2,filteredImg2,5);
    cv::GaussianBlur(filteredImg2,filteredImg2,cv::Size(3,3),0,0);

    
    // resultimg select
    // cv::cvtColor(grayimg,resultImg,cv::COLOR_GRAY2BGR);
    // cv::cvtColor(filteredImg1,resultImg,cv::COLOR_GRAY2BGR);
    cv::cvtColor(filteredImg2,resultImg,cv::COLOR_GRAY2BGR);
    // resultImg=image;





    // ball detect
    cv::HoughCircles(grayImg,circleSeq,cv::HOUGH_GRADIENT,1.5,128,100,25,4,72);

    if(circleSeq.empty())
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--ball not found");

        // ballInView[2]=-1;
        ballFoundFlag=false;
    }
    else if(circleSeq.size()==1)
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--ball found");

        ballInView=circleSeq[0];
        cv::circle(resultImg,cv::Point(ballInView[0],ballInView[1]),ballInView[2],cv::Scalar(255,0,0),3);
        ballFoundFlag=true;
    }
    else
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--more than one ball found");

        static float maxWhiteRatio;
        maxWhiteRatio=0.5;

        for(auto item:circleSeq)
        {
            cv::circle(resultImg,cv::Point(item[0],item[1]),item[2],cv::Scalar(255,0,0));
            // RCLCPP_INFO(playerNode->get_logger(),"(%f,%f),%f",item[0],item[1],item[2]);

            // white ratio cal
            static int whiteCount,pixelCount;
            whiteCount=pixelCount=0;

            for(int r=item[1]-item[2];r<=item[1]+item[2];r++)
            {
                for(int c=item[0]-item[2];c<=item[0]+item[2];c++)
                {
                    if((pow(item[1]-r,2)+pow(item[0]-c,2))<=pow(item[2],2))
                    {
                        ++pixelCount;
                        if(filteredImg1.at<uchar>(r,c))
                            ++whiteCount;
                    }
                    
                }
            }

            if(whiteCount/(double)pixelCount>maxWhiteRatio)
            {
                maxWhiteRatio=whiteCount/(double)pixelCount;
                ballInView=item;
            }

        }

        
        // RCLCPP_INFO(playerNode->get_logger(),"%lf,(%f,%f)",maxWhiteRatio,ballInView[0],ballInView[1]);

        if(maxWhiteRatio<=0.51)
        {
            ballFoundFlag=false;
        }
        else
        {
            cv::circle(resultImg,cv::Point(ballInView[0],ballInView[1]),ballInView[2],cv::Scalar(255,0,0),3);
            ballFoundFlag=true;
        }
        
            
        
    }
    




    // goal frame detect
    cv::findContours(filteredImg2,frameContours,frameHierachy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point(-1,-1));
    // cv::drawContours(resultImg,frameContours,-1,cv::Scalar(0,0,255),1,8,frameHierachy);

    std::vector<std::vector<cv::Point>> frameContoursPloy(frameContours.size());
    std::vector<cv::RotatedRect> frameRectPoly;

    for(size_t i=0; i<frameContours.size();i++)
    {
        cv::approxPolyDP(frameContours[i],frameContoursPloy[i],5,false);
        // for(auto item:frameContoursPloy[i])
        // {
        //     cv::circle(resultImg,item,1,cv::Scalar(0,255,0),1);
        // }
        // RCLCPP_INFO(playerNode->get_logger(),"--polysize=%d",frameContoursPloy[i].size());
        if(frameContoursPloy[i].size()<64)
            frameRectPoly.push_back(cv::minAreaRect(frameContoursPloy[i]));

    }

    if(frameRectPoly.empty())
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--goal not found");

        goalFoundFlag=false;
    }
    else if(frameRectPoly.size()==1)
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--goal found");

        goalInView=frameRectPoly[0];

        cv::circle(resultImg,goalInView.center,3,cv::Scalar(0,255,0),3);

        static cv::Point2f vertices[4];
        goalInView.points(vertices);
        for (int i=0;i<4;i++)
            cv::line(resultImg,vertices[i],vertices[(i+1)%4],cv::Scalar(0,255,0),3);

        goalFoundFlag=true;
    }
    else
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--more than one goal found:");

        // find biggest area
        static int maxArea;
        maxArea=128;
        for(auto item:frameRectPoly)
        {
            if(item.size.area()>maxArea)
            {
                maxArea=item.size.area();
                goalInView=item;
            }
        }

        // white ratio cal
        static int whiteCount,pixelCount;
        whiteCount=0;
        pixelCount=goalInView.size.area();

        for(int r=goalInView.boundingRect().y;r<=goalInView.boundingRect().y+goalInView.boundingRect().height;r++)
        {
            for(int c=goalInView.boundingRect().x;c<=goalInView.boundingRect().x+goalInView.boundingRect().width;c++)
            {
                if(filteredImg1.at<uchar>(r,c))
                    ++whiteCount;
            }
        }

        // RCLCPP_INFO(playerNode->get_logger(),"%lf,(%d,%d)",whiteCount/(double)pixelCount,goalInView.center.x,goalInView.center.y);

        if(whiteCount/(double)pixelCount>0.1)
        {
            goalFoundFlag=true;

            cv::circle(resultImg,goalInView.center,3,cv::Scalar(0,255,0),3);
            
            static cv::Point2f vertices[4];
            goalInView.points(vertices);
            for (int i=0;i<4;i++)
                cv::line(resultImg,vertices[i],vertices[(i+1)%4],cv::Scalar(0,255,0),3);

        }
        else
        {
            goalFoundFlag=false;
            
        }
        

    }

return resultImg;

}

//********************************************************************************************************************************//



//********************************************************************************************************************************//
// function implementation end
