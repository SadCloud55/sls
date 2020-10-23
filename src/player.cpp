#include "topics.hpp"
#include <common/srv/get_color.hpp>











// user define begin
//********************************************************************************************************************************//

enum headFollowTarget_type
{
    ball,
    goal,
    robot,
    none
}headFollowTarget;

enum findingFlag_type
{
    findingball,
    findinggoal,
    findingrobot,
    findingnone
}findingFlag;

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

// mean average filter weight
float mafWeight[5]={0.05,0.15,0.25,0.35,0.20};

float mafUpdate(float val,float* mafBuffer,int size);

#define SGN(x) ((x)>0?(1):((x)<0?(-1):(0)))
const float FIND_HEADANGLE_YAW_MAX=15.0;
const int FIND_GOAL_INTERVAL=400;
const int RID_DEFENSE_TIME=100;

//********************************************************************************************************************************//

// returns:processed image ; image:camera image ; playerNode:used to log data.
// updates:ballInView & ballFoundFlag ; goalInView & goalFoundFlag.
cv::Mat& imageProcess(cv::Mat& image,rclcpp::Node::SharedPtr playerNode);

// htask:used to log control head ; target:center point of tracking object ; distance:estimated distance
// target switched by:headFollowTarget.
void headFollow(common::msg::HeadTask& htask,common::msg::HeadAngles& headAngle);

void followBall(common::msg::HeadAngles& headAngle,common::msg::BodyTask& btask,rclcpp::Node::SharedPtr& playerNode);
void findBall(common::msg::HeadTask& htask,common::msg::BodyTask& btask,rclcpp::Node::SharedPtr& playerNode);
void yawControl(float& yawTarget,common::msg::BodyTask& btask,common::msg::ImuData& imuData,rclcpp::Node::SharedPtr& playerNode);
void passRobot(rclcpp::Node::SharedPtr& playerNode);
void findGoal(common::msg::HeadAngles& headAngle,common::msg::HeadTask& htask,common::msg::BodyTask& btask,common::msg::ImuData& imuData,rclcpp::Node::SharedPtr& playerNode);
void findRobot(common::msg::HeadAngles& headAngle,common::msg::HeadTask& htask,common::msg::BodyTask& btask,common::msg::ImuData& imuData,rclcpp::Node::SharedPtr& playerNode);
void followRobot(common::msg::HeadAngles& headAngle,common::msg::BodyTask& btask,rclcpp::Node::SharedPtr& playerNode);
void defenseFindBall(common::msg::HeadAngles& headAngle,common::msg::HeadTask& htask,common::msg::BodyTask& btask,rclcpp::Node::SharedPtr& playerNode);

//********************************************************************************************************************************//

cv::Vec3f ballInView;   // col,row,radius
bool ballFoundFlag;
float ballDistance;

cv::RotatedRect goalInView;
bool goalFoundFlag;

cv::RotatedRect robotInView;
bool robotFoundFlag;
int robotFoundTime=0;

int secondFlag=0;
bool knowGoalFlag=true;
bool antiAttack=false;
bool defenseFlag=false;
float yawTarget=0;
    
int imgRow=0,imgCol=0;
int myColorInd;

pid_type bodyFollow;

//********************************************************************************************************************************//
// user define end










enum RobotColor 
{
    COLOR_INVALID,
    COLOR_RED,
    COLOR_BLUE
};

RobotColor GetRobotColor(const std::string &name)
{
    if (name.find("red") != std::string::npos) 
    {
        return COLOR_RED;
    } 
    else if (name.find("blue") != std::string::npos) 
    {
        return COLOR_BLUE;
    } 
    else 
    {
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
        if (gameData.state == gameData.STATE_INIT)
        {
            // 每次开球时都会进入这里
            // 例如：RCLCPP_INFO(playerNode->get_logger(), "%d", gameData.blue_score);
            // INII状态时，1号机器人位于己方这边的中圈上，2号机器人位于己方的禁区线上
            initYaw=imuData.yaw; // 获取init状态时，机器人的朝向，此时的方向即为自己进攻的方向
            secondFlag=1;
            robotFoundTime=1;
            headFollowTarget=ball;
            findingFlag=findingnone;
            yawTarget=0;

            knowGoalFlag=true;
            antiAttack=false;
            defenseFlag=false;

            

            // 每次开球时都会进入这里
            // 例如：RCLCPP_INFO(playerNode->get_logger(), "%d", gameData.blue_score);

            ballInView={0,0,30};
            ballDistance=435;
            goalInView={cv::Point2f(0,0),cv::Size2f(0,0),0};

            if((!imgRow||!imgCol)&&!image.empty())
            {
                imgRow=image.rows;
                imgCol=image.cols;
                RCLCPP_INFO(playerNode->get_logger(),"%s--got image size:",argv[1]);
                RCLCPP_INFO(playerNode->get_logger(),"rows=%d,cols=%d",imgRow,imgCol);
            }

            myColorInd=myColor;

            bodyFollow.Kp=0.1;
            bodyFollow.Ki=0.005;
            bodyFollow.Kd=0;
            bodyFollow.sumClamp=100;
            bodyFollow.outClamp=100;
            bodyFollow.isEnabled=false;

            btask.step=0;
            htask.yaw=0.0;
            htask.pitch=45.0;

        }

        else
        {
            if(gameData.state==gameData.STATE_PLAY)
            {
                secondFlag++;
                robotFoundTime++;
            }



            // if(myId==1)
                if (!image.empty())
                    resImgPublisher->Publish(imageProcess(image,playerNode)); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看





            // 将机器人的当前朝向减去init时的朝向，即可得到机器人相对于初始位置的方向，
            // 这样可以保证自己不管是红色或者蓝色，其yaw都是在面朝对方球门时为0
            // 下面提供这种转换的方法，需要该转换的可以把注释符号去掉
            imuData.yaw=imuData.yaw-initYaw; 
            imuData.yaw=abs(imuData.yaw)<180?
                       imuData.yaw:
                       (imuData.yaw>0?imuData.yaw-360:imuData.yaw+360);

            while(headAngle.yaw>180)
                headAngle.yaw-=360;
            while(headAngle.yaw<-180)
                headAngle.yaw+=360;
            //headAngle更正





            // IMU数据里的yaw信息是统一的，属于绝对方向，但是红色机器人和蓝色机器人的进球方向是不一样的，
            // 假如红色的进球方向是0度，则蓝色的进球方向就是180度
            // 比赛时使用什么颜色的机器人是现场决定的，所以对于两种颜色的机器人，都需要考虑如何
            // 如果使用了上面的转换方法，则不需要再关心不同颜色机器人的方向问题
            if(gameData.mode==gameData.MODE_NORM)
            {
                if (myId == 1)
                {
                    if(!defenseFlag)
                    {
                        headFollow(htask,headAngle);
                        followBall(headAngle,btask,playerNode);
                        findBall(htask,btask,playerNode);
                        yawControl(yawTarget,btask,imuData,playerNode);
                        passRobot(playerNode);
                        findGoal(headAngle,htask,btask,imuData,playerNode);
                        // RCLCPP_INFO(playerNode->get_logger(),"btask.step=%f",btask.step);
                        // RCLCPP_INFO(playerNode->get_logger(),"yawTarget=%f",yawTarget);
                        // RCLCPP_INFO(playerNode->get_logger(),"robotFoundTime=%d",robotFoundTime);
                        // RCLCPP_INFO(playerNode->get_logger(),"secondFlag=%d",secondFlag);
                    }
                    else
                    {
                        headFollow(htask,headAngle);
                        findRobot(headAngle,htask,btask,imuData,playerNode);
                        followRobot(headAngle,btask,playerNode);
                        if(ballDistance<350)
                        {
                            headFollowTarget=ball;
                            defenseFlag=true;
                        }
                        // RCLCPP_INFO(playerNode->get_logger(),"DEFENSE!");
                    }
                    
                }
                else if (myId == 2)
                {
                    if(!defenseFlag)
                    {
                        headFollow(htask,headAngle);
                        // followBall(headAngle,btask,playerNode);
                        defenseFindBall(headAngle,htask,btask,playerNode);
                        RCLCPP_INFO(playerNode->get_logger(),"ball to player2:%f",ballDistance);
                        //defense
                    }
                }
            }
            else if (gameData.mode==gameData.MODE_KICK)
            {
                if (myId == 1)
                {
                    btask.type=btask.TASK_ACT;
                    btask.actname="left_kick";
                }
            }
        }
        // 复赛每个组使用两台机器人，需要根据不同的机器人id制定策略的队伍，可以利用myId进行区分
        // 机器人出界后，会将出界的机器人放回己方半场靠近中线的约0.5米处，此时该机器人会有30s的惩罚
        // 由于有两台机器人，每一天机器人重生的位置都是在己方固定的某一边，并不是从哪边出去就从哪边重生，
        // 具体哪一台从哪边重生，请自行运行仿真查看

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

        obj.output=
            obj.Kp*obj.curError+
            obj.Ki*obj.sumError+
            obj.Kd*(obj.curError-obj.lastError);

        obj.output=obj.output>obj.outClamp?obj.outClamp:obj.output;
        obj.output=obj.output<-obj.outClamp?-obj.outClamp:obj.output;
    }
    else
    {
        obj.output=0;
    }
}

float mafUpdate(float val,float* mafBuffer,int size=5)
{
    // last element in mafBuffer should be the most recent
    for(int i=1;i<size;i++)
        mafBuffer[i-1]=mafBuffer[i];
    mafBuffer[size-1]=val;

    static float mafVal;
    mafVal=0;
    for(int i=0;i<size;i++)
        mafVal+=mafBuffer[i]*mafWeight[i];

    return mafVal;
}

void headFollow(common::msg::HeadTask& htask,common::msg::HeadAngles& headAngle)
{
    static pid_type hFollow,vFollow;
    static bool initFlag=true;
    static cv::Point2f target;
    static float distance;

    if(initFlag)
    {
        initFlag=false;

        hFollow.Kp=0.15;
        hFollow.Ki=0.002;
        hFollow.Kd=0.08;
        hFollow.sumClamp=1000;
        hFollow.outClamp=100;
        hFollow.isEnabled=true;

        vFollow.Kp=0.12;
        vFollow.Ki=0.002;
        vFollow.Kd=0.08;
        vFollow.sumClamp=1000;
        vFollow.outClamp=100;
        vFollow.isEnabled=true;

    }

    switch(headFollowTarget)
    {
        case ball:
            target=cv::Point2f(ballInView[0],ballInView[1]);
            distance=ballDistance;
            break;

        case goal:
            target=goalInView.center;
            distance=2450;
            break;

        case robot:
            target=robotInView.center;
            distance=640;
            break;

        case none:
            hFollow.sumError=vFollow.sumError=0;
            return;

        default:
            break;
    }
    


    if(target.x>0)
    {
        hFollow.target=imgCol/2;



        hFollow.lastError=hFollow.curError;
        hFollow.curError=target.x-hFollow.target;

        hFollow.sumError+=hFollow.curError;
        hFollow.sumError=hFollow.sumError>hFollow.sumClamp?hFollow.sumClamp:hFollow.sumError;
        hFollow.sumError=hFollow.sumError<-hFollow.sumClamp?-hFollow.sumClamp:hFollow.sumError;

        hFollow.output=
            hFollow.Kp*SGN(hFollow.curError)*cv::fastAtan2(fabs(hFollow.curError),distance)+
            hFollow.Ki*hFollow.sumError+
            hFollow.Kd*(hFollow.curError-hFollow.lastError);

        hFollow.output=hFollow.output>hFollow.outClamp?hFollow.outClamp:hFollow.output;
        hFollow.output=hFollow.output<-hFollow.outClamp?-hFollow.outClamp:hFollow.output;



        htask.yaw-=hFollow.output;
        // htask.yaw=htask.yaw<0?htask.yaw+360:htask.yaw;
        // htask.yaw=htask.yaw>360?htask.yaw-360:htask.yaw;
    }
    
    if(target.y>0)
    {
        vFollow.target=imgRow/2;



        vFollow.lastError=vFollow.curError;
        vFollow.curError=target.y-vFollow.target;

        vFollow.sumError+=vFollow.curError;
        vFollow.sumError=vFollow.sumError>vFollow.sumClamp?vFollow.sumClamp:vFollow.sumError;
        vFollow.sumError=vFollow.sumError<-vFollow.sumClamp?-vFollow.sumClamp:vFollow.sumError;

        vFollow.output=
            vFollow.Kp*SGN(vFollow.curError)*cv::fastAtan2(fabs(vFollow.curError),distance)+
            vFollow.Ki*vFollow.sumError+
            vFollow.Kd*(vFollow.curError-vFollow.lastError);

        vFollow.output=vFollow.output>vFollow.outClamp?vFollow.outClamp:vFollow.output;
        vFollow.output=vFollow.output<-vFollow.outClamp?-vFollow.outClamp:vFollow.output;



        htask.pitch+=vFollow.output;
        htask.pitch=htask.pitch<0?0:htask.pitch;
        static float hPitchLim;
        hPitchLim=67.5+22.5*cos(headAngle.yaw*M_PI/90);
        htask.pitch=htask.pitch>hPitchLim?hPitchLim:htask.pitch;
    }

}

//********************************************************************************************************************************//

cv::Mat& imageProcess(cv::Mat& image,rclcpp::Node::SharedPtr playerNode)
{
    static cv::Mat grayImg,hsvImg,filteredImg1,filteredImg2,filteredImg3,resultImg;
    static std::vector<cv::Mat> hsvSplit;
    static std::vector<cv::Vec3f> circleSeq;
    static std::vector<std::vector<cv::Point>> frameContours,robotContours;
    static std::vector<cv::Vec4i> frameHierachy,robotHierachy;
    static bool trueBallFoundFlag;
    static int ballNotFoundElapse=0;


    cv::cvtColor(image,grayImg,cv::COLOR_BGR2GRAY);
    for(int r=0;r<imgRow;r++)
    {
        for(int c=0;c<imgCol;c++)
        {
            if(grayImg.at<uchar>(r,c)>=225)
                grayImg.at<uchar>(r,c)=0;
            if(grayImg.at<uchar>(r,c)>45&&grayImg.at<uchar>(r,c)<210)
                grayImg.at<uchar>(r,c)=(0.02)*(grayImg.at<uchar>(r,c)-127)+127;
        }
    }
    cv::medianBlur(grayImg,grayImg,3);
    cv::GaussianBlur(grayImg,grayImg,cv::Size(3,3),0,0);

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

    // anti robot color
    if(myColorInd==1)
        cv::inRange(hsvImg,cv::Scalar(-5,160,200),cv::Scalar(5,255,255),filteredImg3);//blue->white
    else if(myColorInd==2)
        cv::inRange(hsvImg,cv::Scalar(115,160,200),cv::Scalar(125,255,255),filteredImg3);//red->white
    else
        RCLCPP_INFO(playerNode->get_logger(),"--Invalid robot color");
    cv::medianBlur(filteredImg3,filteredImg3,3);
    // cv::GaussianBlur(filteredImg3,filteredImg3,cv::Size(3,3),0,0);
    // RCLCPP_INFO(playerNode->get_logger(),"----%d,%d,%d",hsvImg.at<cv::Vec3b>(0,0)[0],hsvImg.at<cv::Vec3b>(0,0)[1],hsvImg.at<cv::Vec3b>(0,0)[2]);


    
    // resultimg select
    // cv::cvtColor(grayImg,resultImg,cv::COLOR_GRAY2BGR);
    // cv::cvtColor(filteredImg1,resultImg,cv::COLOR_GRAY2BGR);
    // cv::cvtColor(filteredImg2,resultImg,cv::COLOR_GRAY2BGR);
    cv::cvtColor(filteredImg3,resultImg,cv::COLOR_GRAY2BGR);
    // resultImg=image;





    // anti robot detect
    cv::findContours(filteredImg3,robotContours,robotHierachy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point(-1,-1));
    // cv::drawContours(resultImg,robotContours,-1,cv::Scalar(0,255,255),1,8,robotHierachy);

    if(robotContours.empty())
    {
        robotFoundFlag=false;
    }
    else
    {

        // find biggest area
        static int maxArea;
        static double tmpArea;
        maxArea=(defenseFlag==true?64:1920);
        // (defenseFlag==true?64:1920)
        for(auto item:robotContours)
        {
            tmpArea=cv::contourArea(item);
            if(tmpArea>maxArea)
            {
                maxArea=tmpArea;
                robotInView=cv::minAreaRect(item);
            }
        }

        if(maxArea==(defenseFlag==true?64:1920))
        {
            robotFoundFlag=false;
        }
        else
        {
            robotFoundFlag=true;
        }
        
    }

    if(robotFoundFlag)
    {
        cv::circle(resultImg,robotInView.center,3,cv::Scalar(0,255,255),3);

        static cv::Point2f vertices[4];
        robotInView.points(vertices);
        for(int i=0;i<4;i++)
            cv::line(resultImg,vertices[i],vertices[(i+1)%4],cv::Scalar(0,255,255),3);

    }
    else
    {
        
    }
    




    // goal frame detect
    cv::findContours(filteredImg2,frameContours,frameHierachy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point(-1,-1));
    // cv::drawContours(resultImg,frameContours,-1,cv::Scalar(0,0,255),1,8,frameHierachy);

    std::vector<std::vector<cv::Point>> frameContoursPloy(frameContours.size());
    std::vector<cv::RotatedRect> frameRectPoly;

    for(size_t i=0; i<frameContours.size();i++)
    {
        cv::approxPolyDP(frameContours[i],frameContoursPloy[i],15,true);
        // for(auto item:frameContoursPloy[i])
        // {
        //     cv::circle(resultImg,item,1,cv::Scalar(0,255,0),1);
        // }
        // RCLCPP_INFO(playerNode->get_logger(),"--polysize=%d",frameContoursPloy[i].size());
        if(frameContoursPloy[i].size()<16)
            frameRectPoly.push_back(cv::minAreaRect(frameContoursPloy[i]));

    }

    if(frameRectPoly.empty())
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--goal not found");

        goalFoundFlag=false;
    }
    else
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--more than one goal found:");


        
        for(int i=0;i<frameContoursPloy.size();i++)
            for(int j=0;j<frameContoursPloy[i].size();j++)
                cv::line(resultImg,frameContoursPloy[i][j],frameContoursPloy[i][(j+1)%frameContoursPloy[i].size()],cv::Scalar(0,255,0),1);



        // white ratio cal
        static float maxWhiteRatio;
        maxWhiteRatio=0.1;

        for(auto item:frameRectPoly)
        {
            static int whiteCount,pixelCount;
            whiteCount=0;

            if(item.size.area()>=9200&&item.size.aspectRatio()<3&&item.size.aspectRatio()>0.33)
                pixelCount=item.size.area();
            else
                continue;
            
            for(int r=item.boundingRect().y;r<=item.boundingRect().y+item.boundingRect().height;r++)
            {
                for(int c=item.boundingRect().x;c<=item.boundingRect().x+item.boundingRect().width;c++)
                {
                    if(filteredImg1.at<uchar>(r,c)>10)
                        ++whiteCount;
                }
            }
            
            if(whiteCount/(double)pixelCount>maxWhiteRatio)
            {
                maxWhiteRatio=whiteCount/(double)pixelCount;
                goalInView=item;
            }

        }


        // RCLCPP_INFO(playerNode->get_logger(),"%f,%f",maxWhiteRatio,goalInView.size.area());

        if(maxWhiteRatio<=0.11||maxWhiteRatio>0.4)
        {
            goalFoundFlag=false;
        }
        else
        {
            goalFoundFlag=true;
        }
        

    }

    if(goalFoundFlag)
    {
        cv::circle(resultImg,goalInView.center,3,cv::Scalar(0,255,0),3);
            
        static cv::Point2f vertices[4];
        goalInView.points(vertices);
        for(int i=0;i<4;i++)
            cv::line(resultImg,vertices[i],vertices[(i+1)%4],cv::Scalar(0,255,0),3);

        // ballDistance=30*435/ballInView[2];
        // minRadius=ballInView[2]*0.75;
        // maxRadius=ballInView[2]*1.33;
    }
    else
    {
        // minRadius=1;
        // maxRadius=64;
    }



    

    // ball detect
    static int minRadius=1,maxRadius=64;
    static cv::Rect possBallRegion={0,0,511,511};
    static float radiusMafBuffer[5]={0};

    cv::HoughCircles(grayImg,circleSeq,cv::HOUGH_GRADIENT,1,128,64,25,minRadius,maxRadius);
    // cv::HoughCircles(grayImg,circleSeq,cv::HOUGH_GRADIENT,1.5,128,100,25,4,72);

    if(circleSeq.empty())
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--ball not found");
        trueBallFoundFlag=false;
    }
    else
    {
        // RCLCPP_INFO(playerNode->get_logger(),"--one or more ball found");

        static float maxWhiteRatio;
        maxWhiteRatio=0.5;

        for(auto item:circleSeq)
        {
            cv::circle(resultImg,cv::Point2f(item[0],item[1]),item[2],cv::Scalar(255,0,0));
            // RCLCPP_INFO(playerNode->get_logger(),"(%f,%f),%f",item[0],item[1],item[2]);

            // white ratio cal
            static int whiteCount,pixelCount;
            whiteCount=pixelCount=0;

            if(
                !possBallRegion.contains(cv::Point(item[0],item[1]))||
                (robotFoundFlag&&robotInView.boundingRect().contains(cv::Point(item[0],item[1])))||
                (goalFoundFlag&&goalInView.boundingRect().contains(cv::Point(item[0],item[1])))
            )
                continue;

            for(int r=item[1]-item[2];r<=item[1]+item[2];r++)
            {
                for(int c=item[0]-item[2];c<=item[0]+item[2];c++)
                {
                    if((pow(item[1]-r,2)+pow(item[0]-c,2))<=pow(item[2],2))
                    {
                        ++pixelCount;
                        if(filteredImg1.at<uchar>(r,c)>2)
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
            trueBallFoundFlag=false;
        }
        else
        {
            trueBallFoundFlag=true;
        }
        
    }

    ballInView[2]=mafUpdate(ballInView[2],radiusMafBuffer);

    if(trueBallFoundFlag)
    {
        ballNotFoundElapse=0;
         
        cv::circle(resultImg,cv::Point2f(ballInView[0],ballInView[1]),ballInView[2],cv::Scalar(255,0,0),3);

        ballDistance=30*435/ballInView[2];

        minRadius=ballInView[2]*0.75;
        maxRadius=ballInView[2]*1.33;

        static float mult=5;
        mult=3+(0.02)*sqrt(pow(ballInView[0]-imgCol/2,2)+pow(ballInView[1]-imgRow/2,2));
        possBallRegion=cv::Rect(ballInView[0]-ballInView[2]*mult,ballInView[1]-ballInView[2]*mult,ballInView[2]*2*mult,ballInView[2]*2*mult);

        // RCLCPP_INFO(playerNode->get_logger(),"%f",mult);
        cv::rectangle(resultImg,possBallRegion,cv::Scalar(255,0,192),2);
    }
    else
    {
        ballNotFoundElapse++;

        minRadius=1;
        maxRadius=64;

        possBallRegion=cv::Rect(0,0,imgCol,imgRow);
    }
    
    ballFoundFlag=trueBallFoundFlag||ballNotFoundElapse<1*10;





    
    return resultImg;

}

//********************************************************************************************************************************//

//跟球移动
void followBall(common::msg::HeadAngles& headAngle,common::msg::BodyTask& btask,rclcpp::Node::SharedPtr& playerNode)
{
    if(ballFoundFlag&&knowGoalFlag)
    {
        if(abs(headAngle.yaw)<8)
        {
            RCLCPP_INFO(playerNode->get_logger(),"-0-Going ahead,headAngle.yaw=%f",headAngle.yaw);
            btask.turn=0;
            btask.lateral=0;
            btask.step=1;
        }
        else if(abs(headAngle.yaw)<20)
        {
            RCLCPP_INFO(playerNode->get_logger(),"-1-Going ahead & tring to adjust,headAngle.yaw=%f",headAngle.yaw);
            btask.turn=0;
            btask.lateral=(0.001)*headAngle.yaw;
            btask.step=1;
        }
        else if(abs(headAngle.yaw)<90)
        {
            RCLCPP_INFO(playerNode->get_logger(),"-2-Tring to follow the ball,headAngle.yaw=%f",headAngle.yaw);
            btask.turn=0;
            btask.lateral=(0.5)*SGN(headAngle.yaw);
            btask.step=-0.01;
        }
        else
        {
            RCLCPP_INFO(playerNode->get_logger(),"-3-Stepping backwards,headAngle.yaw=%f",headAngle.yaw);
            btask.turn=0;
            btask.lateral=(0.05)*SGN(headAngle.yaw);
            btask.step=-1;
        }
    }
    else
    {
        return ;
    }
    
}

//摆头找球
void findBall(common::msg::HeadTask& htask,common::msg::BodyTask& btask,rclcpp::Node::SharedPtr& playerNode)
{
    static int ballNotFoundElapse=0;
    if(!ballFoundFlag&&headFollowTarget==ball)
    {
        RCLCPP_INFO(playerNode->get_logger(),"I am finding the ball!");
        btask.type=btask.TASK_ACT;
        // btask.step=0;
        // btask.lateral=0;
        // btask.turn=0;
        htask.yaw+=3;
        htask.pitch=20*sin(secondFlag*0.1)+40;
        ballNotFoundElapse++;
        findingFlag=findingball;
        if(ballNotFoundElapse>100)
        {
            findingFlag=findingnone;
            defenseFlag=true;
            ballNotFoundElapse=0;
        }
    }
    else
    {
        findingFlag=findingnone;
        btask.type=btask.TASK_WALK;
        ballNotFoundElapse=0;
        return ;
    }
    
}

//稳定方向
void yawControl(float& yawTarget,common::msg::BodyTask& btask,common::msg::ImuData& imuData,rclcpp::Node::SharedPtr& playerNode)
{
    if(abs(imuData.yaw-yawTarget)>10)
    {
        RCLCPP_INFO(playerNode->get_logger(),"imuData.yaw=%f,yawTarget=%f",imuData.yaw,yawTarget);
        btask.step=0;
        btask.lateral=0;
        btask.turn=-5*SGN(imuData.yaw-yawTarget);
    }
    else
    {
        return ;
    }
    
}

//带球过人
void passRobot(rclcpp::Node::SharedPtr& playerNode)
{
    if(ballFoundFlag&&robotFoundFlag&&(robotFoundTime>RID_DEFENSE_TIME||robotFoundTime==0)&&secondFlag>100&&knowGoalFlag==true&&ballDistance<300)
    //看到了球&&看到了机器人&&过了摆脱防守的时间&&过了开始过人的时间（避免一开始就转向）&&知道球门在哪（不至于找球门的时候看到机器人转向）&& 自己控球
    {
        RCLCPP_INFO(playerNode->get_logger(),"I meet anti robot");
        yawTarget+=SGN(robotInView.center.x-imgCol/2)*45;
        robotFoundTime=1;
        //可能需要更改，可能因为碰到对方机器人疯狂旋转
    }
    else
    {
        return ;
    }
    
}

//寻找球门
void findGoal(common::msg::HeadAngles& headAngle,common::msg::HeadTask& htask,common::msg::BodyTask& btask,common::msg::ImuData& imuData,rclcpp::Node::SharedPtr& playerNode)
{
    if(secondFlag%FIND_GOAL_INTERVAL==0)
    {
        knowGoalFlag=false;
        headFollowTarget=none;
    }
    if(!knowGoalFlag&&robotFoundTime>RID_DEFENSE_TIME&&findingFlag==findingnone)
    {
        RCLCPP_INFO(playerNode->get_logger(),"I am finding the goal!");
        RCLCPP_INFO(playerNode->get_logger(),"--%d",headFollowTarget);
        static int goalFoundElapse=0;
        static int delay=0;
        
        btask.type=btask.TASK_ACT;

        // btask.turn=0;
        // btask.step=0;
        // btask.lateral=0;
        htask.pitch=0;

        findingFlag=findinggoal;

        if(goalFoundFlag&&abs(imuData.yaw+headAngle.yaw)<90)
        {
            goalFoundElapse++;
            delay=0;

            if(goalFoundElapse==10)
            {
                headFollowTarget=goal;
            }
            else if(goalFoundElapse>20)
            {
                yawTarget=imuData.yaw+headAngle.yaw;
                headFollowTarget=ball;
                knowGoalFlag=true;
                goalFoundElapse=0;
                btask.type=btask.TASK_WALK;
                findingFlag=findingnone;
            }

        }
        else
        {
            delay++;
            if(delay>=5)
            {
                htask.yaw+=-8*SGN(imuData.yaw);
            }
            // goalFoundElapse=0;   
        }
        
    }
    else
    {
        return ;
    }
}

//寻找机器人
void findRobot(common::msg::HeadAngles& headAngle,common::msg::HeadTask& htask,common::msg::BodyTask& btask,common::msg::ImuData& imuData,rclcpp::Node::SharedPtr& playerNode)
{
    if(findingFlag==findingnone)
    {
        headFollowTarget=none;
        findingFlag=findingball;
    }
    
    static int robotFoundElapse=0;
    static int delay=0;
        
    btask.type=btask.TASK_ACT;

    // btask.turn=0;
    // btask.step=0;
    // btask.lateral=0;
    htask.pitch=0;
    if(robotFoundFlag&&abs(imuData.yaw+headAngle.yaw)>90)
    {
        robotFoundElapse++;
        delay=0;
        if(robotFoundElapse==10)
        {
            headFollowTarget=robot;
        }
        else if(robotFoundElapse>20)
        {
            yawTarget=imuData.yaw+headAngle.yaw;
            robotFoundElapse=0;
            btask.type=btask.TASK_WALK;
            findingFlag=findingnone;
        }
    }
    else
    {
        delay++;
        if(delay>=5)
        {
            htask.yaw+=-8*SGN(imuData.yaw);
        }
        // goalFoundElapse=0;   
    }
}

//跟随机器人
void followRobot(common::msg::HeadAngles& headAngle,common::msg::BodyTask& btask,rclcpp::Node::SharedPtr& playerNode)
{
    if(robotFoundFlag)
    {
        if(abs(headAngle.yaw)<7.5)
        {
            RCLCPP_INFO(playerNode->get_logger(),"-0-Going ahead,headAngle.yaw=%f",headAngle.yaw);
            btask.turn=0;
            btask.lateral=0;
            btask.step=1;
        }
        else if(abs(headAngle.yaw)<15)
        {
            RCLCPP_INFO(playerNode->get_logger(),"-1-Going ahead & tring to adjust,headAngle.yaw=%f",headAngle.yaw);
            btask.turn=0;
            btask.lateral=(0.001)*headAngle.yaw;
            btask.step=1;
        }
        else if(abs(headAngle.yaw)<90)
        {
            RCLCPP_INFO(playerNode->get_logger(),"-2-Tring to follow the ball,headAngle.yaw=%f",headAngle.yaw);
            btask.turn=0;
            btask.lateral=(0.5)*SGN(headAngle.yaw);
            btask.step=-0.01;
        }
        else
        {
            RCLCPP_INFO(playerNode->get_logger(),"-3-Stepping backwards,headAngle.yaw=%f",headAngle.yaw);
            btask.turn=0;
            btask.lateral=(0.05)*SGN(headAngle.yaw);
            btask.step=-1;
        }
    }
    else
    {
        return ;
    }
}

//防守机器人找球并拦截
void defenseFindBall(common::msg::HeadAngles& headAngle,common::msg::HeadTask& htask,common::msg::BodyTask& btask,rclcpp::Node::SharedPtr& playerNode)
{
    btask.type=btask.TASK_ACT;
    headFollowTarget=none;

    if(!defenseFlag)
    {
        htask.yaw=45*sin(secondFlag*0.005);
        htask.pitch=30;
    }

    if(ballFoundFlag&&ballDistance<600)
    {
        headFollowTarget=ball;
        btask.type=btask.TASK_WALK;
        defenseFlag=true;
        RCLCPP_INFO(playerNode->get_logger(),"They are Approaching!");
        followBall(headAngle,btask,playerNode);
    }
    else
    {
        headFollowTarget=none;
        defenseFlag=false;
        return ;
    }
}

//********************************************************************************************************************************//
// function implementation end
