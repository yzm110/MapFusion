#include "CrossPlanning.h"

namespace CommuteMode {
namespace MapFusion {


void CrossPlanning(const std::vector<float> &Lane_1_x,const std::vector<float> &Lane_1_y,const std::vector<float> &Lane_2_x,const std::vector<float> &Lane_2_y,
 std::vector<float> &Lane_out_x, std::vector<float> &Lane_out_y)
 {

            
            ZPLOTXYF("referLatPathGen",".black5",Lane_1_x,Lane_1_y);
            ZPLOTXYF("referLatPathGen",".black5",Lane_2_x,Lane_2_y);

    Eigen::Vector2d Lane_1_Start(Lane_1_x[Lane_1_x.size()-2], Lane_1_y[Lane_1_y.size()-2]);
    Eigen::Vector2d Lane_1_End(Lane_1_x[Lane_1_x.size()-1], Lane_1_y[Lane_1_y.size()-1]);
    Eigen::Vector2d Lane_2_Start(Lane_2_x[1], Lane_2_y[1]);
    Eigen::Vector2d Lane_2_End(Lane_2_x[0], Lane_2_y[0]);

    //**********************************************计算交点*************************************************
    Eigen::Vector2d intersectionPoint=findIntersection(Lane_1_Start,Lane_1_End,Lane_2_Start,Lane_2_End);

            //画图
            // std::vector<float> PlotinterPointX;
            // std::vector<float> PlotinterPointY;
            // PlotinterPointX.push_back(intersectionPoint[0]);
            // PlotinterPointY.push_back(intersectionPoint[1]);           
            // ZPLOTXYF("referLatPathGen",".black5",PlotinterPointX,PlotinterPointY);
            //VLOG(1) <<"intersectionPointXY:  "<< intersectionPoint[0] << "  "<<intersectionPoint[1]<<std::endl;
            //

    //**********************************************计算轨迹和交点之间的延长点*************************************************        
    //根据起点终点线性插值
    //定义两个向量，一个是X坐标，一个是Y坐标
    std::vector<float> intersectionX;
    intersectionX.push_back(Lane_1_x[Lane_1_x.size()-1]);
    intersectionX.push_back(intersectionPoint[0]);

    std::vector<float> intersectionY;
    intersectionY.push_back(Lane_1_y[Lane_1_y.size()-1]);
    intersectionY.push_back(intersectionPoint[1]);
    resampleTrajectory(intersectionX,intersectionY,5);
    std::vector<Eigen::Vector2d> Lane_1_interSPoint;
    for(int i=0;i<intersectionX.size();i++)
    {
        Eigen::Vector2d p(intersectionX[i],intersectionY[i]);
        Lane_1_interSPoint.push_back(p);
    }
    intersectionX.clear();
    intersectionX.push_back(Lane_2_x[0]);
    intersectionX.push_back(intersectionPoint[0]);
    intersectionY.clear();
    intersectionY.push_back(Lane_2_y[0]);
    intersectionY.push_back(intersectionPoint[1]);
    resampleTrajectory(intersectionX,intersectionY,5);
    std::vector<Eigen::Vector2d> Lane_2_interSPoint;
    for(int i=0;i<intersectionX.size();i++)
    {
        Eigen::Vector2d p(intersectionX[i],intersectionY[i]);
        Lane_2_interSPoint.push_back(p);
    }
            //画图
            // std::vector<float> Plot_2_X;
            // std::vector<float> Plot_2_Y;
            // for(int i=0;i<Lane_1_interSPoint.size();i++)
            // {
            //     Plot_2_X.push_back(Lane_1_interSPoint[i][0]);
            //     Plot_2_Y.push_back(Lane_1_interSPoint[i][1]);           
            //     //VLOG(1) <<"XY:  "<< Lane_1_interSPoint[i][0] << "  "<<Lane_1_interSPoint[i][1]<<std::endl;
            // }
            // for(int i=0;i<Lane_2_interSPoint.size();i++)
            // {
            //     Plot_2_X.push_back(Lane_2_interSPoint[i][0]);
            //     Plot_2_Y.push_back(Lane_2_interSPoint[i][1]);           
            //     //VLOG(1) <<"XY:  "<< Lane_1_interSPoint[i][0] << "  "<<Lane_1_interSPoint[i][1]<<std::endl;
            // }
            // ZPLOTXYF("referLatPathGen",".red5",Plot_2_X,Plot_2_Y);
            //画图

    //**********************************************根据两条轨迹的延长点两两生成贝塞尔曲线************************************************* 
    //两两排列生成贝塞尔曲线，
    //新建一个向量，里面包含所有可能的贝塞尔曲线
    std::vector < std::vector<Eigen::Vector2d> > All_curve; 
    std::vector<int> curve_startIdx;
    std::vector<int> curve_endIdx;

    for(int i=0;i<Lane_1_interSPoint.size()-1;i++) //最后一个交点删除
    {
        for(int j=0;j<Lane_2_interSPoint.size()-1;j++)
        {
            Eigen::Vector2d p0(Lane_1_interSPoint[i][0],Lane_1_interSPoint[i][1]);
            Eigen::Vector2d p3(Lane_2_interSPoint[j][0],Lane_2_interSPoint[j][1]);
            Eigen::Vector2d t1(Lane_1_interSPoint[Lane_1_interSPoint.size()-1][0]-Lane_1_interSPoint[0][0],Lane_1_interSPoint[Lane_1_interSPoint.size()-1][1]-Lane_1_interSPoint[0][1]);
            Eigen::Vector2d t2(Lane_2_interSPoint[0][0]-Lane_2_interSPoint[Lane_2_interSPoint.size()-1][0],Lane_2_interSPoint[0][1]-Lane_2_interSPoint[Lane_2_interSPoint.size()-1][1]);
            std::vector<Eigen::Vector2d> curve=cubicBezierDoubleTangent(p0,p3,t1,t2);
            All_curve.push_back(curve);
            curve_startIdx.push_back(i);
            curve_endIdx.push_back(j);
        }

    }
// //将所有轨迹点画出来，画太多会卡住，已经验证过，暂时注释
//             std::vector<std::vector<float>> Plot_3_X;
//             std::vector<std::vector<float>> Plot_3_Y;

//             for(int c=0;c<All_curve.size();c++)
//             {
                        
//                         std::vector<float> Plot_3_X_com;
//                         std::vector<float> Plot_3_Y_com;
//                         for(int i=0;i<All_curve[c].size();i++)
//                         {
//                             Plot_3_X_com.push_back(All_curve[c][i][0]);
//                             Plot_3_Y_com.push_back(All_curve[c][i][1]);  
                            
                            
//                         }
//                         Plot_3_X.push_back(Plot_3_X_com);   
//                         Plot_3_Y.push_back(Plot_3_Y_com);  
//             }
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[10],Plot_3_Y[10]);  
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[11],Plot_3_Y[11]);
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[12],Plot_3_Y[12]);
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[13],Plot_3_Y[13]);
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[14],Plot_3_Y[14]);
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[15],Plot_3_Y[15]);  

    //**********************************************通过硬约束删除风险轨迹************************************************* 
    //通过硬约束删除风险轨迹
    float k_curvetureLimit=0.5f;
    //定义一个二维向量，表示每个轨迹的最大曲率，及删除轨迹的索引,向量第一个里面表示曲率较大的轨迹点存放的位置，第二个里面是每个轨迹点的最大曲率
    std::vector<std::vector<float>> curveLimit=curvatureLimit(All_curve,k_curvetureLimit);
                // //打印计算出的曲率
                // for(int i=0;i<curveLimit[0].size();i++)
                // {
                //     VLOG(1) <<"exist_id:  "<< curveLimit[0][i]<<std::endl;
                //     VLOG(1) <<"K:  "<< curveLimit[1][i]<<std::endl;
                // }

    //**********************************************通过软约束删除风险轨迹************************************************* 
    //通过软约束删除风险轨迹

    //****************************计算超调
    Eigen::Vector2d line1_vec=Lane_1_interSPoint[0]-Lane_1_interSPoint[Lane_1_interSPoint.size()-1];
    Eigen::Vector2d line2_vec=Lane_2_interSPoint[0]-Lane_2_interSPoint[Lane_1_interSPoint.size()-1];
    std::vector<float> overshootVec;
    std::vector<float> overshoot_id(All_curve.size(),0);
    
    //得到每个轨迹点最大的overshoot情况
    for (int i=0;i<All_curve.size();i++)
    {
        if(curveLimit[0][i]==0)
        {
            overshootVec.push_back(0);
            continue;
        }
        float overshoot=0.0f;
        
        for (int j=1;j<All_curve[i].size()-1;j++)
        {
            Eigen::Vector2d point=All_curve[i][j];
            int region= determinePointRegion(intersectionPoint, line1_vec, line2_vec, point);
            Eigen::Vector2d line_point1;
            Eigen::Vector2d line_point2;
            float distance=0;
            
            switch (region)
            {
                case 1:
                            line_point1 = Lane_1_interSPoint[0];
                            line_point2 = Lane_1_interSPoint[Lane_1_interSPoint.size()-1];
                            distance = pointToLineDistance(point, line_point1, line_point2);
                            overshoot = std::max(overshoot,distance);
                            break;
                case 3:
                            line_point1 = Lane_2_interSPoint[0];
                            line_point2 = Lane_2_interSPoint[Lane_2_interSPoint.size()-1];
                            distance = pointToLineDistance(point, line_point1, line_point2);
                            overshoot = std::max(overshoot,distance);
                            break;
                
            }
            
        }
        overshootVec.push_back(overshoot); 
    }
    
    //把大于0的overshoot位置标记出来
    for(int i=0;i<All_curve.size();i++)
    {
        if(overshootVec[i]>0)
        {
            overshoot_id[i]=1;
        }
        else
        {
            overshoot_id[i]=0;
        }
    }
                // VLOG(1) <<"start:  "<<std::endl;
                //     //打印计算出的超调
                // for(int i=0;i<overshoot_id.size();i++)
                // {
                //     VLOG(1) <<"exist_id:  "<< overshootVec[i]<<std::endl;
                //     VLOG(1) <<"K:  "<< overshoot_id[i]<<std::endl;
                // }

// //画图看一下删了哪些线段

//             std::vector<std::vector<float>> Plot_3_X;
//             std::vector<std::vector<float>> Plot_3_Y;

//             for(int c=0;c<All_curve.size();c++)
//             {
//                         if(overshoot_id[c]==1||curveLimit[0][c]==0)
//                         {
//                             VLOG(1) <<"start:  "<<c<<std::endl;
//                         std::vector<float> Plot_3_X_com;
//                         std::vector<float> Plot_3_Y_com;
//                         for(int i=0;i<All_curve[c].size();i++)
//                         {
//                             Plot_3_X_com.push_back(All_curve[c][i][0]);
//                             Plot_3_Y_com.push_back(All_curve[c][i][1]);  
                            
                            
//                         }
//                         Plot_3_X.push_back(Plot_3_X_com);   
//                         Plot_3_Y.push_back(Plot_3_Y_com);  
//                         }
//             }
//             // for(int j=0;j<Plot_3_X.size();j++)
//             // {
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[0],Plot_3_Y[0]);  
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[1],Plot_3_Y[1]); 
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[2],Plot_3_Y[2]);
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[3],Plot_3_Y[3]);  
//             ZPLOTXYF("referLatPathGen",".red2",Plot_3_X[4],Plot_3_Y[4]);     

//****************************计算起点碰撞风险
    Eigen::Vector2d point=Lane_1_interSPoint[0];
    std::vector<float> start_distance(All_curve.size(),0);
    for(int i=0;i<All_curve.size();i++)
    {
        Eigen::Vector2d start=All_curve[i][0];
        start_distance[i]=std::sqrt(pow((point[0]-start[0]),2)+pow((point[1]-start[1]),2));
    }
//****************************计算终点碰撞风险
    point=Lane_2_interSPoint[0];
    std::vector<float> termin_distance(All_curve.size(),0);
    for(int i=0;i<All_curve.size();i++)
    {
        Eigen::Vector2d termin=All_curve[i][All_curve[i].size()-1];
        termin_distance[i]=std::sqrt(pow((point[0]-termin[0]),2)+pow((point[1]-termin[1]),2));
    }
//****************************设计损失函数筛选最优曲线
    //***********计算曲率项,算法里面是常值？？？？
    float c_curv=1.0f;
    std::vector<float> cost_curv(All_curve.size(),0);

    //**********计算超调项。
    float c_overshoot=1.0f;
    float k_overshootlmt=0.1f;
    std::vector<float> cost_overshoot(All_curve.size(),0);

    //**********计算起点边界碰撞风险项。
    float c_start=1.0f;
    std::vector<float> startDisTab_X={0,60};
    std::vector<float> startDisTab_Y={1,0};
    std::vector<float> cost_start(All_curve.size(),0);

    //**********计算终点边界碰撞风险项。
    float c_termin=1.0f;
    std::vector<float> terminDisTab_X={0,60};
    std::vector<float> terminDisTab_Y={1,0};
    std::vector<float> cost_termin(All_curve.size(),0);

    //**********计算汇总损失函数。
    std::vector<float> cost_fcn(All_curve.size(),0);

    for(int i=0;i<All_curve.size();i++)
    {
        //***********计算曲率项,
        cost_curv[i]=curveLimit[1][i]*curveLimit[1][i];
    
        //**********计算超调项。
        cost_overshoot[i] = c_overshoot*pow((overshootVec[i]-k_overshootlmt),2);
    
        //**********计算起点边界碰撞风险项。
        cost_start[i]=c_start*line_looktable(startDisTab_X,startDisTab_Y,start_distance[i]);
    
        //**********计算终点边界碰撞风险项。
        cost_termin[i]=c_termin*line_looktable(terminDisTab_X,terminDisTab_Y,termin_distance[i]);

        //**********计算汇总损失函数。
        cost_fcn[i]=cost_curv[i]+cost_overshoot[i]+cost_start[i]+cost_termin[i];
    }

    //**********求解汇总损失函数最小值对应的曲线。
    float min_cost=cost_fcn[0];
    int min_index=0;
    for(int i=0;i<All_curve.size();i++)
    {
        if(overshoot_id[i]==0&&curveLimit[0][i]==1)
        {
            if(min_cost<=cost_fcn[i])
            {
                min_cost=cost_fcn[i];
                min_index=i;
            }
        }
    }
    //画出最终的轨迹

            // std::vector<float> Plot_3_X_com;
            // std::vector<float> Plot_3_Y_com;
            // for(int i=0;i<All_curve[min_index].size();i++)
            // {
            //     Plot_3_X_com.push_back(All_curve[min_index][i][0]);
            //     Plot_3_Y_com.push_back(All_curve[min_index][i][1]);      
            // }
            // ZPLOTXYF("referLatPathGen",".red2",Plot_3_X_com,Plot_3_Y_com);  


 
    //**********************************************参考线拼接************************************************* 
    //***********************合并三段轨迹点
    int interindex_Lane_1=curve_startIdx[min_index];
    int interindex_Lane_2=curve_endIdx[min_index];

    // VLOG(1) <<"interindex_Lane_1:  "<< interindex_Lane_1 <<std::endl;
    // VLOG(1) <<"interindex_Lane_1:  "<< interindex_Lane_2 <<std::endl;
    std::vector<Eigen::Vector2d> traj_1;
    std::vector<Eigen::Vector2d> traj_2;
    std::vector<Eigen::Vector2d> traj_3;
    if(interindex_Lane_1>0)
    {
        for(int i=0;i<interindex_Lane_1+1;i++)
        {
            traj_1.push_back(Lane_1_interSPoint[i]);
        }
    }
    else
    {
        traj_1.push_back(Lane_1_interSPoint[0]);
    }
    traj_2=All_curve[min_index];
    if(interindex_Lane_2>0)
    {
        for(int i=interindex_Lane_2;i>=0;i--)
        {
            traj_3.push_back(Lane_2_interSPoint[i]);
        }
    }
    else
    {
        traj_3.push_back(Lane_2_interSPoint[0]);
    }
    // VLOG(1) <<"traj_1.size:  "<< traj_1.size() <<std::endl;
    // VLOG(1) <<"traj_2.size:  "<< traj_2.size() <<std::endl;
    // VLOG(1) <<"traj_3.size:  "<< traj_3.size() <<std::endl;
    std::vector<Eigen::Vector2d> merged=mergeXYTraj(traj_1,traj_2,traj_3);

    //***********************等间距插值得到完整路口参考点
    int numPoints=30; //插值点的个数
    //计算得到X坐标

    std::vector<float> traj_x;
    std::vector<float> traj_y;
    std::vector<float> cumDist;

    for(int i=0;i<merged.size();i++)
    {
        traj_x.push_back(merged[i][0]);
        traj_y.push_back(merged[i][1]);
        if(i==0)
        {
            cumDist.push_back(0);
        }
        else
        {
            cumDist.push_back(cumDist[i-1]+std::sqrt(pow(traj_x[i]-traj_x[i-1],2)+pow(traj_y[i]-traj_y[i-1],2)));
        }
    
    }
    float totalDist=cumDist[cumDist.size()-1];

    std::vector<float> evenDist={0,totalDist};
    interpoint(evenDist,numPoints);


   

    TjaLinearInterpolation(cumDist,traj_x,evenDist,Lane_out_x);  //超出截断

    
    TjaLinearInterpolation(cumDist,traj_y,evenDist,Lane_out_y);  //超出截断

        //画图，最后生成的轨迹
        // ZPLOTXYF("referLatPathGen",".blue4",Lane_out_x,Lane_out_y);  
        // for(int i=0;i<traj_x.size();i++)
        // {
        //     VLOG(1) <<"traj_x:  "<< traj_x[i] <<std::endl;
        // }
        // for(int i=0;i<traj_y.size();i++)
        // {
        //     VLOG(1) <<"traj_y:  "<< traj_y[i] <<std::endl;
        // }

        // for(int i=0;i<merged_X_NEW.size();i++)
        // {
        //     VLOG(1) <<"merged_X_NEW:  "<< merged_X_NEW[i] <<std::endl;
        // }
        // for(int i=0;i<merged_Y_NEW.size();i++)
        // {
        //     VLOG(1) <<"merged_Y_NEW:  "<< merged_Y_NEW[i] <<std::endl;
        // }

        // for(int i=0;i<evenDist.size();i++)
        // {
        //     VLOG(1) <<"evenDist:  "<< evenDist[i] <<std::endl;
        // }       



 }


 Eigen::Vector2d findIntersection(const Eigen::Vector2d &Lane_1_Start,const Eigen::Vector2d &Lane_1_End,const Eigen::Vector2d &Lane_2_Start,const Eigen::Vector2d &Lane_2_End)
 {
    Eigen::Vector2d intersectionPoint(0.0f, 0.0f);
    //计算方向向量
    Eigen::Vector2d dir1=Lane_1_End-Lane_1_Start;
    Eigen::Vector2d dir2=Lane_2_End-Lane_2_Start;
    //创建系数矩阵，原base中没有人使用过矩阵，未防止系统不兼容，使用vector进行手动计算
    //matlab中定义，A=[-dir1(2),dir1(1);-dir2(2),dir2(1)]
    //创建常数向量
    //matlab中定义， b = [-dir1(2)*line1_start(1) + dir1(1)*line1_start(2);
     //                  -dir2(2)*line2_start(1) + dir2(1)*line2_start(2)];
     //A\b=A^-1*b，手动对A取逆后和b相乘即可
    Eigen::Vector2d b(-dir1[1]*Lane_1_Start[0] + dir1[0]*Lane_1_Start[1],-dir2[1]*Lane_2_Start[0] + dir2[0]*Lane_2_Start[1]);
    //对A求逆，
    if((-dir1[1]*dir2[0]+dir1[0]*dir2[1])!=0)
    {
        float factor=1/(-dir1[1]*dir2[0]+dir1[0]*dir2[1]);

        intersectionPoint[0]=factor*(dir2[0]*b[0]-dir1[0]*b[1]);

        intersectionPoint[1]=factor*(dir2[1]*b[0]-dir1[1]*b[1]);
        return intersectionPoint;
    }
    else
    {
        return intersectionPoint;
    }

 } 

void resampleTrajectory(std::vector<float> &X,std::vector<float> &Y,int numPoints)
{
    if(numPoints>1)
    {
        interpoint(X,numPoints);
        interpoint(Y,numPoints);
    }
}

void interpoint(std::vector<float> &X,int numPoints)
{
    if(numPoints>1)
    {
        float startpoint =  X[0];
        float endpoint =  X[X.size()-1];
        
        float delta=(endpoint-startpoint)/(numPoints-1);
        X.clear();
        for(int i=0;i<numPoints;i++)
        {
            if(i==0)
            {
                X.push_back(startpoint);
            }
            else if(i==numPoints-1)
            {
                X.push_back(endpoint);
            }
            else
            {
                X.push_back(startpoint+delta*i);
            }
            //VLOG(1) <<"interPointXX:  "<< X[i] <<std::endl;
        }
    }
}

std::vector<Eigen::Vector2d> cubicBezierDoubleTangent(const Eigen::Vector2d &P0,const Eigen::Vector2d &P3,const Eigen::Vector2d &t1,const Eigen::Vector2d &t2)
{
    Eigen::Vector2d Unit_t1=t1/t1.norm();
    Eigen::Vector2d Unit_t2=t2/t2.norm();
    Eigen::Vector2d dist_P3_P0=P3-P0;
    float dist=dist_P3_P0.norm()/3;
    Eigen::Vector2d P1=P0+dist*Unit_t1;
    Eigen::Vector2d P2=P3-dist*Unit_t2;

    std::vector<float> t_index(51,0);

    for(int i=1;i<51;i++)
    {
        t_index[i]=t_index[i-1]+0.02f;  
    }

    std::vector<Eigen::Vector2d> curve(51);
    for(int j=0;j<51;j++)
    {
        curve[j]= ((1-t_index[j])*(1-t_index[j])*(1-t_index[j]) * P0) + (3*(1-t_index[j])*(1-t_index[j]) * t_index[j] * P1) + (3*(1-t_index[j]) * t_index[j]*t_index[j] * P2) + (t_index[j]*t_index[j]*t_index[j] * P3);
    }

    return curve;
}

//三点计算曲率
float calculateCurvatureThreePoint(const std::vector<float> x,const std::vector<float> y)
{
    int n=x.size();
    std::vector<float> curvature1(n,0);
    std::vector<float> radius1(n,0);
    
    for(int i=1;i<n-1;i++)
    {
        //获取三个连续点
        float x1=x[i-1];
        float y1=y[i-1];
        float x2=x[i];
        float y2=y[i];
        float x3=x[i+1];
        float y3=y[i+1];
        //计算三点之间的距离
        float a=std::sqrt(pow((x2-x1),2)+pow((y2-y1),2));
        float b=std::sqrt(pow((x3-x2),2)+pow((y3-y2),2));
        float c=std::sqrt(pow((x3-x1),2)+pow((y3-y1),2));
        //使用海伦公式计算三角形面积
        float s=(a+b+c)/2;
        float Area=std::sqrt(s*(s-a)*(s-b)*(s-c));
        if (Area>0)
        {
            curvature1[i]=4*Area/(a*b*c);
            radius1[i]=1/curvature1[i];
        }
        else
        {
            curvature1[i]=0.0f;
            radius1[i]=100000.0f;
        }
        
    }
    //处理端点
        curvature1[0] = curvature1[1];
        curvature1[n-1] = curvature1[n-2];
        radius1[0] = radius1[1];
        radius1[n-1] = radius1[n-2];

        auto result = std::max_element(curvature1.begin(),curvature1.end());
        float curv_max=*result;
        return curv_max;
}

std::vector<std::vector<float>> curvatureLimit(const std::vector < std::vector<Eigen::Vector2d> > All_curve,float k_curvatureLimit)
{
    int n=All_curve.size();
    std::vector<float> k(n,0);
    std::vector<float> exist_id(n,0);
    std::vector<std::vector<float>> output={exist_id,k};

//计算每条轨迹的曲率最大值
    for(int c=0;c<n;c++)
    {
        std::vector<float> X_com;
        std::vector<float> Y_com;
        for(int i=0;i<All_curve[c].size();i++)
        {
            X_com.push_back(All_curve[c][i][0]);
            Y_com.push_back(All_curve[c][i][1]);  
        } 
        k[c]=calculateCurvatureThreePoint(X_com,Y_com);
    }
//和限制曲率相比，将曲率大的位置挑出来    
    for(int i=0;i<n;i++)
    {
        if(k[i]<k_curvatureLimit)
        {
            exist_id[i]=1;
        }
        else
        {
            exist_id[i]=0;
        }
    }
    return output={exist_id,k};

}

int determinePointRegion(const Eigen::Vector2d intersection, const Eigen::Vector2d line1_vec, const Eigen::Vector2d line2_vec, const Eigen::Vector2d point)
{
    // % 输入:
    // %   intersection: 两直线交点坐标 [x, y]
    // %   line1_vec: 第一条直线的方向向量 [dx1, dy1]
    // %   line2_vec: 第二条直线的方向向量 [dx2, dy2]
    // %   point: 待判断点的坐标 [x, y]
    // % 输出:
    // %   region: 点所在的区域编号 (1,2,3,4)

    //计算点相对于交点的向量
    int region=0;
    
    Eigen::Vector2d point_vec = point - intersection;

    // 归一化方向向量
    Eigen::Vector2d line1_Unitvec = line1_vec / line1_vec.norm();
    Eigen::Vector2d line2_Unitvec = line2_vec / line2_vec.norm();

    // 计算叉积判断点在直线的哪一侧
    float cross1 = line1_vec[0]*point_vec[1] - line1_vec[1]*point_vec[0];
    float cross2 = line2_vec[0]*point_vec[1] - line2_vec[1]*point_vec[0];

    // 根据叉积判断区域
    if (cross1 >= 0 && cross2 >= 0)
    {
        region = 1;
    }
    else if (cross1 < 0 && cross2 >= 0)
    {
        region = 2;
    }
    else if (cross1 < 0 && cross2 < 0)
    {
        region = 3;
    }
    else
    {
        region = 4;
    }
        return region;
}

//计算点到线的距离，也可以返回垂足
float pointToLineDistance(const Eigen::Vector2d point, const Eigen::Vector2d line_point1, const Eigen::Vector2d line_point2)
 {   // % 计算点到直线的距离和垂足
    // % 输入:
    // %   point: 待计算点的坐标 [x, y]
    // %   line_point1: 直线上的第一个点 [x1, y1]
    // %   line_point2: 直线上的第二个点 [x2, y2]
    // % 输出:
    // %   distance: 点到直线的距离
    // %   foot_point: 垂足坐标 [x, y]

    // 方法1：使用向量叉积
    Eigen::Vector2d line_vec = line_point2 - line_point1;  // 直线方向向量
    Eigen::Vector2d point_vec = point - line_point1;       // 点到直线起点的向量

    // 计算距离
    float distance = std::fabs(line_vec[0]*point_vec[1] - line_vec[1]*point_vec[0]) / line_vec.norm();
    return distance;

    // % 计算垂足
    // % 使用向量投影公式
    // t = dot(point_vec, line_vec) / dot(line_vec, line_vec);
    // foot_point = line_point1 + t * line_vec;

}

//
float line_looktable(const std::vector<float> &X,const std::vector<float> &Y,float inputX)
{
    float outputY=0.0f;
    int n = X.size();
    if (n>1)
    {
        for(int i=0;i<n-1;i++)
        {
            if(inputX<=X[0])
            {
            outputY=Y[0];
            }
            else if(inputX >= X[n-1])
            {
            outputY=Y[n-1];
            }
            else if(X[i]<=inputX&& inputX<= X[i+1])
            {
                     
                float t = safeDivide((inputX - X[i]) , (X[i+1] - X[i]),1e-6);
                outputY=((1-t) * Y[i] + t * Y[i + 1]);
                
            }

        }
    }
    return outputY;
}

std::vector<Eigen::Vector2d> mergeXYTraj(std::vector<Eigen::Vector2d> traj_1,std::vector<Eigen::Vector2d> traj_2,std::vector<Eigen::Vector2d> traj_3)
{
    //检查连续性，暂时不需要
    //移除重复点
    std::vector<Eigen::Vector2d> mergeTraj;
    for(int i=0;i<traj_1.size();i++)
    {
        mergeTraj.push_back(traj_1[i]);
    }
    for(int i=1;i<traj_2.size();i++)
    {
        mergeTraj.push_back(traj_2[i]);
    }
    for(int i=1;i<traj_3.size();i++)
    {
        mergeTraj.push_back(traj_3[i]);
    }
    //移除相近点，暂时不需要

    return mergeTraj;
    

}





void TjaLinearInterpolation(const std::vector<float> &x, const std::vector<float> &y, const std::vector<float> &xSampled, std::vector<float> &ySampled)
{
int n = x.size();
int nSampled = xSampled.size();
int startSearch = 0;
if (n>3)
{
    for(int i=0;i<nSampled;i++)
    {
        if(xSampled[i]<=x[0]) //超出截断
        {
        //   float k = TjaCalculateHeading(x, y, n, 1);
        //   TanLimit(k);
          ySampled.push_back(y[0]);// + std::tan(k) * (xSampled[i] - x[0]));
        }
        else if(xSampled[i] >= x[n-1]) //超出截断
        {
        //   float k = TjaCalculateHeading(x, y, n, n);
        //   TanLimit(k);
          ySampled.push_back(y[n-1]);// + std::tan(k) * (xSampled[i] - x[n-1]));
        }
        else
        {
            while (x[startSearch + 1] < xSampled[i] && startSearch < n - 2)
            {
                startSearch = startSearch + 1;
            }       
               

            float t = safeDivide((xSampled[i] - x[startSearch]) , (x[startSearch + 1] - x[startSearch]),1e-6);
            ySampled.push_back((1-t) * y[startSearch] + t * y[startSearch + 1]);
            
            
        }

    }
}
}

float safeDivide(float nominator,float denominator,float threshold)
{
    float value = 0.0f;
        if(denominator < threshold && denominator > -threshold)
        {
                    value = nominator / threshold;
                    if (denominator < 0.0f)
                    {
                        value = -value;
                    }
        }
        else
        {
            value = nominator / denominator;
        }
        return value;
}

float TjaCalculateHeading(const std::vector<float> &x, const std::vector<float> &y,const int &n, int point)
{
    float heading=0.0f;
    if(n<2)
    {
        heading=0.0f;
    }
    else
    {
        float denominator = 0;
        float nominator = 0;
        int twoInt = 2;
        for (int i=std::max(twoInt, point - twoInt)-1; i<std::min(n, point + twoInt); i++)
        {
            nominator = nominator + safeDivide((y[i] - y[i - 1]) , (x[i] - x[i - 1]),1e-6);
            denominator = denominator + 1;
        }
        heading = std::atan2(nominator, denominator);
    }
    return heading;
}

void TanLimit(float &k)
{
    float temp_angle=0.0f;
    int sign=0;
        if(k>0)
        {
            sign = 1; 
        }              
        else if(k<0)
        {
            sign =-1;
        }           
        else 
        {
            sign = 0;   
        }
        float angle = std::fmod(std::fabs(k),2*M_PI);    
        temp_angle = std::fmod(angle,0.5*M_PI);
        if(std::fabs(temp_angle) >((0.5*M_PI)-0.00001))
        {
            temp_angle=((0.5*M_PI)-0.00001);      
        }
        k=temp_angle*sign;
}

}
}
