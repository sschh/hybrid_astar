#include "trajectory_controller.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>
/*
namespace HybridAStar
{
	namespace control
	{

		void EcarxLatController::CloseLogFile()
		{
			if (FLAGS_enable_csv_debug && mpc_log_file_.is_open())
			{
				mpc_log_file_.close();
			}
		}

		void EcarxLatController::Stop() { CloseLogFile(); }

		std::string EcarxLatController::Name() const {return name_; }

		void EcarxLatController::LoadMPCGainScheduler(
			const MPCControllerConf &mpc_controller_conf)
		{
		}


		Status EcarxLatController::ComputeControlCommand(
			const localization::LocalizationEstimate *localization,
			const float *path_speed,
			Planner planner,
			const planning::ADCTrajectory *planning_published_trajectory,
			ControlCommand *cmd)
		{
			
			auto target_tracking_trajectory = *planning_published_trajectory;
			double Acc_commanded_output;
			double Wheel_angle_commanded_output;
			int step_total;
			double lon_acc0 = 0;//车辆的加速度
			double Wheel_angle_max = 470 / Steer_Ratio * M_PI / 180;//前轮偏角最大值rad（弧度

			double x0, y0, Wheel_angle0, head_angle0;
			double v0 = 0; 
			x0 = localization->pose().position().x();//车辆初始utm坐标位置
			y0 = localization->pose().position().y();
			std::cout << " vehicle x0 and y0 is  " << std::setprecision(12) << x0 << "  " << std::setprecision(12) << y0<<std::endl;//打印日志作用，用于调试
			std::cout<<"beta wheelanglecom is "<<beta_Wheel_angle_com<<std::endl;
			std::cout << " vehicle x0 and y0 is  " << std::setprecision(12) << x0 << "  " << std::setprecision(12) << y0<<std::endl;
			Wheel_angle0 =chassis->steering_percentage()/100*470/Steer_Ratio*M_PI/180;//获取车辆前轮偏角（弧度
			std::cout<<"wheel angle0 is"<<Wheel_angle0<<std::endl;
			v0 = path_speed[0];//车辆速度

			if (v0 < 1)//保证起步时的稳定性，所以降低起步时的最大加速度、最大前轮偏角
			{
				Wheel_angle_max=Wheel_angle_max/25;
				Acc_max=1;
			}
			std::cout << " vehicle v0 speed" << std::setprecision(12) << v0<<std::endl;

			int size =target_tracking_trajectory.trajectory_point().size();//获取轨迹点的个数
			//target_tracking_trajectory.trajectory_point(0).path_point().x()    为第0个轨迹点的x（x为utm坐标系，可类比为全局坐标系）
			//target_tracking_trajectory.trajectory_point(0).path_point().y()    为第0个轨迹点的y（y为utm坐标系，可类比为全局坐标系）
			
			//找到最近的点
			double min_distance = pow((x0 -target_tracking_trajectory.trajectory_point(0).path_point().x()), 2) + pow((y0 -target_tracking_trajectory.trajectory_point(0).path_point().y()), 2);
			int index = 0; //轨迹点中距离车辆位置最近的点的索引
			for (int i = 0; i < size; i++)
			{
				double distance = pow((x0 -target_tracking_trajectory.trajectory_point(i).path_point().x()), 2) + pow((y0 -target_tracking_trajectory.trajectory_point(i).path_point().y()), 2);
				std::cout<< "index:"<<i<<" y is "<<target_tracking_trajectory.trajectory_point(i).path_point().y()<<std::endl;
				if (distance < min_distance)
				{
					min_distance = distance;
					index = i;
				}
				//std::cout <<target_tracking_trajectory.trajectory_point(i).DebugString();
			}
			// std::cout <<"=============point==========:";
			index = index + 1;
			std::cout << " index is " << index << "  size is" << size<<std::endl;
            //target_tracking_trajectory.trajectory_point(0).path_point().theta()    为第0个轨迹点的航向角，东为0，-pi~+pi
			head_angle0 = localization->pose().heading() -target_tracking_trajectory.trajectory_point(index - 1).path_point().theta(); //车辆的航向角 减去 最近轨迹点的航向角
			std::cout << " vehicle heading angle: " << std::setprecision(12) << localization->pose().heading()<<std::endl;
			std::cout << " point  heading angle:" << std::setprecision(12) <<target_tracking_trajectory.trajectory_point(index - 1).path_point().theta()<<std::endl;
			std::cout << " vehicle heading angle-point heading angle " << head_angle0<<std::endl;
			//保证index代表的轨迹点肯定在车辆位置之后,所以之后要循迹的轨迹点是从index：size   但最近的的轨迹点的索引还是index-1
			std::cout << " the nearset point position x0 " << std::setprecision(12) <<target_tracking_trajectory.trajectory_point(index - 1).path_point().x()<<std::endl;
			std::cout << " the nearset point position yo " << std::setprecision(12) <<target_tracking_trajectory.trajectory_point(index - 1).path_point().y()<<std::endl;
			std::cout << " the nearset point+1 position x0 " << std::setprecision(12) <<target_tracking_trajectory.trajectory_point(index).path_point().x()<<std::endl;
			std::cout << " the nearset point+1 position yo " << std::setprecision(12) <<target_tracking_trajectory.trajectory_point(index).path_point().y()<<std::endl;
			if (index - 2 >= 0)
			{
				std::cout << " the nearset point-1 position x0 " << std::setprecision(12) <<target_tracking_trajectory.trajectory_point(index - 2).path_point().x()<<std::endl;
				std::cout << " the nearset point-1 position yo " << std::setprecision(12) <<target_tracking_trajectory.trajectory_point(index - 2).path_point().y()<<std::endl;
			}
			//定义要循迹的轨迹信息,其中车辆信息为第0个点,其余信息为后面的轨迹点
			double nearest_trajectory_info_lat_position_tar[101];
			double nearest_trajectory_info_lon_position_tar[101];
			double nearest_trajectory_info_s[100];
			double nearest_trajectory_info_speed_tar[101];
			double nearest_trajectory_info_time_tar[100];
			double nearest_trajectory_info_lon_slope[101];
			double nearest_trajectory_info_lat_slope[101];
			double nearest_trajectory_info_relative_time[101];
			double nearest_trajectory_info_kappa[100];
			//求初始横向误差（车相对于轨迹的横向误差
			double dx = x0 -target_tracking_trajectory.trajectory_point(index - 1).path_point().x();
			double dy = y0 -target_tracking_trajectory.trajectory_point(index - 1).path_point().y();
			double lateral_error0 = dy*std::cos(target_tracking_trajectory.trajectory_point(index - 1).path_point().theta()) - dx * std::sin(target_tracking_trajectory.trajectory_point(index - 1).path_point().theta());
			
			std::cout<<" the lateralerroe0 and point heta " << std::setprecision(12) << lateral_error0 << "  " << std::setprecision(12) <<target_tracking_trajectory.trajectory_point(index - 1).path_point().theta()<<std::endl;

			///////////////////////////////////////////////////////////////////////////////////
			nearest_trajectory_info_lat_position_tar[0] = lateral_error0; //这个是到轨迹的初始横向距离
			//nearest_trajectory_info_lat_position_tar[0]=0;
			nearest_trajectory_info_speed_tar[0] = v0;
			std::cout<<"v0 is "<<v0<<" nearest_trajectory_info_speed_tar[0] is "<<nearest_trajectory_info_speed_tar[0]<<std::endl;
			//target_tracking_trajectory.trajectory_point(index - 1).path_point().kappa() 为该轨迹点的曲率
			nearest_trajectory_info_kappa[0] =target_tracking_trajectory.trajectory_point(index - 1).path_point().kappa();

			//计算nearest_trajectory_info_lon_position_tar[0],即车辆在轨迹中的累积S
			nearest_trajectory_info_s[0] = pow((
												   pow((x0 -target_tracking_trajectory.trajectory_point(index).path_point().x()), 2) + pow((y0 -target_tracking_trajectory.trajectory_point(index).path_point().y()), 2) - lateral_error0 * lateral_error0),
											   0.5);
			std::cout << " nearest_trajectory_info_s[0] " << std::setprecision(12) << nearest_trajectory_info_s[0]<<std::endl;
			//第一个轨迹点的s值为0，target_tracking_trajectory.trajectory_point(index).path_point().s()为相对于第一个轨迹点的累积距离
			nearest_trajectory_info_lon_position_tar[0] =target_tracking_trajectory.trajectory_point(index).path_point().s() - nearest_trajectory_info_s[0];
			//target_tracking_trajectory.trajectory_point(i).relative_time();第一个轨迹点的相对时间为0
			//nearest_trajectory_info_relative_time[0],即车辆的relative_time为0
			double header_time_ =target_tracking_trajectory.header().timestamp_sec();//planning模块规划轨迹时的系统时间，为unix时间
			double current_control_time = Clock::NowInSeconds();//运行到该行的系统时间
			std::cout << "header time is " << header_time_ << "  current time is " << current_control_time<<std::endl;
			nearest_trajectory_info_relative_time[0] = current_control_time - header_time_<<std::endl;
			std::cout << "relative time0 is " << current_control_time - header_time_<<std::endl;

			//原本轨迹点的时间间隔为0.02s，考虑要循迹的轨迹点时间间隔大于time_interval s,选择其中部分轨迹点,放在数组的前j个位置
			//即如果time_interval=0.1，那么只会把planning的每五个轨迹点选择一个放入nearest_trajectory_info数组中
			int j = 0;
			// for(int i=index;i<size-1;i++)
			for (int i = 0; i < size - 1; i++)
			{
				if (target_tracking_trajectory.trajectory_point(i).relative_time() - nearest_trajectory_info_relative_time[j] >= time_interval)
				{
					nearest_trajectory_info_lon_position_tar[j + 1] =target_tracking_trajectory.trajectory_point(i).path_point().s();
					nearest_trajectory_info_speed_tar[j + 1] =target_tracking_trajectory.trajectory_point(i).v();
					;
					nearest_trajectory_info_relative_time[j + 1] =target_tracking_trajectory.trajectory_point(i).relative_time();
					nearest_trajectory_info_time_tar[j] = nearest_trajectory_info_relative_time[j + 1] - nearest_trajectory_info_relative_time[j];

					nearest_trajectory_info_lon_slope[j] = 0;
					nearest_trajectory_info_lat_slope[j] = 0;
					//更新
					j = j + 1;
				}
				if (j == 100) //定义的存放轨迹的数组长度
					break;
			}
			//此时的j就是轨迹数组中0到j个的有用轨迹信息
			step_total = j;
			std::cout << "steptotal is " << step_total;


			
			std::cout<<"v0 is "<<v0<<" nearest_trajectory_info_speed_tar[0] is "<<nearest_trajectory_info_speed_tar[0]<<std::endl;
			//纵向控制
			double state_vector_lonspeed[101];
			{
				int i, j, k, stp;
				double k_istep, lon_slope_istep;
				double beta[4][4], QQ[4][4], Q[3][3], z[3][1], u, D[3][1], R, Rk, F, Fk, J, E;
				double A[3][3], Ak[3][3], B[3][1], Bk[3][1], Ck[3][1], Sk[3][3];
				double QkP1_multi_Ak[3][3];
				double Pk, Gk[1][3], Hk, Tk[3][1];
				double QkP1[3][3], DkP1[3][1], EkP1, Qk[3][3], Dk[3][1], Ek, QkP1_multi_Ck[3][1];
				double GM[3][100], HM[1][100];

				for (i = 0; i < 4; i++)
				{
					for (j = 0; j < 4; j++)
					{
						beta[i][j] = 0;
					}
				};
				beta[0][0] = beta_lon_position;
				beta[1][1] = beta_lon_speed;
				beta[2][2] = beta_lon_acc;
				beta[3][3] = beta_lon_acc_com;
				double k_road[100], Control_vector_new[100], Control_vector[100], State_vector[3][101],
					Lon_position[101], Speed[101], Acc[101], acc_commanded[100];
				for (i = 0; i < 100; i++)
				{
					k_road[i] = nearest_trajectory_info_kappa[i];
					Control_vector_new[i] = 0;
					Control_vector[i] = 0;
					State_vector[0][i] = 0;
					State_vector[1][i] = 0;
					State_vector[2][i] = 0;
					Lon_position[i] = 0;
					Speed[i] = 0;
					Acc[i] = 0;
				}
				i = 100;
				State_vector[0][i] = 0;
				State_vector[1][i] = 0;
				State_vector[2][i] = 0;
				Lon_position[i] = 0;
				Speed[i] = 0;
				Acc[i] = 0;

				for (int iter_times = 1; iter_times <= iter_times_max; iter_times++)
				{
					Lon_position[0] = nearest_trajectory_info_lon_position_tar[0];
					Speed[0] = nearest_trajectory_info_speed_tar[0];
					Acc[0] = lon_acc0;

					for (i = 0; i < 100; i++)
					{
						acc_commanded[i] = Control_vector_new[i];
					}
					for (i = 0; i < step_total; i++)
					{
						Lon_position[i + 1] = Lon_position[i] + Speed[i] * nearest_trajectory_info_time_tar[i] * cos(nearest_trajectory_info_lon_slope[i]);
						Speed[i + 1] = Speed[i] + Acc[i] * nearest_trajectory_info_time_tar[i];
						Acc[i + 1] = Acc[i] - Acc[i] / lon_acc_delay * nearest_trajectory_info_time_tar[i] + acc_commanded[i] / lon_acc_delay * nearest_trajectory_info_time_tar[i];
					}
					for (i = 0; i < 101; i++) //???line47
					{
						State_vector[0][i] = Lon_position[i];
						State_vector[1][i] = Speed[i];
						State_vector[2][i] = Acc[i];
					}
					for (i = 0; i < 100; i++)
					{
						Control_vector[i] = acc_commanded[i];
					}
					double Lon_position_istep, Speed_istep, Acc_istep, Acc_desired_istep;
					Lon_position_istep = Lon_position[step_total];
					Speed_istep = Speed[step_total];
					Acc_istep = Acc[step_total];
					Acc_desired_istep = acc_commanded[step_total];

					//line 59
					for (k = 0; k < 4; k++)
					{
						for (j = 0; j < 4; j++)
						{
							QQ[k][j] = 0;
						}
					};
					
					QQ[0][0] = 2 * beta[0][0] * nearest_trajectory_info_time_tar[step_total - 1];
					QQ[1][1] = 2 * beta[1][1] * nearest_trajectory_info_time_tar[step_total - 1];
					QQ[2][2] = 2 * beta[2][2] * nearest_trajectory_info_time_tar[step_total - 1];
					QQ[3][3] = 2 * beta[3][3] * nearest_trajectory_info_time_tar[step_total - 1];

					//line 60
					for (k = 0; k < 3; k++)
					{
						for (j = 0; j < 3; j++)
						{
							Q[k][j] = 0;
						}
					};
					Q[0][0] = QQ[0][0];
					Q[1][1] = QQ[1][1];
					Q[2][2] = QQ[2][2];
					//line 62
					z[0][0] = Lon_position_istep;
					z[1][0] = Speed_istep;
					z[2][0] = Acc_istep;
					//line 63
					u = Acc_desired_istep;
					//line 65-69
					D[0][0] = 2 * beta[0][0] * (Lon_position_istep - nearest_trajectory_info_lon_position_tar[step_total]) * nearest_trajectory_info_time_tar[step_total - 1] - Q[0][0] * z[0][0];
					D[1][0] = 2 * beta[1][1] * (Speed_istep - nearest_trajectory_info_speed_tar[step_total]) * nearest_trajectory_info_time_tar[step_total - 1] - Q[1][1] * z[1][0];
					D[2][0] = 2 * beta[2][2] * Acc_istep * nearest_trajectory_info_time_tar[step_total - 1] - Q[2][2] * z[2][0];
					R = 2 * beta[3][3] * nearest_trajectory_info_time_tar[step_total - 1];
					F = 2 * beta[3][3] * u * nearest_trajectory_info_time_tar[step_total - 1] - R * u;
					J = beta[0][0] * (Lon_position_istep - nearest_trajectory_info_lon_position_tar[step_total]) * (Lon_position_istep - nearest_trajectory_info_lon_position_tar[step_total]) + beta[1][1] * (Speed_istep - nearest_trajectory_info_speed_tar[step_total]) * (Speed_istep - nearest_trajectory_info_speed_tar[step_total]) + beta[2][2] * Acc_istep * Acc_istep + beta[3][3] * (Acc_desired_istep) * (Acc_desired_istep);
					//line 71,73
					E = J -
						(z[0][0] * D[0][0] + z[1][0] * D[1][0] + z[2][0] * D[2][0] + z[0][0] * Q[0][0] * z[0][0] / 2 + z[1][0] * Q[1][1] * z[1][0] / 2 + z[2][0] * Q[2][2] * z[2][0] / 2) -
						(u * F + u * R * u / 2);
					//line 74
					for (i = 0; i < 3; i++)
					{
						for (j = 0; j < 3; j++)
						{
							QkP1[i][j] = 0;
						}
					};
					QkP1[0][0] = Q[0][0];
					QkP1[1][1] = Q[1][1];
					QkP1[2][2] = Q[2][2];
					DkP1[0][0] = D[0][0];
					DkP1[1][0] = D[1][0];
					DkP1[2][0] = D[2][0];
					EkP1 = E;
					for (i = 0; i < 100; i++)
					{
						GM[0][i] = 0;
						GM[1][i] = 0;
						GM[2][i] = 0;
						HM[0][i] = 0;
					}

					//line77
					for (i = step_total; i >= 1; i--)
					{
						Lon_position_istep = Lon_position[i - 1];
						Speed_istep = Speed[i - 1];
						Acc_istep = Acc[i - 1];
						Acc_desired_istep = acc_commanded[i - 1];
						k_istep = k_road[i - 1];
						lon_slope_istep = nearest_trajectory_info_lon_slope[i - 1];
						//line 85
						A[0][0] = 1;
						A[0][1] = cos(lon_slope_istep) * nearest_trajectory_info_time_tar[i - 1];
						A[0][2] = 0;
						A[1][0] = 0;
						A[1][1] = 1;
						A[1][2] = 1 * nearest_trajectory_info_time_tar[i - 1];
						A[2][0] = 0;
						A[2][1] = 0;
						A[2][2] = 1 - 1 / lon_acc_delay * nearest_trajectory_info_time_tar[i - 1];
						B[0][0] = 0;
						B[1][0] = 0;
						B[2][0] = 1 / lon_acc_delay * nearest_trajectory_info_time_tar[i - 1];

						//line 90
					
						QQ[0][0] = 2 * beta[0][0] * nearest_trajectory_info_time_tar[i - 1];
						QQ[1][1] = 2 * beta[1][1] * nearest_trajectory_info_time_tar[i - 1];
						QQ[2][2] = 2 * beta[2][2] * nearest_trajectory_info_time_tar[i - 1];
						QQ[3][3] = 2 * beta[3][3] * nearest_trajectory_info_time_tar[i - 1];
						//line 91
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								Q[k][j] = 0;
							}
						};
						Q[0][0] = QQ[0][0];
						Q[1][1] = QQ[1][1];
						Q[2][2] = QQ[2][2];
						//line 93
						z[0][0] = Lon_position_istep;
						z[1][0] = Speed_istep;
						z[2][0] = Acc_istep;
						//line 94
						u = Acc_desired_istep;
						//line 96
						D[0][0] = 2 * beta[0][0] * (Lon_position_istep - nearest_trajectory_info_lon_position_tar[i - 1]) * nearest_trajectory_info_time_tar[i - 1] - Q[0][0] * z[0][0];
						D[1][0] = 2 * beta[1][1] * (Speed_istep - nearest_trajectory_info_speed_tar[i - 1]) * nearest_trajectory_info_time_tar[i - 1] - Q[1][1] * z[1][0];
						D[2][0] = 2 * beta[2][2] * Acc_istep * nearest_trajectory_info_time_tar[i - 1] - Q[2][2] * z[2][0];
						R = 2 * beta[3][3] * nearest_trajectory_info_time_tar[i - 1];
						F = 2 * beta[3][3] * u * nearest_trajectory_info_time_tar[i - 1] - R * u;
						J = beta[0][0] * (Lon_position_istep - nearest_trajectory_info_lon_position_tar[i - 1]) * (Lon_position_istep - nearest_trajectory_info_lon_position_tar[i - 1]) + beta[1][1] * (Speed_istep - nearest_trajectory_info_speed_tar[i - 1]) * (Speed_istep - nearest_trajectory_info_speed_tar[i - 1]) + beta[2][2] * Acc_istep * Acc_istep + beta[3][3] * (Acc_desired_istep) * (Acc_desired_istep);
						E = J -
							(z[0][0] * D[0][0] + z[1][0] * D[1][0] + z[2][0] * D[2][0] + z[0][0] * Q[0][0] * z[0][0] / 2 + z[1][0] * Q[1][1] * z[1][0] / 2 + z[2][0] * Q[2][2] * z[2][0] / 2) -
							(u * F + u * R * u / 2);
						//line 104,105
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								Ak[k][j] = A[k][j];
							}
							Bk[k][0] = B[k][0];
							Ck[k][0] = 0;
						};
						Rk = R;
						Fk = F;
						//line 106
						Pk = Rk + Bk[0][0] * QkP1[0][0] * Bk[0][0] + Bk[1][0] * QkP1[1][1] * Bk[1][0] + Bk[2][0] * QkP1[2][2] * Bk[2][0];
						if (fabs(Pk) < 0.0001) //避免出现0为分母
						{
							Pk = 0.0001;
						}
						Pk = 1 / Pk;

						//line 107
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								QkP1_multi_Ak[k][j] = QkP1[k][0] * Ak[0][j] + QkP1[k][1] * Ak[1][j] + QkP1[k][2] * Ak[2][j];
							}
						};
						Gk[0][0] = -1 * Pk * (Bk[0][0] * QkP1_multi_Ak[0][0] + Bk[1][0] * QkP1_multi_Ak[1][0] + Bk[2][0] * QkP1_multi_Ak[2][0]);
						Gk[0][1] = -1 * Pk * (Bk[0][0] * QkP1_multi_Ak[0][1] + Bk[1][0] * QkP1_multi_Ak[1][1] + Bk[2][0] * QkP1_multi_Ak[2][1]);
						Gk[0][2] = -1 * Pk * (Bk[0][0] * QkP1_multi_Ak[0][2] + Bk[1][0] * QkP1_multi_Ak[1][2] + Bk[2][0] * QkP1_multi_Ak[2][2]);
						//line 108 ?????????CK?????CK????0
						for (k = 0; k < 3; k++)
						{
							QkP1_multi_Ck[k][0] = QkP1[k][0] * Ck[0][0] + QkP1[k][1] * Ck[1][0] + QkP1[k][2] * Ck[2][0];
						}
						Hk = -1 * Pk *
							 (Bk[0][0] * (DkP1[0][0] + QkP1_multi_Ck[0][0]) + Bk[1][0] * (DkP1[1][0] + QkP1_multi_Ck[1][0]) + Bk[2][0] * (DkP1[2][0] + QkP1_multi_Ck[2][0]) + Fk);
						//line 109
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								Sk[k][j] = Ak[k][j] + Bk[k][0] * Gk[0][j];
							}
						};
						//line 110
						for (k = 0; k < 3; k++)
						{
							Tk[k][0] = Bk[k][0] * Hk + Ck[k][0];
						};
						//line 111,112
						GM[0][i - 1] = Gk[0][0];
						GM[1][i - 1] = Gk[0][1];
						GM[2][i - 1] = Gk[0][2];
						HM[0][i - 1] = Hk;
						//line 114
						double Sktranspose_multi_QkP1[3][3], Sktranspose[3][3], Sktranspose_multi_QkP1_multi_Sk[3][3];
						for (k = 0; k < 3; k++) //Qk Sk?????
						{
							for (j = 0; j < 3; j++)
							{
								Qk[k][j] = 0;
								Sktranspose[k][j] = 0;
							}
						};

						for (k = 0; k < 3; k++) //???
						{
							for (j = 0; j < 3; j++)
							{
								Sktranspose[k][j] = Sk[j][k];
							}
						};
						for (k = 0; k < 3; k++) //???????
						{
							for (j = 0; j < 3; j++)
							{
								Sktranspose_multi_QkP1[k][j] = Sktranspose[k][0] * QkP1[0][j] + Sktranspose[k][1] * QkP1[1][j] + Sktranspose[k][2] * QkP1[2][j];
							}
						};
						for (k = 0; k < 3; k++) //???????
						{
							for (j = 0; j < 3; j++)
							{
								Sktranspose_multi_QkP1_multi_Sk[k][j] = Sktranspose_multi_QkP1[k][0] * Sk[0][j] + Sktranspose_multi_QkP1[k][1] * Sk[1][j] + Sktranspose_multi_QkP1[k][2] * Sk[2][j];
							}
						};
						for (k = 0; k < 3; k++) //???qk
						{
							for (j = 0; j < 3; j++)
							{
								Qk[k][j] = Sktranspose_multi_QkP1_multi_Sk[k][j] + Q[k][j];
								Qk[k][j] += Gk[0][k] * Rk * Gk[0][j];
							}
						};
						//line 115
						for (k = 0; k < 3; k++)
						{
							Dk[k][0] = Gk[0][k] * Rk * Hk +
									   Sktranspose_multi_QkP1[k][0] * Tk[0][0] + Sktranspose_multi_QkP1[k][1] * Tk[1][0] + Sktranspose_multi_QkP1[k][2] * Tk[0][2] +
									   Sktranspose[k][0] * DkP1[0][0] + Sktranspose[k][1] * DkP1[1][0] + Sktranspose[k][2] * DkP1[0][2] +
									   D[k][0];
						}
						//line 116
						double Tktranspose_multi_QkP1[1][3];
						for (k = 0; k < 3; k++) //
						{
							Tktranspose_multi_QkP1[0][k] = Tk[0][0] * QkP1[0][k] + Tk[1][0] * QkP1[1][k] + Tk[2][0] * QkP1[2][k];
						};
						Ek = Hk * Rk * Hk / 2 +
							 (Tktranspose_multi_QkP1[0][0] * Tk[0][0] + Tktranspose_multi_QkP1[0][1] * Tk[1][0] + Tktranspose_multi_QkP1[0][2] * Tk[0][2]) +
							 Hk * Fk +
							 (Tk[0][0] * DkP1[0][0] + Tk[1][0] * DkP1[1][0] + Tk[2][0] * DkP1[2][0]) +
							 EkP1 + E;
						//line 122
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								QkP1[k][j] = Qk[k][j];
							}
							DkP1[k][0] = Dk[k][0];
						};
						EkP1 = Ek;
					}
					//line 118
					for (i = 1; i <= step_total; i++)
					{
						Speed_istep = State_vector[1][i - 1];
						k_istep = k_road[i - 1];
						lon_slope_istep = nearest_trajectory_info_lon_slope[i - 1];
						Control_vector_new[i - 1] = GM[0][i - 1] * State_vector[0][i - 1] + GM[1][i - 1] * State_vector[1][i - 1] + GM[2][i - 1] * State_vector[2][i - 1] + HM[0][i - 1];
						A[0][0] = 1;
						A[0][1] = cos(lon_slope_istep) * nearest_trajectory_info_time_tar[i - 1];
						A[0][2] = 0;
						A[1][0] = 0;
						A[1][1] = 1;
						A[1][2] = 1 * nearest_trajectory_info_time_tar[i - 1];
						A[2][0] = 0;
						A[2][1] = 0;
						A[2][2] = 1 - 1 / lon_acc_delay * nearest_trajectory_info_time_tar[i - 1];
						B[0][0] = 0;
						B[1][0] = 0;
						B[2][0] = 1 / lon_acc_delay * nearest_trajectory_info_time_tar[i - 1];
						//line 128
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								Ak[k][j] = A[k][j];
							}
							Bk[k][0] = B[k][0];
							Ck[k][0] = 0;
						};
						//line 129
						for (k = 0; k < 3; k++)
						{
							State_vector[k][i] = Ak[k][0] * State_vector[0][i - 1] + Ak[k][1] * State_vector[1][i - 1] + Ak[k][2] * State_vector[2][i - 1] +
												 Bk[k][0] * Control_vector_new[i - 1] +
												 Ck[k][0];
						}
						if (Control_vector_new[i - 1] > Acc_max)
						{
							Control_vector_new[i - 1] = Acc_max;
						}
						if (Control_vector_new[i - 1] < Acc_min)
						{
							Control_vector_new[i - 1] = Acc_min;
						}
						//line 152
						for (k = 0; k < 3; k++)
						{
							State_vector[k][i] = Ak[k][0] * State_vector[0][i - 1] + Ak[k][1] * State_vector[1][i - 1] + Ak[k][2] * State_vector[2][i - 1] +
												 Bk[k][0] * Control_vector_new[i - 1] +
												 Ck[k][0];
						}
					}
					//line 155
					stp = 1;
					for (i = 1; i <= step_total; i++)
					{
						if (fabs(Control_vector[i - 1] - Control_vector_new[i - 1]) > 0.001)
						{
							stp = 0;
						}
					}
					//line 164
					for (i = 1; i <= step_total; i++)
					{
						Control_vector[i - 1] = Control_vector_new[i - 1];
					}
					if (stp)
					{
						break;
					}
				}

				Acc_commanded_output = Control_vector[0];
				for (i = 1; i <= step_total + 1; i++)
				{
					state_vector_lonspeed[i - 1] = State_vector[1][i - 1];
				}
			}

			for (int i = 1; i <= step_total + 1; i++)
			{
				if (state_vector_lonspeed[i - 1] < 0.001)
				{
					state_vector_lonspeed[i - 1] = 0.001;
				}
			}
			//根据纵向规划的速度和轨迹点的delta_t计算deltas，并查询deltas后最近的kappa
			double accumulated_distance = nearest_trajectory_info_lon_position_tar[0];
			for (int i = 0; i <= step_total - 1; i++)
			{
				nearest_trajectory_info_s[i] = state_vector_lonspeed[i] * nearest_trajectory_info_time_tar[i];
				accumulated_distance = accumulated_distance + nearest_trajectory_info_s[i];
				nearest_trajectory_info_lat_position_tar[i + 1] = 0;
				//查询在lon_position_tar[0]增加deltas后点的曲率，通过查找最近点的方法
				min_distance = fabs(accumulated_distance -target_tracking_trajectory.trajectory_point(0).path_point().s());
				std::cout << "i " << i << " accumulated_distance " << accumulated_distance << " min_distance " << min_distance<<std::endl;
				index = 0;
				for (int j = 1; j < size; j++)
				{
					if (fabs(accumulated_distance -target_tracking_trajectory.trajectory_point(j).path_point().s()) < min_distance)
					{
						index = j;
						min_distance = fabs(accumulated_distance -target_tracking_trajectory.trajectory_point(j).path_point().s());
					}
				}
				std::cout << "index " << index;
				nearest_trajectory_info_kappa[i + 1] =target_tracking_trajectory.trajectory_point(index).path_point().kappa();
			}
			std::cout<<"v0 is "<<v0<<" nearest_trajectory_info_speed_tar[0] is "<<nearest_trajectory_info_speed_tar[0]<<std::endl;
			std::cout << "trajectory_info lat  lon speed kappa relative_time time s  state_vector_lonspeed"<<std::endl;
			for (int i = 0; i <= step_total + 1; i++)
			{

				std::cout << "number " << i << "  :" << std::setprecision(6) << nearest_trajectory_info_lat_position_tar[i] << "  " << std::setprecision(6) << nearest_trajectory_info_lon_position_tar[i] << "  " << std::setprecision(6) << nearest_trajectory_info_speed_tar[i] << "  " << std::setprecision(6) << nearest_trajectory_info_kappa[i] << "  " << std::setprecision(6) << nearest_trajectory_info_relative_time[i]
					   << "  " << std::setprecision(6) << nearest_trajectory_info_time_tar[i] << "  " << std::setprecision(6) << nearest_trajectory_info_s[i]
					   << std::setprecision(6) << "  " << state_vector_lonspeed[i]<<std::endl;
			}
			std::cout << "------------------------------------";

			//横向控制

			{

				int i, j, k, stp;
				double k_istep, lat_slope_istep;
				double beta[4][4], QQ[4][4], Q[3][3], z[3][1], u, D[3][1], R, Rk, F, Fk, J, E;
				double A[3][3], Ak[3][3], B[3][1], Bk[3][1], C[3][1], Ck[3][1], Sk[3][3];
				double QkP1_multi_Ak[3][3], QkP1_multi_Ck[3][1];
				double Pk, Gk[1][3], Hk, Tk[3][1];
				double QkP1[3][3], DkP1[3][1], EkP1, Qk[3][3], Dk[3][1], Ek;
				double GM[3][100], HM[1][100];

				for (i = 0; i < 4; i++)
				{
					for (j = 0; j < 4; j++)
					{
						beta[i][j] = 0;
					}
				};
				beta[0][0] = beta_Lat_position_error;
				beta[1][1] = beta_head_angle_error;
				beta[2][2] = beta_Wheel_angle;
				beta[3][3] = beta_Wheel_angle_com;
				double k_road[100], Control_vector_new[100], Control_vector[100], State_vector[3][101],
					Lat_position[101], Head_angle[101], Wheel_angle[101], Speed[101], Wheel_angle_commanded[100];
				for (i = 0; i < 100; i++)
				{
					k_road[i] = nearest_trajectory_info_kappa[i]; //??????????
					Control_vector_new[i] = 0;
					Control_vector[i] = 0;
					State_vector[0][i] = 0;
					State_vector[1][i] = 0;
					State_vector[2][i] = 0;
					Lat_position[i] = 0;
					Speed[i] = 0;
					Head_angle[i] = 0;
					Wheel_angle[i] = 0;
					Wheel_angle_commanded[i] = 0;
				}
				i = 100;
				State_vector[0][i] = 0;
				State_vector[1][i] = 0;
				State_vector[2][i] = 0;
				Lat_position[i] = 0;
				Speed[i] = 0;
				Head_angle[i] = 0;
				Wheel_angle[i] = 0;

				for (int iter_times = 1; iter_times <= iter_times_max; iter_times++)
				{
					Lat_position[0] = nearest_trajectory_info_lat_position_tar[0];
					Head_angle[0] = head_angle0;
					Speed[0] = state_vector_lonspeed[0];
					Wheel_angle[0] = Wheel_angle0;

					for (i = 0; i < 100; i++)
					{
						Wheel_angle_commanded[i] = Control_vector_new[i];
					}
					//  x_utm[0] = x0;
					//  y_utm[0] = y0;
					for (i = 0; i < step_total; i++)
					{
						Lat_position[i + 1] = Lat_position[i] + Head_angle[i] * nearest_trajectory_info_s[i] * cos(nearest_trajectory_info_lat_slope[i]);
						Head_angle[i + 1] = Head_angle[i] + (Wheel_angle[i] / (Wheelbase_f + Wheelbase_r) - k_road[i]) * nearest_trajectory_info_s[i];

						Wheel_angle[i + 1] = Wheel_angle[i] - Wheel_angle[i] / Wheel_angle_delay * nearest_trajectory_info_s[i] / state_vector_lonspeed[i] + Wheel_angle_commanded[i] / Wheel_angle_delay * nearest_trajectory_info_s[i] / state_vector_lonspeed[i];
						// x_utm[i+1] = x_utm[i] + Speed[i] * cos(theta);
					}
					for (i = 0; i < 101; i++) //???line47
					{
						State_vector[0][i] = Lat_position[i];
						State_vector[1][i] = Head_angle[i];
						State_vector[2][i] = Wheel_angle[i];
					}
					for (i = 0; i < 100; i++)
					{
						Control_vector[i] = Wheel_angle_commanded[i];
					}
					double Lat_position_istep, Head_angle_istep,
						Speed_istep, Wheel_angle_istep, Wheel_angle_desired_istep, Lat_position_tar_istep;
					Lat_position_istep = Lat_position[step_total];
					Head_angle_istep = Head_angle[step_total];
					Speed_istep = state_vector_lonspeed[step_total];
					Wheel_angle_istep = Wheel_angle[step_total];
					Wheel_angle_desired_istep = Wheel_angle_commanded[step_total];
					Lat_position_tar_istep = nearest_trajectory_info_lat_position_tar[step_total];

					//line 59
					for (k = 0; k < 4; k++)
					{
						for (j = 0; j < 4; j++)
						{
							QQ[k][j] = 0;
						}
					};
					
					QQ[0][0] = 2 * beta[0][0] * nearest_trajectory_info_s[step_total - 1];
					QQ[1][1] = 2 * beta[1][1] * nearest_trajectory_info_s[step_total - 1];
					QQ[2][2] = 2 * beta[2][2] * nearest_trajectory_info_s[step_total - 1];
					QQ[3][3] = 2 * beta[3][3] * nearest_trajectory_info_s[step_total - 1];

					//line 60
					for (k = 0; k < 3; k++)
					{
						for (j = 0; j < 3; j++)
						{
							Q[k][j] = 0;
						}
					};
					Q[0][0] = QQ[0][0];
					Q[1][1] = QQ[1][1];
					Q[2][2] = QQ[2][2];
					//line 62
					z[0][0] = Lat_position_istep;
					z[1][0] = Head_angle_istep;
					z[2][0] = Wheel_angle_istep;
					//line 63
					u = Wheel_angle_desired_istep;
					//line 65-69
					D[0][0] = 2 * beta[0][0] * (Lat_position_istep - Lat_position_tar_istep) * nearest_trajectory_info_s[step_total - 1] - Q[0][0] * z[0][0];

					D[1][0] = 2 * beta[1][1] * (Head_angle_istep)*nearest_trajectory_info_s[step_total - 1] - Q[1][1] * z[1][0];
					D[2][0] = 2 * beta[2][2] * Wheel_angle_istep * nearest_trajectory_info_s[step_total - 1] - Q[2][2] * z[2][0];

					R = 2 * beta[3][3] * nearest_trajectory_info_s[step_total - 1];
					F = 2 * beta[3][3] * u * nearest_trajectory_info_s[step_total - 1] - R * u;
					J = beta[0][0] * (Lat_position_istep - Lat_position_tar_istep) * (Lat_position_istep - Lat_position_tar_istep) + beta[1][1] * Head_angle_istep * Head_angle_istep + beta[2][2] * Wheel_angle_istep * Wheel_angle_istep + beta[3][3] * (Wheel_angle_desired_istep) * (Wheel_angle_desired_istep);
					//line 71,73
					E = J -
						(z[0][0] * D[0][0] + z[1][0] * D[1][0] + z[2][0] * D[2][0] + z[0][0] * Q[0][0] * z[0][0] / 2 + z[1][0] * Q[1][1] * z[1][0] / 2 + z[2][0] * Q[2][2] * z[2][0] / 2) -
						(u * F + u * R * u / 2);
					//line 74
					for (i = 0; i < 3; i++)
					{
						for (j = 0; j < 3; j++)
						{
							QkP1[i][j] = 0;
						}
					};
					QkP1[0][0] = Q[0][0];
					QkP1[1][1] = Q[1][1];
					QkP1[2][2] = Q[2][2];
					DkP1[0][0] = D[0][0];
					DkP1[1][0] = D[1][0];
					DkP1[2][0] = D[2][0];
					EkP1 = E;
					for (i = 0; i < 100; i++)
					{
						GM[0][i] = 0;
						GM[1][i] = 0;
						GM[2][i] = 0;
						HM[0][i] = 0;
					}

					//line77
					for (i = step_total; i >= 1; i--)
					{
						Lat_position_istep = Lat_position[i - 1];
						Head_angle_istep = Head_angle[i - 1];
						Speed_istep = state_vector_lonspeed[i - 1];
						Wheel_angle_istep = Wheel_angle[i - 1];
						Wheel_angle_desired_istep = Wheel_angle_commanded[i - 1];
						Lat_position_tar_istep = nearest_trajectory_info_lat_position_tar[i - 1];
						k_istep = k_road[i - 1];
						lat_slope_istep = nearest_trajectory_info_lat_slope[i - 1];
						//line 85
						A[0][0] = 1;
						A[0][1] = cos(lat_slope_istep) * nearest_trajectory_info_s[i - 1];
						A[0][2] = 0;
						A[1][0] = 0;
						A[1][1] = 1;
						A[1][2] = 1 / (Wheelbase_f + Wheelbase_r) * nearest_trajectory_info_s[i - 1];
						A[2][0] = 0;
						A[2][1] = 0;
						A[2][2] = 1 - 1 / Wheel_angle_delay / Speed_istep * nearest_trajectory_info_s[i - 1];
						B[0][0] = 0;
						B[1][0] = 0;
						B[2][0] = 1 / Wheel_angle_delay / Speed_istep * nearest_trajectory_info_s[i - 1];

						C[0][0] = 0;
						C[1][0] = -1 * k_istep * nearest_trajectory_info_s[i - 1];
						C[2][0] = 0;
						//line 90
						
						QQ[0][0] = 2 * beta[0][0] * nearest_trajectory_info_s[i - 1];
						QQ[1][1] = 2 * beta[1][1] * nearest_trajectory_info_s[i - 1];
						QQ[2][2] = 2 * beta[2][2] * nearest_trajectory_info_s[i - 1];
						QQ[3][3] = 2 * beta[3][3] * nearest_trajectory_info_s[i - 1];
						//line 91
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								Q[k][j] = 0;
							}
						};
						Q[0][0] = QQ[0][0];
						Q[1][1] = QQ[1][1];
						Q[2][2] = QQ[2][2];
						//line 93
						z[0][0] = Lat_position_istep;
						z[1][0] = Head_angle_istep;
						z[2][0] = Wheel_angle_istep;

						u = Wheel_angle_desired_istep;

						D[0][0] = 2 * beta[0][0] * (Lat_position_istep - Lat_position_tar_istep) * nearest_trajectory_info_s[i - 1] - Q[0][0] * z[0][0];
						D[1][0] = 2 * beta[1][1] * (Head_angle_istep)*nearest_trajectory_info_s[i - 1] - Q[1][1] * z[1][0];
						D[2][0] = 2 * beta[2][2] * Wheel_angle_istep * nearest_trajectory_info_s[i - 1] - Q[2][2] * z[2][0];
						R = 2 * beta[3][3] * nearest_trajectory_info_s[i - 1];
						F = 2 * beta[3][3] * u * nearest_trajectory_info_s[i - 1] - R * u;
						J = beta[0][0] * (Lat_position_istep - Lat_position_tar_istep) * (Lat_position_istep - Lat_position_tar_istep) + beta[1][1] * Head_angle_istep * Head_angle_istep + beta[2][2] * Wheel_angle_istep * Wheel_angle_istep + beta[3][3] * (Wheel_angle_desired_istep) * (Wheel_angle_desired_istep);

						E = J -
							(z[0][0] * D[0][0] + z[1][0] * D[1][0] + z[2][0] * D[2][0] + z[0][0] * Q[0][0] * z[0][0] / 2 + z[1][0] * Q[1][1] * z[1][0] / 2 + z[2][0] * Q[2][2] * z[2][0] / 2) -
							(u * F + u * R * u / 2);
						//line 104,105
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								Ak[k][j] = A[k][j];
							}
							Bk[k][0] = B[k][0];
							Ck[k][0] = C[k][0];
						};
						Rk = R;
						Fk = F;
						//line 106
						Pk = Rk + Bk[0][0] * QkP1[0][0] * Bk[0][0] + Bk[1][0] * QkP1[1][1] * Bk[1][0] + Bk[2][0] * QkP1[2][2] * Bk[2][0];
						if (fabs(Pk) < 0.0001) //避免出现0为分母
						{
							Pk = 0.0001;
						}
						Pk = 1 / Pk;
						//line 107
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								QkP1_multi_Ak[k][j] = QkP1[k][0] * Ak[0][j] + QkP1[k][1] * Ak[1][j] + QkP1[k][2] * Ak[2][j];
							}
						};
						Gk[0][0] = -1 * Pk * (Bk[0][0] * QkP1_multi_Ak[0][0] + Bk[1][0] * QkP1_multi_Ak[1][0] + Bk[2][0] * QkP1_multi_Ak[2][0]);
						Gk[0][1] = -1 * Pk * (Bk[0][0] * QkP1_multi_Ak[0][1] + Bk[1][0] * QkP1_multi_Ak[1][1] + Bk[2][0] * QkP1_multi_Ak[2][1]);
						Gk[0][2] = -1 * Pk * (Bk[0][0] * QkP1_multi_Ak[0][2] + Bk[1][0] * QkP1_multi_Ak[1][2] + Bk[2][0] * QkP1_multi_Ak[2][2]);

						//line 108 ?????????CK?????CK????0
						for (k = 0; k < 3; k++)
						{
							QkP1_multi_Ck[k][0] = QkP1[k][0] * Ck[0][0] + QkP1[k][1] * Ck[1][0] + QkP1[k][2] * Ck[2][0];
						}
						Hk = -1 * Pk *
							 (Bk[0][0] * (DkP1[0][0] + QkP1_multi_Ck[0][0]) + Bk[1][0] * (DkP1[1][0] + QkP1_multi_Ck[1][0]) + Bk[2][0] * (DkP1[2][0] + QkP1_multi_Ck[2][0]) + Fk);

						//line 109
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								Sk[k][j] = Ak[k][j] + Bk[k][0] * Gk[0][j];
							}
						};
						//line 110
						for (k = 0; k < 3; k++)
						{
							Tk[k][0] = Bk[k][0] * Hk + Ck[k][0];
						};
						//line 111,112
						GM[0][i - 1] = Gk[0][0];
						GM[1][i - 1] = Gk[0][1];
						GM[2][i - 1] = Gk[0][2];
						HM[0][i - 1] = Hk;

						//line 114
						double Sktranspose_multi_QkP1[3][3], Sktranspose[3][3], Sktranspose_multi_QkP1_multi_Sk[3][3];
						for (k = 0; k < 3; k++) //Qk Sk?????
						{
							for (j = 0; j < 3; j++)
							{
								Qk[k][j] = 0;
								Sktranspose[k][j] = 0;
							}
						};

						for (k = 0; k < 3; k++) //???
						{
							for (j = 0; j < 3; j++)
							{
								Sktranspose[k][j] = Sk[j][k];
							}
						};
						for (k = 0; k < 3; k++) //???????
						{
							for (j = 0; j < 3; j++)
							{
								Sktranspose_multi_QkP1[k][j] = Sktranspose[k][0] * QkP1[0][j] + Sktranspose[k][1] * QkP1[1][j] + Sktranspose[k][2] * QkP1[2][j];
							}
						};
						for (k = 0; k < 3; k++) //???????
						{
							for (j = 0; j < 3; j++)
							{
								Sktranspose_multi_QkP1_multi_Sk[k][j] = Sktranspose_multi_QkP1[k][0] * Sk[0][j] + Sktranspose_multi_QkP1[k][1] * Sk[1][j] + Sktranspose_multi_QkP1[k][2] * Sk[2][j];
							}
						};
						for (k = 0; k < 3; k++) //???qk
						{
							for (j = 0; j < 3; j++)
							{
								Qk[k][j] = Sktranspose_multi_QkP1_multi_Sk[k][j] + Q[k][j];
								Qk[k][j] += Gk[0][k] * Rk * Gk[0][j];
							}
						};
						//line 115
						for (k = 0; k < 3; k++)
						{
							Dk[k][0] = Gk[0][k] * Rk * Hk +
									   Sktranspose_multi_QkP1[k][0] * Tk[0][0] + Sktranspose_multi_QkP1[k][1] * Tk[1][0] + Sktranspose_multi_QkP1[k][2] * Tk[0][2] +
									   Sktranspose[k][0] * DkP1[0][0] + Sktranspose[k][1] * DkP1[1][0] + Sktranspose[k][2] * DkP1[0][2] +
									   D[k][0];
						}
						//line 116
						double Tktranspose_multi_QkP1[1][3];
						for (k = 0; k < 3; k++) //
						{
							Tktranspose_multi_QkP1[0][k] = Tk[0][0] * QkP1[0][k] + Tk[1][0] * QkP1[1][k] + Tk[2][0] * QkP1[2][k];
						};
						Ek = Hk * Rk * Hk / 2 +
							 (Tktranspose_multi_QkP1[0][0] * Tk[0][0] + Tktranspose_multi_QkP1[0][1] * Tk[1][0] + Tktranspose_multi_QkP1[0][2] * Tk[0][2]) +
							 Hk * Fk +
							 (Tk[0][0] * DkP1[0][0] + Tk[1][0] * DkP1[1][0] + Tk[2][0] * DkP1[2][0]) +
							 EkP1 + E;
						//line 122
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								QkP1[k][j] = Qk[k][j];
							}
							DkP1[k][0] = Dk[k][0];
						};
						EkP1 = Ek;
					}
					//line 118
					for (i = 1; i <= step_total; i++)
					{
						k_istep = k_road[i - 1];
						lat_slope_istep = nearest_trajectory_info_lat_slope[i - 1];
						Speed_istep = state_vector_lonspeed[i - 1];
						Control_vector_new[i - 1] = GM[0][i - 1] * State_vector[0][i - 1] + GM[1][i - 1] * State_vector[1][i - 1] + GM[2][i - 1] * State_vector[2][i - 1] + HM[0][i - 1];

						A[0][0] = 1;
						A[0][1] = cos(lat_slope_istep) * nearest_trajectory_info_s[i - 1];
						A[0][2] = 0;
						A[1][0] = 0;
						A[1][1] = 1;
						A[1][2] = 1 / (Wheelbase_f + Wheelbase_r) * nearest_trajectory_info_s[i - 1];
						A[2][0] = 0;
						A[2][1] = 0;
						A[2][2] = 1 - 1 / Wheel_angle_delay / Speed_istep * nearest_trajectory_info_s[i - 1];
						B[0][0] = 0;
						B[1][0] = 0;
						B[2][0] = 1 / Wheel_angle_delay / Speed_istep * nearest_trajectory_info_s[i - 1];
						//line 128
						for (k = 0; k < 3; k++)
						{
							for (j = 0; j < 3; j++)
							{
								Ak[k][j] = A[k][j];
							}
							Bk[k][0] = B[k][0];
						};
						Ck[0][0] = 0;
						Ck[1][0] = -1 * k_istep * nearest_trajectory_info_s[i - 1];
						Ck[2][0] = 0;
						//line 135
						for (k = 0; k < 3; k++)
						{
							State_vector[k][i] = Ak[k][0] * State_vector[0][i - 1] + Ak[k][1] * State_vector[1][i - 1] + Ak[k][2] * State_vector[2][i - 1] +
												 Bk[k][0] * Control_vector_new[i - 1] +
												 Ck[k][0];
						}
						//line 138
						double time_istep;
						// time_istep=nearest_trajectory_info_time_tar[i-1];
						// if(fabs(nearest_trajectory_info_time_tar[i-1])<0.001)
						// {
						// 	time_istep=0.001;

						// }
						// if (i==1)
						// {   if ((Control_vector_new[i-1]-Wheel_angle0)/time_istep]<-Wheel_angle_change_rate_max)
						// 		{Control_vector_new[i-1]=Wheel_angle0-Wheel_angle_change_rate_max*time_istep;}
						// 	if ((Control_vector_new[i-1]-Wheel_angle0)/time_istep>Wheel_angle_change_rate_max)
						// 		{Control_vector_new[i-1]=Wheel_angle0+Wheel_angle_change_rate_max*time_istep;}
						// }
						// else
						// {    if ((Control_vector_new[i-1]-Control_vector_new[i-1-1])/time_istep<-Wheel_angle_change_rate_max)
						// 		{Control_vector_new[i-1]=Control_vector_new[i-1-1]-Wheel_angle_change_rate_max*time_istep;}
						// 	if ((Control_vector_new[i-1]-Control_vector_new[i-1-1])/time_istep]>Wheel_angle_change_rate_max)
						// 		{Control_vector_new[i-1]=Control_vector_new[i-1-1]+Wheel_angle_change_rate_max*time_istep;}

						// }

						//line 156
						if (Control_vector_new[i - 1] > Wheel_angle_max)
						{
							Control_vector_new[i - 1] = Wheel_angle_max;
						}
						if (Control_vector_new[i - 1] < -Wheel_angle_max)
						{
							Control_vector_new[i - 1] = -Wheel_angle_max;
						}
						//line 176
						for (k = 0; k < 3; k++)
						{
							State_vector[k][i] = Ak[k][0] * State_vector[0][i - 1] + Ak[k][1] * State_vector[1][i - 1] + Ak[k][2] * State_vector[2][i - 1] +
												 Bk[k][0] * Control_vector_new[i - 1] +
												 Ck[k][0];
						}
					}
					//line 155
					stp = 1;
					for (i = 1; i <= step_total; i++)
					{
						if (fabs(Control_vector[i - 1] - Control_vector_new[i - 1]) > 0.0001)
						{
							stp = 0;
						}
					}

					//line 164
					for (i = 1; i <= step_total; i++)
					{
						Control_vector[i - 1] = Control_vector_new[i - 1];
					}
					if (stp)
					{
						break;
					}
				}

				Wheel_angle_commanded_output = Control_vector[0];
				std::cout << "vector_lonspeed   wheelanglecom   stata_lat   stata_heading_error"<<std::endl;
				for (int j = 0; j <= step_total; j++)
				{
					std::cout << "number " << i << "  :" << state_vector_lonspeed[j] << "  " << Control_vector_new[j] << "  " << State_vector[0][j] << "  " << State_vector[1][j]<<std::endl;
				}
			}
			
			cmd->set_steering_target(Wheel_angle_commanded_output * Steer_Ratio * 180 / M_PI / 470 * 100);
			if (v0 < 1 & Acc_commanded_output < 0)//保证停车稳定
			{
				Acc_commanded_output = -1;
			}
			cmd->set_acceleration(Acc_commanded_output);

			std::cout << "  Acc_commanded_output " << Acc_commanded_output << "  Wheel_angle_commanded_output  " << Wheel_angle_commanded_output * 180 / 3.14<<std::endl;
		}

		Status EcarxLatController::Reset()
		{

			return Status::OK();
		}

	} // namespace control
} // namespace
*/